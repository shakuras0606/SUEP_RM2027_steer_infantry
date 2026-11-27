#include "shoot.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "robot_def.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_log.h"

static DJIMotorInstance *friction_l, *friction_r; // 左/右摩擦轮 (M3508 + C620)
static DMMotorInstance *loader;                   // 拨弹轮 (DM4310, MIT模式)

static Publisher_t *shoot_pub;                   // 发射应用消息发布者(发射反馈给cmd)
static Subscriber_t *shoot_sub;                  // cmd控制消息订阅者
static Shoot_Upload_Data_s shoot_feedback_data;  // 回传给cmd的发射状态信息
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;          // 来自cmd的控制信息

/* 拨弹轮MIT控制的内部状态 */
static uint8_t loader_enabled = 0;            // 拨弹轮是否已使能,避免每个周期重复发送使能帧
static loader_mode_e loader_last_mode = LOAD_STOP; // 上一次的拨弹模式,用于单发/三发的边沿触发
static float loader_target_rad = 0.0f;        // 单发/三发/保持的位置目标,单位rad
static float loader_position_ref = 0.0f;      // 最近一次有效的位置目标,反馈丢失时用作退化目标

/* 将角度限幅到 MIT 位置量程内(周期为2*PMAX的缠绕),避免单发累加后越界 */
static float LoaderWrapAngle(float angle)
{
    const float period = 2.0f * LOADER_PMAX;
    while (angle > LOADER_PMAX)
        angle -= period;
    while (angle < -LOADER_PMAX)
        angle += period;
    return angle;
}

/* 拨弹轮当前的位置目标: 优先用反馈,反馈无效时用上一次目标 */
static float LoaderCurrentPosition()
{
    if (loader->measure.valid)
        loader_position_ref = loader->measure.position_rad;
    return loader_position_ref;
}

void ShootInit()
{
    /* 摩擦轮: M3508 + C620, 一帧0x200报文可以控制4个电调,左右两个分别挂在ID1/ID2 */
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = SHOOT_MOTOR_CAN,
            .tx_id = SHOOT_FRICTION_L_CAN_ID, // 会被下面的右摩擦轮共用,实际发送由dji_motor分组完成
        },
        .controller_param_init_config = {
            /* 速度环输出直接作为电流指令(close_loop_type只开SPEED_LOOP),
             * 满量程16384对应20A,M3508转速上限约8000rpm=48000deg/s,
             * 下面的Kp/Ki是让摩擦轮能快速到达空载转速的初值,必须实测弹道后整定 */
            .speed_PID = {
                .Kp = 20,
                .Ki = 0.5,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut = 16384,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            /* 电流环暂不启用(未标定电流环增益,直接开会导致输出为0),
             * 速度环输出就是电调电流指令 */
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    friction_l = DJIMotorInit(&friction_config);

    // 右侧摩擦轮与左侧共用一个C620控制帧,只需修改ID和反转标志
    friction_config.can_init_config.tx_id = SHOOT_FRICTION_R_CAN_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    /* 拨弹轮: DM4310, MIT模式。MIT控制帧=0x300+ESC_ID, 反馈帧=MST_ID,
     * 量程必须与达妙调试助手中写入的 PMAX/VMAX/TMAX 完全一致 */
    CAN_Init_Config_s loader_can_config = {
        .can_handle = SHOOT_MOTOR_CAN,
        .tx_id = SHOOT_LOADER_DM_CTRL_ID,
        .rx_id = SHOOT_LOADER_DM_MST_ID,
    };
    DMMotorLimits_s loader_limits = {
        .position_rad = LOADER_PMAX,
        .velocity_rad_s = LOADER_VMAX,
        .torque_nm = LOADER_TMAX,
    };
    loader = DMMotorInit(loader_can_config, loader_limits);
    if (loader == NULL)
        LOGERROR("[shoot] DM4310 loader init failed, check DM_MOTOR_COUNT and id conflict");

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

void ShootTask()
{
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    /* ---------- 使能/失能: 只在状态切换时发送,避免总线被使能帧占满 ---------- */
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        if (loader != NULL && loader_enabled)
        {
            DMMotorSetEnabled(loader, 0);
            loader_enabled = 0;
        }
        loader_last_mode = LOAD_STOP; // 重新使能后按STOP重新走边沿
    }
    else
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        if (loader != NULL && !loader_enabled)
        {
            DMMotorSetEnabled(loader, 1);
            loader_enabled = 1;
            loader_last_mode = LOAD_STOP;
        }
    }

    /* ---------- 拨弹轮 ---------- */
    if (loader != NULL && shoot_cmd_recv.shoot_mode != SHOOT_OFF)
    {
        switch (shoot_cmd_recv.load_mode)
        {
        case LOAD_STOP: // 停在当前位置,保持拨盘不因弹丸挤压而回退
        {
            loader_target_rad = LoaderCurrentPosition();
            DMMotorSendPosition(loader, (DMMotorPositionRef_s){
                                            .position_rad = loader_target_rad,
                                            .kp = LOADER_POS_KP,
                                            .kd = LOADER_POS_KD,
                                        });
            break;
        }
        case LOAD_1_BULLET: // 单发: 仅在模式刚切换时累加目标角度,之后保持
        case LOAD_3_BULLET: // 三发
        {
            if (loader_last_mode != shoot_cmd_recv.load_mode)
            {
                const float bullets = (shoot_cmd_recv.load_mode == LOAD_1_BULLET) ? 1.0f : 3.0f;
                loader_target_rad =
                    LoaderWrapAngle(LoaderCurrentPosition() + bullets * ONE_BULLET_DELTA_ANGLE * DEGREE_2_RAD);
            }
            DMMotorSendPosition(loader, (DMMotorPositionRef_s){
                                            .position_rad = loader_target_rad,
                                            .kp = LOADER_POS_KP,
                                            .kd = LOADER_POS_KD,
                                        });
            break;
        }
        case LOAD_BURSTFIRE: // 连发: MIT速度模式, kp=0, 靠kd阻尼跟踪拨盘转速
            DMMotorSendVelocity(loader,
                                shoot_cmd_recv.shoot_rate * ONE_BULLET_DELTA_ANGLE * DEGREE_2_RAD,
                                LOADER_SPEED_KD);
            break;
        case LOAD_REVERSE: // 反转退弹
            DMMotorSendVelocity(loader, -LOADER_REVERSE_SPEED, LOADER_SPEED_KD);
            break;
        default: // 非法模式,急停拨弹轮
            LOGERROR("[shoot] invalid load_mode %d", shoot_cmd_recv.load_mode);
            DMMotorSetEnabled(loader, 0);
            loader_enabled = 0;
            break;
        }
        loader_last_mode = shoot_cmd_recv.load_mode;
    }

    /* ---------- 摩擦轮 ---------- */
    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF)
    {
        if (shoot_cmd_recv.friction_mode == FRICTION_ON)
        {
            float friction_speed = SHOOT_FRICTION_SPEED_DEFAULT;
            switch (shoot_cmd_recv.bullet_speed)
            {
            case SMALL_AMU_15:
                friction_speed = SHOOT_FRICTION_SPEED_15;
                break;
            case SMALL_AMU_18:
                friction_speed = SHOOT_FRICTION_SPEED_18;
                break;
            case SMALL_AMU_30:
                friction_speed = SHOOT_FRICTION_SPEED_30;
                break;
            default:
                LOGWARNING("[shoot] unknown bullet speed %d, use default", shoot_cmd_recv.bullet_speed);
                break;
            }
            DJIMotorSetRef(friction_l, friction_speed);
            DJIMotorSetRef(friction_r, friction_speed);
        }
        else // 摩擦轮停转
        {
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
        }
    }

    // 图传/弹仓盖(预留)
    switch (shoot_cmd_recv.lid_mode)
    {
    case LID_OPEN: // 打开弹仓盖
        break;
    case LID_CLOSE: // 关闭弹仓盖
        break;
    default:
        break;
    }

    // 推送消息
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
