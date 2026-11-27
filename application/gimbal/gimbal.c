#include "gimbal.h"
#include "robot_def.h"
#include "dm4310_ctrl4.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DM4310Ctrl4Instance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    /* 一控四固件下 yaw/pitch 的 PID 输出已经不再是6020的电压值,而是标幺电流(±10000=±Imax),
     * 因此速度环的 Kp/Ki/MaxOut 按 6020 的 20000 满量程折算为 10000,下面的参数为折算初值,
     * 仍需按实际负载整定(重点: yaw 的 Kp/Ki、pitch 的重力补偿/积分) */
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = GIMBAL_MOTOR_CAN,
            .tx_id = GIMBAL_YAW_MOTOR_ID, // 电机ID,驱动内部会换算为控制帧0x3FE
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500, // 角度环输出为速度参考(度/秒)
            },
            .speed_PID = {
                .Kp = 25,  // 25(6020为50,电流满量程减半)
                .Ki = 100, // 100(6020为200)
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1500, // 6020为3000
                .MaxOut = 10000,       // 一控四标幺电流满量程
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 一控四电流方向与IMU正方向相反时改为REVERSE
        },
    };
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = GIMBAL_MOTOR_CAN,
            .tx_id = GIMBAL_PITCH_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500, // 角度环输出为速度参考(度/秒)
            },
            .speed_PID = {
                .Kp = 25,  // 25(6020为50,电流满量程减半)
                .Ki = 175, // 175(6020为350)
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1250, // 6020为2500
                .MaxOut = 10000,       // 一控四标幺电流满量程
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    // 电机对IMU姿态闭环,上电时保持静止,收到遥控器数据再动
    yaw_motor = DM4310Ctrl4Init(&yaw_config);
    pitch_motor = DM4310Ctrl4Init(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DM4310Ctrl4Stop(yaw_motor);
        DM4310Ctrl4Stop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DM4310Ctrl4Enable(yaw_motor);
        DM4310Ctrl4Enable(pitch_motor);
        DM4310Ctrl4ChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DM4310Ctrl4ChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DM4310Ctrl4ChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DM4310Ctrl4ChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DM4310Ctrl4SetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DM4310Ctrl4SetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DM4310Ctrl4Enable(yaw_motor);
        DM4310Ctrl4Enable(pitch_motor);
        DM4310Ctrl4ChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DM4310Ctrl4ChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DM4310Ctrl4ChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DM4310Ctrl4ChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DM4310Ctrl4SetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DM4310Ctrl4SetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
