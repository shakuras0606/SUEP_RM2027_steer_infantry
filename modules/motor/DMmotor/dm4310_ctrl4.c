#include "dm4310_ctrl4.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static DM4310Ctrl4Instance *dm4310_instance[DM4310_CTRL4_MOTOR_CNT] = {NULL};
static uint8_t idx = 0; // 已注册电机数

/* 每个控制组(0x3FE/0x4FE)共用一个发送 CAN 实例,组内4个电机写同一份 tx_buff */
static CANInstance *group_sender[2] = {NULL, NULL};
static uint8_t group_enable_flag[2] = {0, 0};

/**
 * @brief 根据电机ID计算控制组下标
 *
 * @param motor_id 电机ID 1~8
 * @return uint8_t 0: 0x3FE(ID1~4), 1: 0x4FE(ID5~8)
 */
static uint8_t GetGroupIndex(uint8_t motor_id)
{
    return motor_id <= 4 ? 0 : 1;
}

/**
 * @brief 根据电机ID计算其在控制帧中的 slot 下标
 *
 * @param motor_id 电机ID 1~8
 * @return uint8_t 0~3
 */
static uint8_t GetSlotIndex(uint8_t motor_id)
{
    return (uint8_t)((motor_id - 1) % 4);
}

/**
 * @brief CAN 发送帧 ID 检查,用于初始化时给用户更明确的错误信息
 */
static uint8_t IsValidMotorId(uint8_t motor_id)
{
    return motor_id >= 1 && motor_id <= 8;
}

static void DM4310Ctrl4LostCallback(void *motor_ptr)
{
    DM4310Ctrl4Instance *motor = (DM4310Ctrl4Instance *)motor_ptr;
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;
    /* 直接内联总线号,避免关闭日志系统时出现 unused variable 警告 */
    LOGWARNING("[dm4310_ctrl4] Motor lost, can bus [%d], motor id [%d]",
               motor->motor_can_instance->can_handle == &hcan1 ? 1
                                                               : (motor->motor_can_instance->can_handle == &hcan2 ? 2 : 3),
               motor->motor_id);
}

static void DecodeDM4310Ctrl4(CANInstance *can)
{
    DM4310Ctrl4Instance *motor = (DM4310Ctrl4Instance *)can->id;
    if (motor == NULL || can->rx_len != 8)
        return;
    /* bsp_can 已按 rx_id 精确匹配,这里再校验一次,避免注册错乱时解错报文 */
    if (can->rx_id != (uint32_t)DM4310_CTRL4_FEEDBACK_ID_BASE + motor->motor_id)
        return;

    const uint8_t *data = can->rx_buff;
    DM4310Ctrl4_Measure_s *measure = &motor->measure;

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]) & 0x1FFFu;
    measure->angle_single_round = (float)measure->ecd * DM4310_CTRL4_ECD_ANGLE_COEF;

    /* 速度: 高8位在前,有符号,单位 rpm×100;×0.06 换算为度/秒并做低通 */
    int16_t speed_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    float speed_aps = (float)speed_raw * DM4310_CTRL4_SPEED_2_ANGLE_PER_SEC;
    measure->speed_aps = (1.0f - DM4310_CTRL4_SPEED_SMOOTH_COEF) * measure->speed_aps +
                         DM4310_CTRL4_SPEED_SMOOTH_COEF * speed_aps;

    /* 电流: 高8位在前,有符号,单位 mA */
    int16_t current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    measure->current_mA = (1.0f - DM4310_CTRL4_CURRENT_SMOOTH_COEF) * measure->current_mA +
                          DM4310_CTRL4_CURRENT_SMOOTH_COEF * (float)current_raw;

    measure->temperature = data[6];
    measure->error_state = data[7];

    /* 多圈计数: 8192格一圈,跨越半圈(4096)判断方向,与 dji_motor 的 total_angle 语义一致 */
    if (!measure->valid)
    {
        /* 首帧位置未知,直接初始化 last_ecd/total_round,避免上电第一帧产生一圈偏置 */
        measure->last_ecd = measure->ecd;
        measure->total_round = 0;
    }
    else if (measure->ecd - measure->last_ecd > 4096)
    {
        measure->total_round--;
    }
    else if (measure->ecd - measure->last_ecd < -4096)
    {
        measure->total_round++;
    }
    measure->total_angle = (float)measure->total_round * 360.0f + measure->angle_single_round;

    measure->updated_ms = HAL_GetTick();
    measure->valid = 1;
}

/**
 * @brief 初始化时的安全检查,尽早暴露配置错误而不是让电机乱转/任务死锁
 */
static uint8_t CheckInitConfig(Motor_Init_Config_s *config)
{
    Motor_Control_Setting_s *setting = &config->controller_setting_init_config;
    Motor_Controller_Init_s *controller = &config->controller_param_init_config;

    if ((setting->close_loop_type & ANGLE_LOOP) && setting->angle_feedback_source == OTHER_FEED &&
        controller->other_angle_feedback_ptr == NULL)
    {
        LOGERROR("[dm4310_ctrl4] angle loop uses OTHER_FEED but other_angle_feedback_ptr is NULL");
        return 0;
    }
    if ((setting->close_loop_type & SPEED_LOOP) && setting->speed_feedback_source == OTHER_FEED &&
        controller->other_speed_feedback_ptr == NULL)
    {
        LOGERROR("[dm4310_ctrl4] speed loop uses OTHER_FEED but other_speed_feedback_ptr is NULL");
        return 0;
    }
    if ((setting->feedforward_flag & SPEED_FEEDFORWARD) && controller->speed_feedforward_ptr == NULL)
    {
        LOGERROR("[dm4310_ctrl4] speed feedforward enabled but speed_feedforward_ptr is NULL");
        return 0;
    }
    if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) && controller->current_feedforward_ptr == NULL)
    {
        LOGERROR("[dm4310_ctrl4] current feedforward enabled but current_feedforward_ptr is NULL");
        return 0;
    }
    /* 外层闭环必须包含在启用的闭环中,否则参考值不会经过任何 PID 直接被当作电流发送 */
    if (setting->outer_loop_type != OPEN_LOOP && !(setting->outer_loop_type & setting->close_loop_type))
    {
        LOGERROR("[dm4310_ctrl4] outer_loop_type is not included in close_loop_type, check config");
        return 0;
    }
    return 1;
}

DM4310Ctrl4Instance *DM4310Ctrl4Init(Motor_Init_Config_s *config)
{
    if (config == NULL || config->can_init_config.can_handle == NULL)
    {
        LOGERROR("[dm4310_ctrl4] init failed: config or can_handle is NULL");
        return NULL;
    }

    /* 约定 tx_id 传入电压机ID,不是 CAN 发送 ID */
    uint8_t motor_id = (uint8_t)config->can_init_config.tx_id;
    if (!IsValidMotorId(motor_id))
    {
        LOGERROR("[dm4310_ctrl4] invalid motor id [%d], must be 1~8", motor_id);
        return NULL;
    }
    if (idx >= DM4310_CTRL4_MOTOR_CNT)
    {
        LOGERROR("[dm4310_ctrl4] instance exceeded MAX num [%d]", (int)DM4310_CTRL4_MOTOR_CNT);
        return NULL;
    }
    if (!CheckInitConfig(config))
        return NULL;

    /* 同一总线上的同ID电机必然抢同一反馈帧,提前报错,否则 bsp_can 会死循环 */
    for (size_t i = 0; i < idx; ++i)
    {
        if (dm4310_instance[i]->motor_id == motor_id &&
            dm4310_instance[i]->motor_can_instance->can_handle == config->can_init_config.can_handle)
        {
            LOGERROR("[dm4310_ctrl4] motor id [%d] crash on the same CAN bus", motor_id);
            return NULL;
        }
    }

    DM4310Ctrl4Instance *instance = (DM4310Ctrl4Instance *)malloc(sizeof(DM4310Ctrl4Instance));
    if (instance == NULL)
    {
        LOGERROR("[dm4310_ctrl4] malloc failed");
        return NULL;
    }
    memset(instance, 0, sizeof(DM4310Ctrl4Instance));

    instance->motor_id = motor_id;
    instance->motor_settings = config->controller_setting_init_config;
    instance->sender_group = GetGroupIndex(motor_id);
    instance->message_num = GetSlotIndex(motor_id);

    /* 三环 PID 与额外反馈指针,复用框架的通用控制器结构 */
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;

    /* CAN 注册: tx_id 必须为控制组帧ID(0x3FE/0x4FE), rx_id 为自身反馈帧ID(0x300+ID) */
    config->can_init_config.tx_id = instance->sender_group ? DM4310_CTRL4_GROUP_HIGH_CAN_ID
                                                           : DM4310_CTRL4_GROUP_LOW_CAN_ID;
    config->can_init_config.rx_id = (uint32_t)DM4310_CTRL4_FEEDBACK_ID_BASE + motor_id;
    config->can_init_config.can_module_callback = DecodeDM4310Ctrl4;
    config->can_init_config.id = instance;
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    /* 组内第一个电机作为该控制组的发送实例,其余电机共享它的 tx_buff */
    if (group_sender[instance->sender_group] == NULL)
    {
        group_sender[instance->sender_group] = instance->motor_can_instance;
        memset(group_sender[instance->sender_group]->tx_buff, 0, 8);
        group_enable_flag[instance->sender_group] = 1;
    }
    instance->tx_sender = group_sender[instance->sender_group];

    Daemon_Init_Config_s daemon_config = {
        .callback = DM4310Ctrl4LostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到反馈则判定为离线
    };
    instance->daemon = DaemonRegister(&daemon_config);

    instance->stop_flag = MOTOR_ENALBED;
    dm4310_instance[idx++] = instance;

    LOGINFO("[dm4310_ctrl4] motor id %d registered, ctrl id 0x%03X, feedback id 0x%03X, slot %d",
            motor_id, (unsigned)(instance->sender_group ? DM4310_CTRL4_GROUP_HIGH_CAN_ID
                                                        : DM4310_CTRL4_GROUP_LOW_CAN_ID),
            (unsigned)(DM4310_CTRL4_FEEDBACK_ID_BASE + motor_id), instance->message_num);
    return instance;
}

void DM4310Ctrl4SetRef(DM4310Ctrl4Instance *motor, float ref)
{
    if (motor == NULL)
    {
        LOGERROR("[dm4310_ctrl4] SetRef: motor is NULL");
        return;
    }
    motor->motor_controller.pid_ref = ref;
}

void DM4310Ctrl4ChangeFeed(DM4310Ctrl4Instance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (motor == NULL)
    {
        LOGERROR("[dm4310_ctrl4] ChangeFeed: motor is NULL");
        return;
    }
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dm4310_ctrl4] ChangeFeed: loop type error, only ANGLE_LOOP/SPEED_LOOP are supported");
}

void DM4310Ctrl4OuterLoop(DM4310Ctrl4Instance *motor, Closeloop_Type_e outer_loop)
{
    if (motor == NULL)
    {
        LOGERROR("[dm4310_ctrl4] OuterLoop: motor is NULL");
        return;
    }
    motor->motor_settings.outer_loop_type = outer_loop;
}

void DM4310Ctrl4Stop(DM4310Ctrl4Instance *motor)
{
    if (motor == NULL)
    {
        LOGERROR("[dm4310_ctrl4] Stop: motor is NULL");
        return;
    }
    motor->stop_flag = MOTOR_STOP;
}

void DM4310Ctrl4Enable(DM4310Ctrl4Instance *motor)
{
    if (motor == NULL)
    {
        LOGERROR("[dm4310_ctrl4] Enable: motor is NULL");
        return;
    }
    motor->stop_flag = MOTOR_ENALBED;
}

void DM4310Ctrl4Control(void)
{
    if (idx == 0)
        return;

    DM4310Ctrl4Instance *motor;
    Motor_Control_Setting_s *motor_setting;
    Motor_Controller_s *motor_controller;
    DM4310Ctrl4_Measure_s *measure;
    float pid_measure, pid_ref;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = dm4310_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;

        pid_ref = motor_controller->pid_ref; // 保存参考值,防止计算过程中被上层任务修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        /* 角度环: 仅当启用角度环且外层闭环为角度环时计算 */
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            pid_measure = (motor_setting->angle_feedback_source == OTHER_FEED)
                              ? *motor_controller->other_angle_feedback_ptr
                              : measure->total_angle;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        /* 速度环: 外层为角度环或速度环时计算 */
        if ((motor_setting->close_loop_type & SPEED_LOOP) &&
            (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            pid_measure = (motor_setting->speed_feedback_source == OTHER_FEED)
                              ? *motor_controller->other_speed_feedback_ptr
                              : measure->speed_aps;
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        /* 电流环: 若启用则用电机自身电流反馈;一控四固件内部已有电流环,一般不需要再启用 */
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->current_mA, pid_ref);

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        /* 输出为标幺电流,限制在±10000(对应±Imax) */
        LIMIT_MIN_MAX(pid_ref, -DM4310_CTRL4_CURRENT_MAX, DM4310_CTRL4_CURRENT_MAX);
        int16_t set = (int16_t)lroundf(pid_ref);

        /* 停机时直接下发0电流,不依赖 PID 收敛 */
        if (motor->stop_flag == MOTOR_STOP)
            set = 0;

        /* 一控四控制帧为低字节在前,与反馈帧的高字节在前相反 */
        CANInstance *sender = motor->tx_sender;
        uint8_t slot = motor->message_num;
        sender->tx_buff[2 * slot] = (uint8_t)((uint16_t)set & 0x00FFu);
        sender->tx_buff[2 * slot + 1] = (uint8_t)(((uint16_t)set >> 8) & 0x00FFu);
    }

    /* 每个控制组每周期只发送一帧,4个电机的电流并行下发 */
    for (size_t group = 0; group < 2; ++group)
    {
        if (group_enable_flag[group] && group_sender[group] != NULL)
            CANTransmit(group_sender[group], 1);
    }
}
