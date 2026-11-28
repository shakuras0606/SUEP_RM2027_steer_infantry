#include "lk7015_motor.h"
#include <math.h>

#define LK7015_TORQUE_ID 0x280u
#define LK7015_MODE_ID   0x288u

/* PIDInit 按 PID_Init_Config_s 的长度整段拷进 PIDInstance 开头，两者字段顺序一致。
 * 四个轮子共用这一份参数，各自持有独立的 PIDInstance(积分量互不影响)。 */
static PID_Init_Config_s speed_pid_config = {
    .Kp = LK7015_SPEED_KP,
    .Ki = LK7015_SPEED_KI,
    .Kd = LK7015_SPEED_KD,
    .MaxOut = LK7015_TORQUE_LIMIT,
    .DeadBand = LK7015_SPEED_DEADBAND_DPS,
    .Improve = PID_Integral_Limit,
    .IntegralLimit = LK7015_TORQUE_INTEGRAL_LIMIT,
};

static uint16_t ReadLE16(const uint8_t *data)
{
    return (uint16_t)((data[1] << 8) | data[0]);
}

static void ResetSpeedLoop(LK7015MotorInstance *motor)
{
    PIDInit(&motor->speed_pid, &speed_pid_config);
    motor->torque_cmd = 0.0f;
}

static void DecodeLK7015(CANInstance *can)
{
    LK7015MotorInstance *motor = can->id;
    const uint8_t *data = can->rx_buff;
    /* 0xA0 开环、0xA1 力矩闭环、0xA2 速度闭环、0x9C 读状态2 的回复布局相同；
     * 现在用 0x280 广播力矩，驱动回的是 0xA1 */
    if (can->rx_len != 8 ||
        (data[0] != 0xa0 && data[0] != 0xa1 && data[0] != 0xa2 && data[0] != 0x9c))
        return;

    LK7015Measure_s measure = {
        .temperature_c = (int8_t)data[1],
        .effort_raw = (int16_t)ReadLE16(data + 2),
        .speed_dps = (int16_t)ReadLE16(data + 4),
        .encoder = ReadLE16(data + 6),
        .updated_ms = HAL_GetTick(),
        .valid = 1,
    };
    motor->measure = measure;
}

LK7015MotorInstance *LK7015MotorInit(LK7015Group_s *group, FDCAN_HandleTypeDef *bus, uint8_t node_id)
{
    if (node_id < 1 || node_id > LK7015_GROUP_SIZE ||
        (group->sender.can_handle && group->sender.can_handle != bus))
        return NULL;

    LK7015MotorInstance *motor = &group->motor[node_id - 1];
    if (motor->motor_can_instance)
        return NULL;

    CAN_Init_Config_s config = {
        .can_handle = bus,
        .tx_id = LK7015_TORQUE_ID,
        .rx_id = 0x140u + node_id,
        .can_module_callback = DecodeLK7015,
        .id = motor,
    };
    motor->motor_can_instance = CANRegister(&config);
    ResetSpeedLoop(motor);
    group->sender.can_handle = bus;
    group->sender.txconf = motor->motor_can_instance->txconf;
    return motor;
}

void LK7015MotorSetSpeed(LK7015MotorInstance *motor, float speed_dps)
{
    motor->speed_ref_dps = speed_dps;
}

uint8_t LK7015SpeedLoopStep(LK7015Group_s *group, uint8_t closed_loop)
{
    CANInstance *can = &group->sender;
    uint32_t now = HAL_GetTick();

    for (unsigned i = 0; i < LK7015_GROUP_SIZE; ++i) {
        LK7015MotorInstance *motor = &group->motor[i];
        float torque = 0.0f;

        if (closed_loop && isfinite(motor->speed_ref_dps) && motor->measure.valid &&
            now - motor->measure.updated_ms <= LK7015_FEEDBACK_TIMEOUT_MS) {
            torque = PIDCalculate(&motor->speed_pid, (float)motor->measure.speed_dps, motor->speed_ref_dps);
            if (!isfinite(torque)) {
                torque = 0.0f;
                ResetSpeedLoop(motor);
            }
        } else {
            /* 本轮无力、给定无效或反馈超时，只清本轮的力矩和积分。 */
            ResetSpeedLoop(motor);
        }

        motor->torque_cmd = torque;
        uint16_t value = (uint16_t)(int16_t)lroundf(fmaxf(-LK7015_TORQUE_LIMIT, fminf(LK7015_TORQUE_LIMIT, torque)));
        can->tx_buff[2 * i] = (uint8_t)(value & 0xffu);
        can->tx_buff[2 * i + 1] = (uint8_t)(value >> 8);
    }

    can->tx_id = can->txconf.Identifier = LK7015_TORQUE_ID;
    return CANTransmit(can, 1);
}

uint8_t LK7015SetEnabled(LK7015Group_s *group, uint8_t enabled)
{
    CANInstance *can = &group->sender;
    for (unsigned i = 0; i < LK7015_GROUP_SIZE; ++i) {
        LK7015MotorInstance *motor = &group->motor[i];
        if (motor->enable_requested != enabled)
            ResetSpeedLoop(motor); /* 启停切换时丢掉上一段闭环的积分 */
        motor->enable_requested = enabled;
        motor->speed_ref_dps = 0;
        can->tx_buff[2 * i] = motor->motor_can_instance ? (enabled ? 0x88 : 0x80) : 0;
        can->tx_buff[2 * i + 1] = 0;
    }
    can->tx_id = can->txconf.Identifier = LK7015_MODE_ID;
    return CANTransmit(can, 1);
}
