#include "dmmotor.h"
#include <math.h>
#include <string.h>

#define DM_MOTOR_COUNT 5u

static DMMotorInstance motors[DM_MOTOR_COUNT];
static unsigned motor_count;

static float DecodeSigned(unsigned value, unsigned full_scale, float limit)
{
    return (2.0f * limit / full_scale) * value - limit;
}

static void DecodeDMMotor(CANInstance *can)
{
    DMMotorInstance *motor = can->id;
    const uint8_t *data = can->rx_buff;
    if (can->rx_len != 8 || (data[0] & 0x0fu) != (can->tx_id & 0x0fu))
        return;

    DMMotorMeasure_s measure = {
        .state = data[0] >> 4,
        .position_rad = DecodeSigned((data[1] << 8) | data[2], 65535, motor->limits.position_rad),
        .velocity_rad_s = DecodeSigned((data[3] << 4) | (data[4] >> 4), 4095, motor->limits.velocity_rad_s),
        .torque_nm = DecodeSigned(((data[4] & 15u) << 8) | data[5], 4095, motor->limits.torque_nm),
        .mos_temp_c = data[6],
        .rotor_temp_c = data[7],
        .updated_ms = HAL_GetTick(),
        .valid = 1,
    };
    motor->measure = measure;
}

DMMotorInstance *DMMotorInit(CAN_Init_Config_s can, DMMotorLimits_s limits)
{
    if (motor_count >= DM_MOTOR_COUNT || !(limits.position_rad > 0.0f) ||
        !(limits.velocity_rad_s > 0.0f) || !(limits.torque_nm > 0.0f))
        return NULL;

    DMMotorInstance *motor = &motors[motor_count++];
    motor->limits = limits;
    can.id = motor;
    can.can_module_callback = DecodeDMMotor;
    motor->motor_can_instance = CANRegister(&can);
    return motor;
}

uint8_t DMMotorSetEnabled(DMMotorInstance *motor, uint8_t enabled)
{
    CANInstance *can = motor->motor_can_instance;
    memset(can->tx_buff, 0xff, 7);
    can->tx_buff[7] = enabled ? 0xfc : 0xfd;
    motor->enable_requested = enabled;
    motor->reference = (DMMotorPositionRef_s){0};
    return CANTransmit(can, 1);
}

uint8_t DMMotorSendPosition(DMMotorInstance *motor, DMMotorPositionRef_s reference)
{
    float limit = motor->limits.position_rad;
    if (!isfinite(reference.position_rad) || fabsf(reference.position_rad) > limit ||
        !(reference.kp >= 0.0f && reference.kp <= 500.0f) ||
        !(reference.kd >= 0.0f && reference.kd <= 5.0f))
        return 0;

    motor->reference = reference;
    uint16_t p = (uint16_t)lroundf((reference.position_rad / limit + 1.0f) * 32767.5f);
    uint16_t kp = (uint16_t)lroundf(reference.kp * 4095.0f / 500.0f);
    uint16_t kd = (uint16_t)lroundf(reference.kd * 4095.0f / 5.0f);
    uint8_t *data = motor->motor_can_instance->tx_buff;
    data[0] = p >> 8;
    data[1] = (uint8_t)p;
    data[2] = 0x80;                      /* 目标速度为零，编码 0x800 */
    data[3] = kp >> 8;
    data[4] = (uint8_t)kp;
    data[5] = kd >> 4;
    data[6] = (uint8_t)((kd << 4) | 0x08u);
    data[7] = 0;                         /* 前馈力矩为零，编码 0x800 */
    return CANTransmit(motor->motor_can_instance, 1);
}
