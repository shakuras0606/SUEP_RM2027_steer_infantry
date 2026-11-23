#include "lk7015_motor.h"
#include <math.h>

#define LK7015_SPEED_ID 0x281u
#define LK7015_MODE_ID  0x288u

static uint16_t ReadLE16(const uint8_t *data)
{
    return (uint16_t)((data[1] << 8) | data[0]);
}

static void DecodeLK7015(CANInstance *can)
{
    LK7015MotorInstance *motor = can->id;
    const uint8_t *data = can->rx_buff;
    if (can->rx_len != 8 || data[0] != 0xa2)
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
        .tx_id = LK7015_SPEED_ID,
        .rx_id = 0x140u + node_id,
        .can_module_callback = DecodeLK7015,
        .id = motor,
    };
    motor->motor_can_instance = CANRegister(&config);
    group->sender.can_handle = bus;
    group->sender.txconf = motor->motor_can_instance->txconf;
    return motor;
}

void LK7015MotorSetSpeed(LK7015MotorInstance *motor, float speed_dps)
{
    motor->speed_ref_dps = speed_dps;
}

uint8_t LK7015SendSpeeds(LK7015Group_s *group)
{
    CANInstance *can = &group->sender;
    for (unsigned i = 0; i < LK7015_GROUP_SIZE; ++i) {
        float speed = group->motor[i].speed_ref_dps;
        if (!isfinite(speed))
            return 0;
        uint16_t value = (uint16_t)(int16_t)lroundf(fmaxf(-32768.0f, fminf(32767.0f, speed)));
        can->tx_buff[2 * i] = (uint8_t)value;
        can->tx_buff[2 * i + 1] = value >> 8;
    }
    can->tx_id = can->txconf.Identifier = LK7015_SPEED_ID;
    return CANTransmit(can, 1);
}

uint8_t LK7015SetEnabled(LK7015Group_s *group, uint8_t enabled)
{
    CANInstance *can = &group->sender;
    for (unsigned i = 0; i < LK7015_GROUP_SIZE; ++i) {
        LK7015MotorInstance *motor = &group->motor[i];
        motor->enable_requested = enabled;
        motor->speed_ref_dps = 0;
        can->tx_buff[2 * i] = motor->motor_can_instance ? (enabled ? 0x88 : 0x80) : 0;
        can->tx_buff[2 * i + 1] = 0;
    }
    can->tx_id = can->txconf.Identifier = LK7015_MODE_ID;
    return CANTransmit(can, 1);
}
