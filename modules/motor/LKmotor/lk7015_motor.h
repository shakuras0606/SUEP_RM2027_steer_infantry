#pragma once

#include "bsp_can.h"

#define LK7015_GROUP_SIZE 4u

typedef struct {
    int16_t speed_dps;
    int16_t effort_raw;         /* 电流或功率原始值，取决于驱动型号 */
    uint16_t encoder;
    int8_t temperature_c;
    uint32_t updated_ms;
    uint8_t valid;
} LK7015Measure_s;

typedef struct {
    volatile LK7015Measure_s measure;
    CANInstance *motor_can_instance;
    float speed_ref_dps;
    uint8_t enable_requested;
} LK7015MotorInstance;

/* 同一总线的一个广播组拥有四个独立反馈实例和一个专用发送缓冲。 */
typedef struct {
    LK7015MotorInstance motor[LK7015_GROUP_SIZE];
    CANInstance sender;
} LK7015Group_s;

LK7015MotorInstance *LK7015MotorInit(LK7015Group_s *group, FDCAN_HandleTypeDef *bus, uint8_t node_id);
void LK7015MotorSetSpeed(LK7015MotorInstance *motor, float speed_dps);
uint8_t LK7015SendSpeeds(LK7015Group_s *group);
uint8_t LK7015SetEnabled(LK7015Group_s *group, uint8_t enabled);
