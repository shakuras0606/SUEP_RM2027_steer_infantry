#pragma once

#include "bsp_can.h"

/* MIT 固件的量程必须与上位机配置一致。 */
typedef struct {
    float position_rad, velocity_rad_s, torque_nm;
} DMMotorLimits_s;

typedef struct {
    float position_rad, velocity_rad_s, torque_nm;
    uint8_t state, mos_temp_c, rotor_temp_c;
    uint32_t updated_ms;
    uint8_t valid;
} DMMotorMeasure_s;

typedef struct {
    float position_rad, kp, kd;
} DMMotorPositionRef_s;

typedef struct {
    volatile DMMotorMeasure_s measure;
    DMMotorPositionRef_s reference;
    DMMotorLimits_s limits;
    CANInstance *motor_can_instance;
    uint8_t enable_requested;
} DMMotorInstance;

DMMotorInstance *DMMotorInit(CAN_Init_Config_s can, DMMotorLimits_s limits);
uint8_t DMMotorSetEnabled(DMMotorInstance *motor, uint8_t enabled);
/* 发送 MIT 位置目标，速度和前馈力矩为零。零增益用于启动探测。 */
uint8_t DMMotorSendPosition(DMMotorInstance *motor, DMMotorPositionRef_s reference);
/* 发送 MIT 速度目标(KP=0,仅靠 KD 阻尼跟踪 v_des)，用于拨弹轮连发/反转等速度场合。
 * p_des 填当前反馈位置，因此需要先收到过一帧反馈(measure.valid)，否则按 0 位置发送。 */
uint8_t DMMotorSendVelocity(DMMotorInstance *motor, float velocity_rad_s, float kd);
