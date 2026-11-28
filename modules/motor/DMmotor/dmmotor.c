#include "dmmotor.h"
#include "motor_def.h"
#include <math.h>
#include <string.h>

#define DM_MOTOR_COUNT 8u // 底盘4个转向电机 + 拨弹轮,预留余量

static DMMotorInstance motors[DM_MOTOR_COUNT];
static unsigned motor_count;

static float DecodeSigned(unsigned value, unsigned full_scale, float limit)
{
    return (2.0f * limit / full_scale) * value - limit;
}

/* 达妙 MIT 反馈 D[0] = (错误码 << 4) | (ESC_ID & 0x0F)，只有 ESC_ID ≤ 15 时
 * 高 4 位才是错误码。ESC_ID 更大时(本底盘用 0x51~0x54)D[0] 回的是整个字节的
 * ESC_ID，高位属于 ID，按错误码读会恒定得到 5(0x51>>4)。 */
static uint8_t DecodeErrorCode(const CANInstance *can)
{
    return can->tx_id < 0x10u ? (uint8_t)(can->rx_buff[0] >> 4) : 0u;
}

static void DecodeDMMotor(CANInstance *can)
{
    DMMotorInstance *motor = can->id;
    const uint8_t *data = can->rx_buff;
    if (can->rx_len != 8 || (data[0] & 0x0fu) != (can->tx_id & 0x0fu))
        return;

    DMMotorMeasure_s measure = {
        .state = DecodeErrorCode(can),
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

static uint8_t SendSpecialFrame(DMMotorInstance *motor, uint8_t command)
{
    CANInstance *can = motor->motor_can_instance;
    memset(can->tx_buff, 0xff, 7);
    can->tx_buff[7] = command;
    return CANTransmit(can, 1);
}

uint8_t DMMotorSetEnabled(DMMotorInstance *motor, uint8_t enabled)
{
    motor->enable_requested = enabled;
    motor->reference = (DMMotorPositionRef_s){0};
    return SendSpecialFrame(motor, enabled ? 0xfc : 0xfd);
}

uint8_t DMMotorSaveZero(DMMotorInstance *motor)
{
    /* 只要求"本驱动没有使能它"且已有反馈。不能用 measure.state 判断失能：
     * 那是错误码(且 ESC_ID>15 时根本不可用)，与使能状态无关。设零是否真的生效
     * 由调用方在后续反馈里核对位置是否归零。 */
    if (!motor || !motor->motor_can_instance || motor->enable_requested ||
        !motor->measure.valid)
        return 0;
    return SendSpecialFrame(motor, 0xfe);
}

/**
 * @brief 按达妙 MIT 协议打包并发送一帧控制报文
 *
 * @param p_des  位置给定(rad),范围±PMAX
 * @param v_des  速度给定(rad/s),范围±VMAX
 * @param kp     位置增益,范围0~500
 * @param kd     速度增益,范围0~5
 * @param t_ff   前馈力矩(N·m),范围±TMAX
 * @return uint8_t 1: 发送成功; 0: 参数非法或发送失败
 *
 * 12/16 位字段采用达妙官方 float_to_uint 的线性映射:
 *   uint = (value - min) / (max - min) * (2^bits - 1),
 * 因此零速度/零力矩编码为 2048(0x800)而不是0。字节打包顺序为
 *   D0-D1: p(16bit) | D2: v高8位 | D3: v低4位+kp高4位 | D4: kp低8位
 *   D5: kd高8位 | D6: kd低4位+t高4位 | D7: t低8位
 */
static uint8_t SendMITFrame(DMMotorInstance *motor, float p_des, float v_des, float kp, float kd, float t_ff)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return 0;

    const float p_limit = motor->limits.position_rad;
    const float v_limit = motor->limits.velocity_rad_s;
    const float t_limit = motor->limits.torque_nm;

    if (!isfinite(p_des) || !isfinite(v_des) || !isfinite(kp) || !isfinite(kd) || !isfinite(t_ff))
        return 0;
    if (fabsf(p_des) > p_limit || fabsf(v_des) > v_limit || fabsf(t_ff) > t_limit)
        return 0;
    if (!(kp >= 0.0f && kp <= 500.0f) || !(kd >= 0.0f && kd <= 5.0f))
        return 0;

    uint16_t p = (uint16_t)lroundf((p_des / p_limit + 1.0f) * 32767.5f);
    uint16_t v = (uint16_t)lroundf((v_des / v_limit + 1.0f) * 2047.5f);
    uint16_t kp_uint = (uint16_t)lroundf(kp * 4095.0f / 500.0f);
    uint16_t kd_uint = (uint16_t)lroundf(kd * 4095.0f / 5.0f);
    uint16_t t = (uint16_t)lroundf((t_ff / t_limit + 1.0f) * 2047.5f);

    uint8_t *data = motor->motor_can_instance->tx_buff;
    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0Fu) << 4) | (kp_uint >> 8));
    data[4] = (uint8_t)kp_uint;
    data[5] = (uint8_t)(kd_uint >> 4);
    data[6] = (uint8_t)(((kd_uint & 0x0Fu) << 4) | (t >> 8));
    data[7] = (uint8_t)t;
    return CANTransmit(motor->motor_can_instance, 1);
}

uint8_t DMMotorSendPosition(DMMotorInstance *motor, DMMotorPositionRef_s reference)
{
    if (motor == NULL)
        return 0;
    motor->reference = reference;
    return SendMITFrame(motor, reference.position_rad, 0.0f, reference.kp, reference.kd, 0.0f);
}

uint8_t DMMotorSendVelocity(DMMotorInstance *motor, float velocity_rad_s, float kd)
{
    if (motor == NULL)
        return 0;

    /* p_des 填当前反馈位置,避免 MIT 内部位置项在 KP=0 时的数值无意义 */
    float position = motor->measure.valid ? motor->measure.position_rad : 0.0f;
    LIMIT_MIN_MAX(position, -motor->limits.position_rad, motor->limits.position_rad);
    motor->reference = (DMMotorPositionRef_s){.position_rad = position, .kp = 0.0f, .kd = kd};
    return SendMITFrame(motor, position, velocity_rad_s, 0.0f, kd, 0.0f);
}
