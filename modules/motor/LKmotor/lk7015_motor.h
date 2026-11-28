#pragma once

#include "bsp_can.h"
#include "controller.h"

#define LK7015_GROUP_SIZE 4u

/* ==================== 广播力矩控制(0x280)下的速度环 ====================
 * 这批驱动只实现了广播力矩控制：一帧 0x280 带四轮的目标力矩值，所以速度闭环
 * 放在主控里做——测速取自 0x141~0x144 的回复帧，PID 输出就是下一帧的力矩码。
 *
 * 参数四个轮子共用一套，整定时只改下面这组宏：
 *   1. 先把 Ki、Kd 置 0，加大 Kp 到"跟得上、松手不振荡"；
 *   2. 再加 Ki 吃掉稳态误差，加到起步不迟滞、停转不超调；
 *   3. Kd 一般留 0：转速反馈只有 1dps 分辨率，微分会放大量化噪声。
 * 0x280 的控制量对 MF/MG 是转矩电流、对 MS 是开环电压，协议范围 ±2000。 */
#define LK7015_SPEED_KP              1.0f
#define LK7015_SPEED_KI              0.4f
#define LK7015_SPEED_KD              0.0f
#define LK7015_SPEED_DEADBAND_DPS    0.0f    /* 误差小于该值时不动作，静差抖动明显再调大 */
#define LK7015_TORQUE_LIMIT          2000.0f /* 单轮力矩限幅，占协议 ±2000 的 60% */
#define LK7015_TORQUE_INTEGRAL_LIMIT 800.0f  /* 积分限幅，堵转时防积分饱和 */
#define LK7015_FEEDBACK_TIMEOUT_MS   60u     /* 反馈超过该时间没更新就清零力矩输出 */

typedef struct {
    int16_t speed_dps;
    int16_t effort_raw;         /* 转矩电流 iq 原始值，标度随驱动型号(MF/MG)而定 */
    uint16_t encoder;
    int8_t temperature_c;
    uint32_t updated_ms;
    uint8_t valid;
} LK7015Measure_s;

typedef struct {
    volatile LK7015Measure_s measure;
    CANInstance *motor_can_instance;
    float speed_ref_dps;
    float torque_cmd;           /* 本周期实际下发的力矩码，调试时可直接观察 */
    PIDInstance speed_pid;
    uint8_t enable_requested;
} LK7015MotorInstance;

/* 同一总线的一个广播组拥有四个独立反馈实例和一个专用发送缓冲。 */
typedef struct {
    LK7015MotorInstance motor[LK7015_GROUP_SIZE];
    CANInstance sender;
} LK7015Group_s;

LK7015MotorInstance *LK7015MotorInit(LK7015Group_s *group, FDCAN_HandleTypeDef *bus, uint8_t node_id);
void LK7015MotorSetSpeed(LK7015MotorInstance *motor, float speed_dps);
/* 广播一帧 0x280：closed_loop=1 时四轮各跑一次速度环(PID)，0 时直接发 0 力矩
 * 保活（轮子自由，但仍能收到 0x141~0x144 反馈）。成功返回 1 */
uint8_t LK7015SpeedLoopStep(LK7015Group_s *group, uint8_t closed_loop);
uint8_t LK7015SetEnabled(LK7015Group_s *group, uint8_t enabled);
