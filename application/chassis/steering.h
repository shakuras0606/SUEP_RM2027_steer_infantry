#pragma once

#include <stdint.h>

/* 轮位下标固定为顺时针顺序: 左前 → 右前 → 右后 → 左后。
 * 该下标同时决定 wheels[]/targets[] 的下标、published_feedback 各掩码的位序,
 * 以及舵电机 CAN_ID 与轮毂节点号的编号(1/2/3/4),三者必须一致 */
typedef enum {
    WHEEL_FL, WHEEL_FR, WHEEL_RR, WHEEL_RL, WHEEL_COUNT
} WheelIndex_e;

typedef struct {
    float x_m, y_m;             /* x 向前，y 向左 */
    float zero_rad;             /* 朝前时舵轴读数(舵轴 rad, ±π) */
    float steer_ratio;          /* 含方向的舵轴转角 / 轮组转角, 通常 ±1(CHASSIS_x_STEER_DIR) */
    float drive_direction;      /* 轮毂转向极性 ±1(CHASSIS_x_DRIVE_DIR) */
} SwerveGeometry_s;

typedef struct {
    float steer_rad;            /* 舵轴目标角, rad(±π), 下发前换算成电机侧 rad */
    float speed_mps;
} SwerveTarget_s;

/* 低速时保留 target 中的上次舵角。越出位置量程则返回 0。 */
uint8_t SwerveSolveWheel(const SwerveGeometry_s *wheel, float vx, float vy, float wz,
                         float position_rad, float position_max_rad, SwerveTarget_s *target);
void SwerveLimitSpeeds(SwerveTarget_s targets[WHEEL_COUNT], float max_mps);
