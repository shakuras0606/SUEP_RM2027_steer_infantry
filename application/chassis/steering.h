#pragma once

#include <stdint.h>

typedef enum {
    WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR, WHEEL_COUNT
} WheelIndex_e;

typedef struct {
    float x_m, y_m;             /* x 向前，y 向左 */
    float zero_rad;             /* 朝前时舵电机输出轴读数 */
    float steer_ratio;          /* 含方向的输出轴转角 / 轮组转角 */
    float drive_direction;
} SwerveGeometry_s;

typedef struct {
    float steer_rad;
    float speed_mps;
} SwerveTarget_s;

/* 低速时保留 target 中的上次舵角。越出位置量程则返回 0。 */
uint8_t SwerveSolveWheel(const SwerveGeometry_s *wheel, float vx, float vy, float wz,
                         float position_rad, float position_max_rad, SwerveTarget_s *target);
void SwerveLimitSpeeds(SwerveTarget_s targets[WHEEL_COUNT], float max_mps);
