#pragma once

#include "steering.h"

typedef enum {
    STEER_ZERO_IDLE, STEER_ZERO_WAIT_DISABLED, STEER_ZERO_WAIT_FEEDBACK,
    STEER_ZERO_SUCCESS, STEER_ZERO_FAILED
} SteerCalibrationPhase_e;

typedef struct {
    SteerCalibrationPhase_e phase;
    uint8_t request_id, wheel, confirmed_mask, failed_mask;
    uint32_t since_ms;
} SteerCalibration_s;

/* disabled_mask 仅包含在线且反馈为失能的电机。返回 WHEEL_COUNT 表示本周期无需发送。 */
WheelIndex_e SteerCalibrationStep(SteerCalibration_s *cal, uint8_t permitted, uint8_t request, uint8_t disabled_mask,
                                  const float position_rad[], const uint32_t updated_ms[], uint32_t now);
void SteerCalibrationSent(SteerCalibration_s *cal, uint8_t queued, uint32_t now);
