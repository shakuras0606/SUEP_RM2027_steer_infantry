#include "steer_calibration.h"
#include <math.h>

#define ALL_STEER_WHEELS ((1u << WHEEL_COUNT) - 1u)
#define ZERO_WAIT_MS 300u          // 等待失能或本轮设零后反馈的上限
#define ZERO_TOLERANCE_RAD 0.02f

static void Fail(SteerCalibration_s *cal, uint8_t mask)
{
    cal->phase = STEER_ZERO_FAILED;
    cal->failed_mask = mask;
}

WheelIndex_e SteerCalibrationStep(SteerCalibration_s *cal, uint8_t permitted, uint8_t request, uint8_t disabled_mask,
                                  const float position_rad[], const uint32_t updated_ms[], uint32_t now)
{
    if (cal->phase != STEER_ZERO_WAIT_DISABLED && cal->phase != STEER_ZERO_WAIT_FEEDBACK) {
        if (permitted && request && request != cal->request_id)
            *cal = (SteerCalibration_s){.phase = STEER_ZERO_WAIT_DISABLED, .request_id = request, .since_ms = now};
        return WHEEL_COUNT;
    }
    if (!permitted || (request && request != cal->request_id)) {
        Fail(cal, ALL_STEER_WHEELS & ~cal->confirmed_mask);
        return WHEEL_COUNT;
    }

    uint8_t bit = 1u << cal->wheel;
    uint32_t elapsed = now - cal->since_ms;
    if (cal->phase == STEER_ZERO_WAIT_DISABLED) {
        if (disabled_mask == ALL_STEER_WHEELS)
            return (WheelIndex_e)cal->wheel;
        if (elapsed >= ZERO_WAIT_MS)
            Fail(cal, ALL_STEER_WHEELS & ~disabled_mask);
    } else {
        /* 严格晚于入队时刻；同一毫秒内无法判定先后，等下次停机帧的反馈。 */
        if ((disabled_mask & bit) && now - updated_ms[cal->wheel] < elapsed &&
            fabsf(position_rad[cal->wheel]) <= ZERO_TOLERANCE_RAD) {
            cal->confirmed_mask |= bit;
            ++cal->wheel;
            cal->phase = cal->wheel == WHEEL_COUNT ? STEER_ZERO_SUCCESS : STEER_ZERO_WAIT_DISABLED;
            cal->since_ms = now;
        } else if (elapsed >= ZERO_WAIT_MS) {
            Fail(cal, bit);
        }
    }
    return WHEEL_COUNT;
}

void SteerCalibrationSent(SteerCalibration_s *cal, uint8_t queued, uint32_t now)
{
    if (!queued) {
        Fail(cal, 1u << cal->wheel);
        return;
    }
    cal->phase = STEER_ZERO_WAIT_FEEDBACK;
    cal->since_ms = now;
}
