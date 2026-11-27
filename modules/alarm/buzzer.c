#include "bsp_pwm.h"
#include "buzzer.h"
#include <stdlib.h>
#include "string.h"

static PWMInstance *buzzer;
static volatile BuzzerResult_e pending_result;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

/**
 * @brief 蜂鸣器初始化
 *
 */
void BuzzerInit()
{
    PWM_Init_Config_s buzzer_config = {
        .htim = &htim12,
        .channel = TIM_CHANNEL_2,
        .dutyratio = 0,
        .period = 0.001,
    };
    buzzer = PWMRegister(&buzzer_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config->alarm_level >= BUZZER_DEVICE_CNT) // alarm_level 同时是数组下标
        while (1)
            ;
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    buzzer->alarm_state = state;
}

void BuzzerPlayResult(BuzzerResult_e result)
{
    pending_result = result;
}

void BuzzerTask()
{
    static const uint16_t alarm_freq[] = {DoFreq, ReFreq, MiFreq, FaFreq, SoFreq, LaFreq, SiFreq};
    static BuzzerResult_e playing;
    static uint32_t started_ms;
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        BuzzzerInstance *buzz = buzzer_list[i];
        if (buzz && buzz->alarm_state == ALARM_ON) {
            if ((unsigned)buzz->octave < sizeof(alarm_freq) / sizeof(alarm_freq[0]))
                PWMSetPeriod(buzzer, 1.0f / alarm_freq[buzz->octave]);
            PWMSetDutyRatio(buzzer, buzz->loudness);
            return;
        }
    }

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    BuzzerResult_e next = pending_result;
    pending_result = BUZZER_RESULT_NONE;
    __set_PRIMASK(mask);
    uint32_t now = HAL_GetTick();
    if (next != BUZZER_RESULT_NONE) {
        playing = next;
        started_ms = now;
    }
    uint32_t elapsed = now - started_ms;
    if (playing == BUZZER_RESULT_SUCCESS && elapsed < 280u) {
        PWMSetPeriod(buzzer, 1.0f / SiFreq);
        PWMSetDutyRatio(buzzer, elapsed < 100u || elapsed >= 180u ? 0.5f : 0.0f);
    } else if (playing == BUZZER_RESULT_FAILURE && elapsed < 420u) {
        PWMSetPeriod(buzzer, 1.0f / DoFreq);
        PWMSetDutyRatio(buzzer, 0.5f);
    } else {
        playing = BUZZER_RESULT_NONE;
        PWMSetDutyRatio(buzzer, 0);
    }
}
