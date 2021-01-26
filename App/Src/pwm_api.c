#include "pwm_api.h"

#include "stdint.h"
#include "stdio.h"

#define RL_PWM_FREQ 168000000
#define POL_PWM_FREQ 168000000

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

static TIM_HandleTypeDef* RL_PWM_TIM = &htim10;
static TIM_HandleTypeDef* POL_PWM_TIM = &htim11;

static uint32_t FreToPrescaler(uint32_t _Fre)
{
    uint32_t Prescaler = 0;

    if (_Fre >= 11758) {
        Prescaler = 0;
    } else if (_Fre >= 783) {
        Prescaler = 9;
    } else if (_Fre >= 39) {
        Prescaler = 209;
    } else {
        Prescaler = 1679;
    }

    return Prescaler;
}

//横滚电机角速度设置
void MotoSpeedSetROLL(unsigned long int _Fre)
{
    const uint32_t SysClock = RL_PWM_FREQ;
    uint32_t f = 0;
    uint32_t Period = 0;
    uint32_t Prescaler = 0;
    f = _Fre;

    Prescaler = FreToPrescaler(_Fre);

    Period = (uint32_t)((SysClock / (Prescaler + 1))) / f;

    __HAL_TIM_SET_PRESCALER(RL_PWM_TIM, Prescaler);
    __HAL_TIM_SET_AUTORELOAD(RL_PWM_TIM, Period);
    __HAL_TIM_SET_COMPARE(RL_PWM_TIM, TIM_CHANNEL_1, Period / 2);
}

//极化电机角速度设置
void MotoSpeedSetPOL(unsigned long int _Fre)
{
    const uint32_t SysClock = POL_PWM_FREQ;
    uint32_t f = 0;
    uint32_t Period = 0;
    uint32_t Prescaler = 0;
    f = _Fre;

    Prescaler = FreToPrescaler(_Fre);

    Period = (uint32_t)((SysClock / (Prescaler + 1))) / f;

    __HAL_TIM_SET_PRESCALER(POL_PWM_TIM, Prescaler);
    __HAL_TIM_SET_AUTORELOAD(POL_PWM_TIM, Period);
    __HAL_TIM_SET_COMPARE(POL_PWM_TIM, TIM_CHANNEL_1, Period / 2);
}
