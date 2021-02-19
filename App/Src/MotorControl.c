#include "MotorControl.h"

#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

#include "ParametersDefine.h"
#include "Senser.h"
#include "main.h"
#include "pwm_api.h"

MOTOCTR MotoCtr;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

UART_HandleTypeDef* M1_COM = &huart1;
UART_HandleTypeDef* M2_COM = &huart2;

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

static TIM_HandleTypeDef* RL_PWM_TIM = &htim10;
static TIM_HandleTypeDef* POL_PWM_TIM = &htim11;

#define AZ_EN_PIN AZ_EN_Pin
#define AZ_EN_GPIO AZ_EN_GPIO_Port
#define EL_EN_PIN EL_EN_Pin
#define EL_EN_GPIO EL_EN_GPIO_Port
#define RL_EN_PIN RX_EN_Pin
#define RL_EN_GPIO RX_EN_GPIO_Port
#define POL_EN_PIN TX_EN_Pin
#define POL_EN_GPIO TX_EN_GPIO_Port

#define AZ_DIR_PIN AZ_DIR_Pin
#define AZ_DIR_GPIO AZ_DIR_GPIO_Port
#define EL_DIR_PIN EL_DIR_Pin
#define EL_DIR_GPIO EL_DIR_GPIO_Port
#define RL_DIR_PIN RX_DIR_Pin
#define RL_DIR_GPIO RX_DIR_GPIO_Port
#define POL_DIR_PIN TX_DIR_Pin
#define POL_DIR_GPIO TX_DIR_GPIO_Port

typedef enum {
    AZ = 1,
    EL = 2,
    RL = 3,
    POL = 4
} MOTO_t;

float uLimit(float val, float LMT)
{
    if (val >= LMT)
        return LMT;
    else if (val < -LMT)
        return -LMT;
    else
        return val;
}

//将角度转换为±180°范围内
float _180convert(float angle)
{
    if (fabs(angle) <= 180) {
        angle = angle;
    } else if (angle > 180) {
        angle = angle - 360;
    } else {
        angle = angle + 360;
    }
    return (angle);
}

//将角度转换为±10°范围内
float _10convert(float angle)
{
    const int limit = 10;
    if (fabs(angle) <= limit) {
        angle = angle;
    } else if (angle > limit) {
        angle = limit;
    } else {
        angle = -1 * limit;
    }
    return angle;
}

void MotoEnableCmd(MOTO_t moto, FunctionalState flag)
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;

    switch (moto) {
    case AZ:
        GPIO_Pin = AZ_EN_PIN;
        GPIOx = AZ_EN_GPIO;
        break;
    case EL:
        GPIO_Pin = EL_EN_PIN;
        GPIOx = EL_EN_GPIO;
        break;
    case RL:
        GPIO_Pin = RL_EN_PIN;
        GPIOx = RL_EN_GPIO;
        break;
    case POL:
        GPIO_Pin = POL_EN_PIN;
        GPIOx = POL_EN_GPIO;
        break;
    default:
        break;
    }
    if (flag == ENABLE) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    }
    return;
}

//方位电机使能
void MotoEnableAZ()
{
    HAL_UART_Transmit_DMA(M1_COM, (uint8_t*)"UM=2\r\n", sizeof("UM=2\r\n") - 1);
    HAL_UART_Transmit_DMA(M1_COM, (uint8_t*)"MO=1\r\n", sizeof("MO=1\r\n") - 1);
}

//方位电机去使能
void MotoDisableAZ()
{
    HAL_UART_Transmit_DMA(M1_COM, (uint8_t*)"MO=0\r\n", sizeof("MO=0\r\n") - 1);
}

//俯仰电机使能
void MotoEnableEL()
{
    HAL_UART_Transmit_DMA(M2_COM, (uint8_t*)"UM=2\r\n", sizeof("UM=2\r\n") - 1);
    HAL_UART_Transmit_DMA(M2_COM, (uint8_t*)"MO=1\r\n", sizeof("MO=1\r\n") - 1);
}

//俯仰电机去使能
void MotoDisableEL()
{
    HAL_UART_Transmit_DMA(M2_COM, (uint8_t*)"MO=0\r\n", sizeof("MO=0\r\n") - 1);
}

//横滚电机使能
void MotoEnableROLL()
{
    MotoEnableCmd(RL, ENABLE);
}

//横滚电机去使能
void MotoDisableROLL()
{
    MotoEnableCmd(RL, DISABLE);
}

//横滚电机角速度控制
void SpeedCtrROLL(float speed)
{
    float motoStep = 1.8; //电机步距
    float motoDiv = 16; //驱动器步距细分
    float ratGear = 25.6f; //齿轮齿数比
    float ratRdt = 1; //减速机减速比
    float speedMoto = 0; //负载端角速度对应到电机角速度
    unsigned long int _fre = 0; //电机端角速度对应的pwm频率

    ratGear = 1;
    speedMoto = fabs(speed * ratGear * ratRdt);

    if (fabs(speed) < 0.06) {
        _fre = 0;
    } else {
        _fre = (unsigned long int)((speedMoto) / (motoStep / motoDiv));
    }

    if (_fre == 0) {
        //当角速度为0时，频率为0
        //在计算PWM定时器预分频与周期时，频率作为除数
        //故当_fre为0时，需要做关闭定时器的特殊处理
        HAL_TIM_PWM_Stop(RL_PWM_TIM, TIM_CHANNEL_1);
    } else {
        MotoSpeedSetROLL(_fre);
        HAL_TIM_PWM_Start(RL_PWM_TIM, TIM_CHANNEL_1);
    }

    if (speed >= 0) {
        HAL_GPIO_WritePin(RL_DIR_GPIO, RL_DIR_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RL_DIR_GPIO, RL_DIR_PIN, GPIO_PIN_RESET);
    }
}

//极化电机使能
void MotoEnablePOL()
{
    MotoEnableCmd(POL, ENABLE);
}

//极化电机去使能
void MotoDisablePOL()
{
    MotoEnableCmd(POL, DISABLE);
}

//极化电机角速度控制
void SpeedCtrPOL(float speed)
{
    float motoStep = 1.8; //电机步距
    float motoDiv = 16; //驱动器步距细分
    float ratGear = 8; //齿轮齿数比
    float ratRdt = 1; //减速机减速比
    float speedMoto = 0; //负载端角速度对应到电机角速度
    unsigned long int _fre = 0; //电机端角速度对应的pwm频率

    ratGear = 1;
    speedMoto = fabs(speed * ratGear * ratRdt);

    _fre = (unsigned long int)(speedMoto / (motoStep / motoDiv));
    if (_fre == 0) {
        //当角速度为0时，频率为0
        //在计算PWM定时器预分频与周期时，频率作为除数
        //故当_fre为0时，需要做关闭定时器的特殊处理
        HAL_TIM_PWM_Stop(POL_PWM_TIM, TIM_CHANNEL_1);
    } else {
        MotoSpeedSetPOL(_fre);
        HAL_TIM_PWM_Start(POL_PWM_TIM, TIM_CHANNEL_1);
    }

    if (speed >= 0) {
        HAL_GPIO_WritePin(POL_DIR_GPIO, POL_DIR_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(POL_DIR_GPIO, POL_DIR_PIN, GPIO_PIN_RESET);
    }
}

#include "stdio.h"
void SpeedCtrElmoAZ(float speed)
{
    static char buff[40] = { 0 };
    uint8_t len = 0;
    int32_t sFre = 0;

    sFre = speed * (180 / 25.0) * 44 * 4096 / 60 * 60.0 / 360.0;

    len = sprintf(buff, "JV=%d\r\nBG\r\n", sFre);

    HAL_UART_Transmit_DMA(M1_COM, (uint8_t*)buff, len);
}

void SpeedCtrElmoEL(float speed)
{
    static char buff[40] = { 0 };
    uint8_t len = 0;
    int32_t sFre = 0;

    sFre = -speed * (130 / 25.0) * 30 * 4096 / 60 * 60.0 / 360.0;

    len = sprintf(buff, "JV=%d\r\nBG\r\n", sFre);

    HAL_UART_Transmit_DMA(M2_COM, (uint8_t*)buff, len);
}

// 速度控制
void SpeedCtrl()
{
    SpeedCtrElmoAZ(MotoCtr.AZSpeed);
    SpeedCtrElmoEL(MotoCtr.ESpeed);

    SpeedCtrROLL(MotoCtr.ROLLSpeed);
    SpeedCtrPOL(MotoCtr.POLSpeed);
}
