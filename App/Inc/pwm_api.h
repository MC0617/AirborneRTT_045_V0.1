#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

//方位正交解码模块定时器初始化
void ENCODER_INIT_AZ(void);
//俯仰正交解码模块定时器初始化
void ENCODER_INIT_EL(void);
//横滚正交解码模块定时器初始化
void ENCODER_INIT_ROLL(void);
//极化正交解码模块定时器初始化
void ENCODER_INIT_POL(void);
//方位驱动器控制PWM初始化
void PWM_INIT_AZ(void);
//俯仰驱动器控制PWM初始化
void PWM_INIT_EL(void);
//横滚驱动器控制PWM初始化
void PWM_INIT_POL(void);
//极化驱动器控制PWM初始化
void PWM_INIT_ROLL(void);

//方位电机角速度设置
void MotoSpeedSetAZ(unsigned long int _Fre);
//俯仰电机角速度设置
void MotoSpeedSetEL(unsigned long int _Fre);
//横滚电机角速度设置
void MotoSpeedSetROLL(unsigned long int _Fre);
//极化电机角速度设置
void MotoSpeedSetPOL(unsigned long int _Fre);
