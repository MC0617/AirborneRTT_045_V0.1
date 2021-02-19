#ifndef __MEMS_H__
#define __MEMS_H__

#include "stm32f4xx_hal.h"
#include "Senser.h"

extern uint8_t IN_GetBuffer[MEMS_RECV_LENGTH];
extern volatile int IN_RX_Completed;

//惯导航向校准
void AmandMemsNew(void);

//惯导零位校准
void MemsZeroCheck(void);

//中断接收返回数据
extern uint8_t MEMS_CH[1];
void MEMS_UART_RecvIT(UART_HandleTypeDef* huart);

#endif // !__MEMS_H__
