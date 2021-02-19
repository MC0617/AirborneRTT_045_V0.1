#include "mems.h"

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "string.h"

#include "MotorControl.h"
#include "Parameters.h"
#include "Senser.h"
#include "beacon.h"
#include "thread_ctrl.h"

extern UART_HandleTypeDef huart5;

UART_HandleTypeDef* IN_COM = &huart5;

uint8_t MemsSendBuff[30];
uint32_t MemsSendBuffDMA[10];

//惯导航向校准
void AmandMemsNew(void)
{
#if MEMS_TYPE == 1
    AmandMemsNewJ();
    return;
#elif (MEMS_TYPE == 0) || (MEMS_TYPE == 2)
    short int temp = 0;
    unsigned char checkResult = 0;
    unsigned char i = 0;
    if (GDAmand.IsAmand == 1) {
        GDAmand.IsAmand = 0;

        MemsSendBuff[0] = 0xAA;
        MemsSendBuff[1] = 0x55;
        MemsSendBuff[2] = 0x91;
        if (GDAmand.isFirst == 1) {
            MemsSendBuff[3] = 0x11;
            GDAmand.isFirst = 0;
        } else {
            MemsSendBuff[3] = 0x01;
        }

        temp = GDAmand.Value * 100;
        MemsSendBuff[4] = temp & 0xFF;
        MemsSendBuff[5] = (temp & 0xFF00) >> 8;
        for (i = 6; i < 23; i++)
            MemsSendBuff[i] = 0;

        for (i = 0; i < 23; i++)
            checkResult += MemsSendBuff[i];

        checkResult = (0 - checkResult) & 0xFF;
        MemsSendBuff[23] = checkResult;

        MemsSendBuffDMA[0] = 0x009155AA | ((MemsSendBuff[3] << 24) & 0xFF000000);
        MemsSendBuffDMA[1] = MemsSendBuff[4] + (MemsSendBuff[5] << 8);
        MemsSendBuffDMA[2] = 0;
        MemsSendBuffDMA[3] = 0;
        MemsSendBuffDMA[4] = 0;
        MemsSendBuffDMA[5] = checkResult << 24;

        HAL_UART_Transmit_DMA(IN_COM, MemsSendBuff, 24);

        memset(MemsSendBuff, 0, 24);
    }
#endif
}

//惯导零位校准
void MemsZeroCheck(void)
{
    unsigned char checkResult = 0;
    unsigned char i = 0;

    MemsSendBuff[0] = 0xAA;
    MemsSendBuff[1] = 0x55;
    MemsSendBuff[2] = 0xA1;
    MemsSendBuff[3] = 0xAB;

    for (i = 4; i < 23; i++) {
        MemsSendBuff[i] = 0;
    }

    for (i = 0; i < 23; i++)
        checkResult += MemsSendBuff[i];

    checkResult = (0 - checkResult) & 0xFF;
    MemsSendBuff[23] = checkResult;

    MemsSendBuffDMA[0] = 0xABA155AA;
    MemsSendBuffDMA[1] = 0;
    MemsSendBuffDMA[2] = 0;
    MemsSendBuffDMA[3] = 0;
    MemsSendBuffDMA[4] = 0;
    MemsSendBuffDMA[5] = checkResult << 24;

    HAL_UART_Transmit_DMA(IN_COM, MemsSendBuff, 24);

    memset(MemsSendBuff, 0, 24);
}

uint8_t MemsRecvBuff[40];
int MemsRecvCount = 0;
uint8_t MEMS_CH[1] = { 0 };
void MEMS_UART_RecvIT(UART_HandleTypeDef* huart)
{
    uint8_t ch = MEMS_CH[0];
    HAL_UART_Receive_IT(huart, MEMS_CH, 1);
    if (IN_RX_Completed == 0) {
        MemsRecvCount++;
        if (MemsRecvCount == 1) {
            //帧头
            if (ch != 0xAA)
                MemsRecvCount = 0;
            else
                MemsRecvBuff[MemsRecvCount - 1] = ch;
        } else if (MemsRecvCount == 2) {
            //帧头
            if (ch != 0x55)
                MemsRecvCount = 0;
            else
                MemsRecvBuff[MemsRecvCount - 1] = ch;
        } else if (MemsRecvCount < MEMS_RECV_LENGTH) {
            MemsRecvBuff[MemsRecvCount - 1] = ch;
        } else if (MemsRecvCount == MEMS_RECV_LENGTH) {
            MemsRecvBuff[MemsRecvCount - 1] = ch;
            for (uint8_t i = 0; i < MEMS_RECV_LENGTH; i++) {
                IN_GetBuffer[i] = MemsRecvBuff[i];
            }
            IN_RX_Completed = 1;

            rt_event_send(&event_mems, EVENT_FLAG_MEMSRECV);
            MemsRecvCount = 0;
        } else {
            MemsRecvCount = 0;
        }
    }
}
