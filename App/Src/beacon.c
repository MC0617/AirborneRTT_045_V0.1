//链讯多模信标机通信实现
//1_HDR+1_CMD+1_FLAG+N_CONTENT+1_TAIL
#include "beacon.h"

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

#include "Parameters.h"
#include "Senser.h"

extern UART_HandleTypeDef huart4;
UART_HandleTypeDef* BC_COM = &huart4;

// 预设置信标机信息
BeaconInfo_t BeaconInfo;

uint8_t DVBRecvCount = 0;
uint8_t DVBRecvBuffLenth = 4;
uint8_t DVBRecvBuff[50] = { 0 };
uint8_t DVBRecvFlag = 0;

// 信标机配置初始化
void BC_Init()
{
    BeaconInfo.mode = 2;
    BeaconInfo.mono = 0;

    BC_SetMode(2);
    BC_SetRefreshRate(10);
    BC_SetVMapping(1);
    if (SatellitesInfo.localOsci == BENZHEN_097) {
        BC_SetFeed(1);
    } else if (SatellitesInfo.localOsci == BENZHEN_106) {
        BC_SetFeed(2);
    }
    BC_SetCapRange(50);
}

//工作模式
void BC_SetMode(uint8_t mode /*, uint8_t *buf, uint8_t *len */)
{
    uint8_t buf[] = { 0x7B, 0x09, '>', 0x02, 0x7D };

    switch (mode) {
    case 0: //DVBSX
    case 1: //检波
    case 2: //信标
    default:
        break;
    }

    buf[3] = mode;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetMode(void)
{
    uint8_t buf[] = { 0x7B, 0x09, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//DVB搜索参数-略

//信标频率
void BC_SetFreq(uint32_t freq)
{
    uint32_t FREQT = freq * 1000;
    uint8_t buf[] = { 0x7B, 0x08, '>', 0x01, 0x02, 0x03, 0x04, 0x7D }; //0x01020304, Hz

    buf[6] = (FREQT >> 0) & 0xFF;
    buf[5] = (FREQT >> 8) & 0xFF;
    buf[4] = (FREQT >> 16) & 0xFF;
    buf[3] = (FREQT >> 24) & 0xFF;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetFreq(void)
{
    uint8_t buf[] = { 0x7B, 0x08, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//捕获范围
void BC_SetCapRange(uint8_t range)
{
    uint8_t buf[] = { 0x7B, 0x0C, '>', 50, 0x7D }; //50~245

    buf[3] = range;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetCapRange(void)
{
    uint8_t buf[] = { 0x7B, 0x0C, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//信标状态刷新率
void BC_SetRefreshRate(uint8_t rate)
{
    uint8_t buf[] = { 0x7B, 0x0A, '>', 0x0A, 0x7D };

    buf[3] = rate;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetRefreshRate(void)
{
    uint8_t buf[] = { 0x7B, 0x0A, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//检波模式
//检波参数
void BC_SetDetection(uint32_t freq, uint32_t sym, uint32_t rng, uint8_t roll)
{
    uint32_t u32Temp = 0;
    uint8_t buf[] = { 0x7B, 0x0E, '>', 0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x01, 0x7D };

    u32Temp = freq * 1000;

    buf[3] = (u32Temp >> 24) & 0xFF;
    buf[4] = (u32Temp >> 16) & 0xFF;
    buf[5] = (u32Temp >> 8) & 0xFF;
    buf[6] = (u32Temp >> 0) & 0xFF;

    buf[7] = (sym >> 24) & 0xFF;
    buf[8] = (sym >> 16) & 0xFF;
    buf[9] = (sym >> 8) & 0xFF;
    buf[10] = (sym >> 0) & 0xFF;

    //    buf[11] = (rng >> 24) & 0xFF;
    //    buf[12] = (rng >> 16) & 0xFF;
    buf[11] = (rng >> 8) & 0xFF;
    buf[12] = (rng >> 0) & 0xFF;

    buf[13] = roll;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetDetection(void)
{
    uint8_t buf[] = { 0x7B, 0x0E, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//DVB模式
//DVB参数
void BC_SetDVB(uint32_t freq, uint32_t sym)
{
    uint32_t u32Temp = 0;
    uint8_t buf[] = { 0x7B, 0x0B, '>', 0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04, 0x7D };

    u32Temp = freq * 1000;

    buf[3] = (u32Temp >> 24) & 0xFF;
    buf[4] = (u32Temp >> 16) & 0xFF;
    buf[5] = (u32Temp >> 8) & 0xFF;
    buf[6] = (u32Temp >> 0) & 0xFF;

    buf[7] = (sym >> 24) & 0xFF;
    buf[8] = (sym >> 16) & 0xFF;
    buf[9] = (sym >> 8) & 0xFF;
    buf[10] = (sym >> 0) & 0xFF;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetDVB(void)
{
    uint8_t buf[] = { 0x7B, 0x0B, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//电压映射
void BC_SetVMapping(uint8_t sw)
{
    uint8_t buf[] = { 0x7B, 0x10, '>', 0x00, 0x7D };

    buf[3] = sw;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetVMapping(void)
{
    uint8_t buf[] = { 0x7B, 0x10, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

// 馈电
// 1:13.4, 2:18.2, 3:14.6, 4:19.4
void BC_SetFeed(uint8_t vo)
{
    uint8_t buf[] = { 0x7B, 0x05, '>', 0x00, 0x7D }; //1:13.4, 2:18.2, 3:14.6, 4:19.4

    buf[3] = vo;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetFeed(void)
{
    uint8_t buf[] = { 0x7B, 0x05, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//22KHz单音
void BC_Set22K(uint8_t sw)
{
    uint8_t buf[] = { 0x7B, 0x06, '>', 0x00, 0x7D };

    buf[3] = sw;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_Get22K(void)
{
    uint8_t buf[] = { 0x7B, 0x06, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//版本信息-略

//DVB载噪比映射范围-略

//信标电平映射范围
void BC_SetVMappingRange(uint32_t min, uint32_t max)
{
    uint8_t buf[] = { 0x7B, 0x12, '>',
        0x01, 0x02, 0x03, 0x04, //min, 0x01020304
        0x10, 0x20, 0x30, 0x40, //max, 0x10203040
        0x7D };

    buf[3] = (min >> 24) & 0xFF;
    buf[4] = (min >> 16) & 0xFF;
    buf[5] = (min >> 8) & 0xFF;
    buf[6] = (min >> 0) & 0xFF;

    buf[7] = (max >> 24) & 0xFF;
    buf[8] = (max >> 16) & 0xFF;
    buf[9] = (max >> 8) & 0xFF;
    buf[10] = (max >> 0) & 0xFF;

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

void BC_GetVMappingRange(void)
{
    uint8_t buf[] = { 0x7B, 0x12, '?', 0x7D };

    HAL_UART_Transmit_DMA(BC_COM, buf, sizeof(buf));
}

//检波电平映射范围-略

//接收功率电平解析

//查询信标机信息
void BC_GetBCInfo()
{
    uint8_t BCState = BeaconInfo.IsSet;

    if (((BCState >> BC_MODE) & 0x01) == STATE_NOK) {
        BC_GetMode();
    }

    if (((BCState >> BC_FEED) & 0x01) == STATE_NOK) {
        BC_GetFeed();
    }

    if (((BCState >> BC_MONO) & 0x01) == STATE_NOK) {
        BC_Get22K();
    }

    if (((BCState >> BC_FREQ) & 0x01) == STATE_NOK) {
        if (SatellitesInfo.recvMode == SATEMODE_BC) {
            BC_GetFreq();
        } else if (SatellitesInfo.recvMode == SATEMODE_DC) {
            BC_GetDetection();
        } else if (SatellitesInfo.recvMode == SATEMODE_DVB) {
            BC_GetDVB();
        }
    }
}

//设置信标机信息
#include "string.h"
void BC_SetBCInfo()
{
    uint8_t BCState = BeaconInfo.IsSet;

    BeaconInfo.freq = SatellitesInfo.freq - SatellitesInfo.localOsci;

    if (((BCState >> BC_MODE) & 0x01) == STATE_NOK) {
        BC_SetMode(BeaconInfo.mode);
        memset(&Senser.BeaconInfo, 0, sizeof(Senser.BeaconInfo));
    }

    if (((BCState >> BC_FEED) & 0x01) == STATE_NOK) {
        BC_SetFeed(BeaconInfo.feed);
    }

    if (((BCState >> BC_MONO) & 0x01) == STATE_NOK) {
        BC_Set22K(BeaconInfo.mono);
    }

    if (((BCState >> BC_FREQ) & 0x01) == STATE_NOK) {
        if (SatellitesInfo.recvMode == SATEMODE_BC) {
            BC_SetFreq(BeaconInfo.freq);
        } else if (SatellitesInfo.recvMode == SATEMODE_DC) {
            BC_SetDetection(BeaconInfo.freq, BeaconInfo.symbolRate, BeaconInfo.range, BeaconInfo.roll);
        } else if (SatellitesInfo.recvMode == SATEMODE_DVB) {
            BC_SetDVB(BeaconInfo.freq, BeaconInfo.symbolRate);
        }
    }
}

void BC_SetTrState(uint8_t pos, uint8_t isOk)
{
    if (isOk == STATE_NOK) {
        BeaconInfo.IsSet |= (1 << pos);
    } else {
        BeaconInfo.IsSet &= ~(1 << pos);
    }
}

void Debug_AutoGetOsi(void)
{
    //设置接收机参数，载波频率、符码率
    BeaconInfo.freq = SatellitesInfo.freq - SatellitesInfo.localOsci;
    if (BeaconInfo.freq < 950000) {
        BeaconInfo.feed = 1;
        SatellitesInfo.localOsci = BENZHEN_097;
    }
    if (BeaconInfo.freq > 2200000) {
        BeaconInfo.feed = 2;
        SatellitesInfo.localOsci = BENZHEN_106;
    }
    BeaconInfo.freq = SatellitesInfo.freq - SatellitesInfo.localOsci;
}

uint8_t BC_IsNeedSet()
{

    BeaconInfo.freq = SatellitesInfo.freq - SatellitesInfo.localOsci;
    BeaconInfo.symbolRate = SatellitesInfo.symbolRate;
    BeaconInfo.range = SatellitesInfo.searchRange;
    BeaconInfo.roll = SatellitesInfo.roll;
    BeaconInfo.mode = SatellitesInfo.recvMode;
    //    Debug_AutoGetOsi();

    switch (SatellitesInfo.localOsci) {
    case BENZHEN_097:
        BeaconInfo.feed = 1;
        break;
    case BENZHEN_106:
        BeaconInfo.feed = 2;
        break;
    case BENZHEN_113:
        BeaconInfo.feed = 1;
    default:
        BeaconInfo.feed = 1;
        break;
    }

    if (BeaconInfo.mono != Senser.BeaconInfo.mono) {
        BC_SetTrState(BC_MONO, STATE_NOK);
    } else {
        BC_SetTrState(BC_MONO, STATE_OK);
    }

    if (BeaconInfo.feed != Senser.BeaconInfo.feed) {
        BC_SetTrState(BC_FEED, STATE_NOK);
    } else {
        BC_SetTrState(BC_FEED, STATE_OK);
    }

    if (BeaconInfo.mode != Senser.BeaconInfo.mode) {
        BC_SetTrState(BC_MODE, STATE_NOK);
    } else {
        BC_SetTrState(BC_MODE, STATE_OK);
    }

    if (SatellitesInfo.recvMode == SATEMODE_BC) {
        if (BeaconInfo.freq != Senser.BeaconInfo.freq) {
            BC_SetTrState(BC_FREQ, STATE_NOK);
        } else {
            BC_SetTrState(BC_FREQ, STATE_OK);
        }
    } else if (SatellitesInfo.recvMode == SATEMODE_DC) {
        if (BeaconInfo.freq != Senser.BeaconInfo.freq
            || BeaconInfo.symbolRate != Senser.BeaconInfo.symbolRate
            || BeaconInfo.range != Senser.BeaconInfo.range
            || BeaconInfo.roll != Senser.BeaconInfo.roll) {
            BC_SetTrState(BC_FREQ, STATE_NOK);
        } else {
            BC_SetTrState(BC_FREQ, STATE_OK);
        }
    } else if (SatellitesInfo.recvMode == SATEMODE_DVB) {
        if (BeaconInfo.freq != Senser.BeaconInfo.freq
            || BeaconInfo.symbolRate != Senser.BeaconInfo.symbolRate) {
            BC_SetTrState(BC_FREQ, STATE_NOK);
        } else {
            BC_SetTrState(BC_FREQ, STATE_OK);
        }
    }

    return BeaconInfo.IsSet;
}
