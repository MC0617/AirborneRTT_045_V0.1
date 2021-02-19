#ifndef __BUC_H__
#define __BUC_H__

#include "stm32f4xx.h"

typedef enum _BUC_CMD {
    SET_GAIN = 0x01,
    SET_SWITCH = 0x02,
    SET_REBOOT = 0x0B,
    SET_IP_ADDR = 0x0C,
    SET_IP_MASK = 0x60,
    SET_IP_GW = 0x61,

    GET_RANGE = 0x1B,
    GET_IP_MASK = 0x71,
    GET_IP_GW = 0x72,
    GET_FAN = 0xC0,
    GET_PN = 0xC1,
    GET_GAIN = 0xE0,
    GET_POWER = 0xE1,
    GET_ERR = 0xE4,
    GET_VER = 0xE5,
    GET_TEMP = 0xE6,
    GET_SWITCH = 0xE9,
    GET_SN = 0xEA,
    GET_IP_ADDR = 0xF2
} BUC_CMD_t;

typedef enum _BUC_SWITCH {
    BUC_ON = 0x55,
    BUC_OFF = 0xAA
} BUC_SWITCH_t;

typedef struct _BUC_Data {
    uint16_t rangeMax;
    uint16_t rangeMin;
    uint8_t mask[4];
    uint8_t gw[4];
    uint8_t fan;
    uint8_t PN[32];
    float gain;
    float power;
    uint16_t err;
    uint8_t version[3];
    float temp;
    BUC_SWITCH_t state;
    uint8_t sn[32];
    uint8_t addr[4];
    uint8_t setErrCode;
} BUC_Data_t;

extern BUC_Data_t BUC_Data;

extern uint8_t BUC_RecvLength;
extern uint8_t BUC_RecvCount;
extern uint8_t BUC_RecvBuff[50];
extern uint8_t BUC_RecvFlag;

//功放控制
void BUC_Ctrl(BUC_CMD_t cmd, uint32_t arg);

//IP地址转换
//参数：IP地址
//  如地址为192.168.0.1则调用 BUC_CoverIP(192, 168, 0, 1);
uint32_t BUC_CoverIP(uint8_t addr0, uint8_t addr1, uint8_t addr2, uint8_t addr3);

//功放增益转换
//参数：功率
uint32_t BUC_CoverGain(float value);

/* 举例：
    设置功放IP为 192.168.0.1 
        BUC_Ctrl(SET_IP_ADDR, BUC_CoverIP(192, 168, 0, 1));
    设置增益为 12.34 
        BUC_Ctrl(SET_GAIN, BUC_CoverGain(12.34));
    设置功放打开
        BUC_Ctrl(SET_SWITCH, BUC_ON);
    
    查询增益
        BUC_Ctrl(GET_GAIN, 0);
    查询功率
        BUC_Ctrl(GET_POWER, 0);
*/

//返回数据解析
void AnalysisBUC(uint8_t* buff, uint8_t len);

//中断接收返回数据
extern uint8_t BUC_CH[1];
void BUC_UART_RecvIT(UART_HandleTypeDef* huart);

#endif // !__BUC_H__
