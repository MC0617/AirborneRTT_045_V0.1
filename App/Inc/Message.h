#ifndef MESSAGE_H_
#define MESSAGE_H_

#include "stm32f4xx_hal.h"

#define BytesPerPeriod 4

// 报文发送结构体
typedef struct _MSG {
    // 周期计数器
    int PeriodCount;

    // 一个报文总共周期数
    int PeriodsPerMsg;

    // 报文字节总数
    int BytesCount;

    int Mode;
} MSG;
extern MSG msg;

// 报文类型集合
typedef enum {
    tParamsReqAnswer = 0x00, //参数设置使能应答
    tParamSetAnswer = 0x01, //参数设置应答
    tACUData = 0x02, //Acu通信数据
    tAllData = 0x03, //全数据
    tAZAll = 0x04, //方位综合
    tELAll = 0x05, //俯仰综合
    tROLLAll = 0x06, //极化(RX)综合
    tPOLAll = 0x07, //极化(TX)综合
    tDVBData = 0x08, //惯导数据
    tTrack1 = 0x09, //跟踪1
    tTrack2 = 0x0A, //跟踪2
    tTrack3 = 0x0B, //跟踪3
} CMDTYPE;

// 通过DMA方式向上位机发送报文
void SendMsgDMA(void);
// 计算得到方位角速率
int AZEncoderSpeed(void);
// 计算得到俯仰角速率
int ELEncoderSpeed(void);
// 拼装报文状态字，保护报文类型及工作模式
uint8_t GetCh(uint8_t cmdtype, uint8_t workmode);
// 生成ACU数据报文
void ACUData(void);
// 生成全数据报文
uint16_t AllData(void);
// 生成方位数据报文
uint8_t AZData(void);
// 生成俯仰数据报文
uint8_t ELData(void);
// 生成横滚数据报文
uint8_t ROLLData(void);
// 生成极化数据报文
uint8_t POLData(void);
// 拼装接收机原始数据报文
uint8_t DVBData(void);
//跟踪监视1
uint8_t TrackMsg1(void);
//跟踪监视2
uint8_t TrackMsg2(void);

#endif
