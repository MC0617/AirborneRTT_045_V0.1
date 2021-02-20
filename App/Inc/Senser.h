#ifndef SENSER_H_
#define SENSER_H_

#include "stm32f4xx_hal.h"

#include "Parameters.h"

#define PI 3.141592653f
#define BENZHEN_106 10600000
#define BENZHEN_113 11300000
#define BENZHEN_097 9750000

#define MEMS_TYPE 2

// AGC信息
typedef struct _AGCMAX {
    float Agc;
    float AZg;
    float ELg;

    float azb, elb, crb, head, pitch, roll;
} AGCMAX;

enum _BugInfo {
    BUG_AZ = 0,
    BUG_EL,
    BUG_ROLL,
    BUG_POL,
    BUG_IO_AZ,
    BUG_IO_EL,
    BUG_IO_ROLL,
    BUG_IO_POL,
    BUG_RECVER,
    BUG_MEMS,
    BUG_GPSLOST,
    BUG_AGCNOTFOUND,
    BUG_AGCLOST
};
// 编码器信息
typedef struct _ENCODER {
    int32_t Count;
    int32_t Count_Last;
    int16_t PeriodCount;
    int16_t PeriodCount_Last;
    unsigned short int PeriodCountTest;

} ENCODER;

//DVB接收机
typedef struct _DVB {
    unsigned long int Fre;
    unsigned long int FreWait; //等待设置的载波频率
    unsigned long int Xinbiao;
    unsigned long int XinbiaoWait; //等待设置的信标频率
    unsigned long int SR;
    unsigned long int SRWait; //等待设置的符码率
    unsigned short int LostCount; //DVB丢帧计数器
    unsigned short int ErrCount;
    unsigned char Setting; //设置参数标志位;0:无设置,1:正在设置
    unsigned char RecvMode;
    unsigned int BugTime;
    //unsigned char SetResult;	//设置结果;0:失败,1:成功
} DVB;

typedef struct SerialRecv {
    uint16_t head;
    uint16_t nextHead;
    uint8_t recvLength;
    uint8_t recvCount;
    uint8_t buff[200];
} SerialRecv_t;

//Mems惯导
typedef struct _MEMS {
    unsigned char Status; //惯导状态
    unsigned char LostCount; //惯导丢帧计数器
    unsigned char ErrCount; //惯导错帧计数器
    unsigned int BugTime;
} MEMS;

//温湿度
typedef struct _TEMP_HUM {
    float Temperature; //温度
    float Humidity; //湿度
} TEMP_HUM;

// 传感器数据
typedef struct _SENSER {
    float Angle_AZ[11];
    float Encoder_AZ[11];
    float Angle_EL[11];
    float Angle_ROLL[11];
    float Angle_POL[11];

    float SpeedAZ;
    float Speed_EL;
    float SpeedROLL;
    float SpeedPOL;

    float Head[11];
    float Pitch[11];
    float Roll[11];

    float HeadLast;
    float HeadNew;

    float PitchLast;
    float PitchNew;

    float RollLast;
    float RollNew;

    float Agc[11];

    AGCMAX AgcMAx;

    //寻零得到的编码器Count值得0位的偏移量
    int32_t AngleOffset_AZ;
    int32_t AngleOffset_EL;
    int32_t AngleOffset_ROLL;
    int32_t AngleOffset_POL;

    //Longitude
    float GPSX;
    //Latitude
    float GPSY;
    //Height
    float GPSH;

    float AZg[11];
    float ELg[11];

    unsigned char ReceiverLocked;

    unsigned char AZ_On_Off;
    unsigned char EL_On_Off;

    ENCODER AZEncoder;
    ENCODER ELEncoder;
    ENCODER ROLLEncoder;
    ENCODER POLEncoder;

    TEMP_HUM TempHum;

    float HeadOffset;

    float MemsX;
    float MemsY;
    float MemsZ;

    SATELLITEINFO SateInfo;
    BeaconInfo_t BeaconInfo;

    MEMS MEMSInfo;

    uint32_t BC_COM_BAUD;
    uint32_t BENZHEN;

    uint16_t AZBugTime;
    uint16_t ELBugTime;
    uint16_t ROLLBugTime;
    uint16_t POLBugTime;

    uint16_t AgcFindBugTime;
    uint8_t AgcLostFlag;

    uint32_t BugInfo;

    uint8_t MemsFlag;

} SENSER;
extern SENSER Senser;

#if MEMS_TYPE == 0
#define MEMS_RECV_LENGTH 24
#elif MEMS_TYPE == 1
#define MEMS_RECV_LENGTH 32
#elif MEMS_TYPE == 2
#define MEMS_RECV_LENGTH 34
#endif
extern uint8_t IN_GetBuffer[MEMS_RECV_LENGTH];

//测试数组，存储接收到的DVB接收数据，通过PC通信串口直接输出至上位机
extern unsigned char DVBRecvBuffTest[20];

// 解算各传感器数据，包括编码器、MEMS、DVB
void DataCollection(void);

//自跟踪模式下AGC滤波
void AGCFilter(void);

//惯导零位校准
void MemsZeroCheck(void);

// 等待惯导数据，并作为一个控制周期的开始
void GetMemsData(uint32_t e);

// 保存故障信息
void BugSys(void);

#endif /* SENSER_H_ */
