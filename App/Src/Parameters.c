#include "Senser.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "MotorControl.h"
#include "Parameters.h"
#include "ParametersDefine.h"
#include "beacon.h"
#include "buc.h"
#include "flash.h"
#include "main.h"

#define SATELINFO_LENGTH 27
int PeriodPilotLamp = 0; //指示灯控制计数器

char AZFindZeroEndFlag = 0; //方位寻零完成标志
char ELFindZeroEndFlag = 0; //俯仰寻零完成标志
char ROLLFindZeroEndFlag = 0; //横滚寻零完成标志
char POLFindZeroEndFlag = 0; //极化寻零完成标志

char HeadCheckFlag = 0; //航向校准完成标志

//	工作模式
uint8_t WorkMode;

//工作目标卫星信息
SATELLITEINFO SatellitesInfo;

SEARCHPARAMS SearchParams;

TRACKPARAMS TrackParams;

// 方位寻零参数
FINDZEROPARAMS AZFindZeroParams;
// 俯仰寻零参数
FINDZEROPARAMS ELFindZeroParams;
// 横滚寻零参数
FINDZEROPARAMS ROLLFindZeroParams;
// 极化寻零参数
FINDZEROPARAMS POLFindZeroParams;
// 初始航向校准参数
FINDZEROPARAMS HeadCheckParams;

DEBUGPARAMS DebugParams;

GDAMAND GDAmand;

int HandControlEnable = 0; //手动控制使能;1:使能,0:去使能

int StandbyTimeCount = 0; //待机减速计时器

//数组，用于存储调试参数
static uint8_t FlashBufferA[210];
static uint8_t FlashBufferB[260];

// 卫星信息当前存储地址标记
static uint8_t CurrSataAddr = 0;

void InitEncoderData(void)
{
    Senser.AZEncoder.Count = 0;
    Senser.AZEncoder.Count_Last = 0;
    Senser.AZEncoder.PeriodCount = 0;
    Senser.AZEncoder.PeriodCount_Last = 0;

    Senser.ELEncoder.Count = 0;
    Senser.ELEncoder.Count_Last = 0;
    Senser.ELEncoder.PeriodCount = 0;
    Senser.ELEncoder.PeriodCount_Last = 0;

    Senser.ROLLEncoder.Count = 0;
    Senser.ROLLEncoder.Count_Last = 0;
    Senser.ROLLEncoder.PeriodCount = 0;
    Senser.ROLLEncoder.PeriodCount_Last = 0;

    Senser.POLEncoder.Count = 0;
    Senser.POLEncoder.Count_Last = 0;
    Senser.POLEncoder.PeriodCount = 0;
    Senser.POLEncoder.PeriodCount_Last = 0;
}

uint8_t CheckSumResult(uint8_t* Buff, uint8_t len)
{
    uint8_t RV = 0;
    uint8_t iCnt = 0;
    for (iCnt = 0; iCnt < len; ++iCnt) {
        RV ^= Buff[iCnt];
    }

    RV = 0x7F & RV;
    RV = 0x40 | RV;
    return RV;
}

uint8_t ReadSateInfo(uint8_t* Buff)
{
    uint8_t BuffA[SATELINFO_LENGTH] = { 0 };
    uint8_t BuffB[SATELINFO_LENGTH] = { 0 };
    Flash_Read8(ADDR_FLASH_SECTOR[9] + SATELINFO_LENGTH, BuffA, SATELINFO_LENGTH);
    Flash_Read8(ADDR_FLASH_SECTOR[10] + SATELINFO_LENGTH, BuffB, SATELINFO_LENGTH);

    if (CheckSumResult(BuffA, SATELINFO_LENGTH - 1) == BuffA[SATELINFO_LENGTH - 1]) {
        memcpy(Buff, BuffA, SATELINFO_LENGTH);
        CurrSataAddr = 0x01;
        return 0x01;
    } else if (CheckSumResult(BuffB, SATELINFO_LENGTH - 1) == BuffB[SATELINFO_LENGTH - 1]) {
        memcpy(Buff, BuffB, SATELINFO_LENGTH);
        CurrSataAddr = 0x02;
        return 0x02;
    } else {
        Flash_Read8(ADDR_FLASH_SECTOR[9], BuffA, SATELINFO_LENGTH);
        Flash_Read8(ADDR_FLASH_SECTOR[10], BuffB, SATELINFO_LENGTH);
        if (CheckSumResult(BuffA, SATELINFO_LENGTH - 1) == BuffA[SATELINFO_LENGTH - 1]) {
            SatellitesInfo.IsStoreData = 1;
            memcpy(Buff, BuffA, SATELINFO_LENGTH);
            CurrSataAddr = 0x11;
            return 0x11;
        } else if (CheckSumResult(BuffB, SATELINFO_LENGTH - 1) == BuffB[SATELINFO_LENGTH - 1]) {
            SatellitesInfo.IsStoreData = 1;
            memcpy(Buff, BuffB, SATELINFO_LENGTH);
            CurrSataAddr = 0x12;
            return 0x12;
        } else {
            SatellitesInfo.longitude = 87.5;
            SatellitesInfo.localOsci = 10600000;
            SatellitesInfo.IsStoreData = 1;
            SatellitesInfo.polerMode = 0;
            SatellitesInfo.freq = 11699800;
            SatellitesInfo.symbolRate = 2500000;
            CurrSataAddr = 0xFF;
        }
    }
    return 0;
}

// 从Flash中读取卫星信息
void ReadSatelliteInfo(void)
{
    //从Flash中读取卫星信息包括：卫星经度，极化方式，载波频率，符码率
    long int tempInt32 = 0;
    short int tempInt16 = 0;
    uint8_t b3 = 0, b2 = 0, b1 = 0, b0 = 0;

    if (0 == ReadSateInfo(FlashBufferA)) {
        return;
    }
    //取卫星经度
    b1 = FlashBufferA[0];
    b0 = FlashBufferA[1];
    tempInt16 = (b1 << 8) + b0;
    SatellitesInfo.longitude = tempInt16 / 100.0f;

    //取极化方式
    b0 = FlashBufferA[2];
    SatellitesInfo.polerMode = b0;

    //取本振
    b3 = FlashBufferA[3];
    b2 = FlashBufferA[4];
    b1 = FlashBufferA[5];
    b0 = FlashBufferA[6];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
    SatellitesInfo.localOsci = tempInt32;

    //取信标机设置
    b3 = FlashBufferA[7];
    b2 = FlashBufferA[8];
    b1 = FlashBufferA[9];
    b0 = FlashBufferA[10];
    SatellitesInfo.roll = b3;
    BeaconInfo.feed = b2;
    BeaconInfo.mono = b1;
    SatellitesInfo.recvMode = b0;

    //取信标频率
    b3 = FlashBufferA[11];
    b2 = FlashBufferA[12];
    b1 = FlashBufferA[13];
    b0 = FlashBufferA[14];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
    SatellitesInfo.freq = tempInt32;

    //取符号速率
    b3 = FlashBufferA[15];
    b2 = FlashBufferA[16];
    b1 = FlashBufferA[17];
    b0 = FlashBufferA[18];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
    SatellitesInfo.symbolRate = tempInt32;

    //取搜索范围
    b3 = FlashBufferA[19];
    b2 = FlashBufferA[20];
    b1 = FlashBufferA[21];
    b0 = FlashBufferA[22];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
    SatellitesInfo.searchRange = tempInt32;

    //取预留参数
    b3 = FlashBufferA[23];
    b2 = FlashBufferA[24];
    b1 = FlashBufferA[25];
    b0 = FlashBufferA[26];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
}

void WriteSateInfo(uint8_t* Buff, uint32_t len)
{
    Buff[len - 1] = CheckSumResult(Buff, len - 1);

    Flash_Write8(ADDR_FLASH_SECTOR[9], Buff, len);
    CurrSataAddr = 0x02;
}

// 将当前卫星信息写入Flash
void WriteSatelliteInfo(void)
{
    //写入数据包括：卫星经度、极化方式、载波频率、符码率
    short int tempInt16 = 0;
    long int tempInt32 = 0;
    uint8_t b3 = 0, b2 = 0, b1 = 0, b0 = 0;

    //卫星经度
    tempInt16 = SatellitesInfo.longitude * 100;
    b1 = (tempInt16 >> 8) & 0xFF;
    b0 = tempInt16 & 0xFF;
    FlashBufferA[0] = b1;
    FlashBufferA[1] = b0;

    //极化方式
    FlashBufferA[2] = SatellitesInfo.polerMode;

    //本振
    tempInt32 = SatellitesInfo.localOsci;
    b3 = (tempInt32 >> 24) & 0xFF;
    b2 = (tempInt32 >> 16) & 0xFF;
    b1 = (tempInt32 >> 8) & 0xFF;
    b0 = tempInt32 & 0xFF;
    FlashBufferA[3] = b3;
    FlashBufferA[4] = b2;
    FlashBufferA[5] = b1;
    FlashBufferA[6] = b0;

    //信标机设置
    FlashBufferA[7] = SatellitesInfo.roll;
    FlashBufferA[8] = BeaconInfo.feed;
    FlashBufferA[9] = BeaconInfo.mono;
    FlashBufferA[10] = SatellitesInfo.recvMode;

    //接收频率
    tempInt32 = SatellitesInfo.freq;
    b3 = (tempInt32 >> 24) & 0xFF;
    b2 = (tempInt32 >> 16) & 0xFF;
    b1 = (tempInt32 >> 8) & 0xFF;
    b0 = tempInt32 & 0xFF;
    FlashBufferA[11] = b3;
    FlashBufferA[12] = b2;
    FlashBufferA[13] = b1;
    FlashBufferA[14] = b0;

    //符号速率
    tempInt32 = SatellitesInfo.symbolRate;
    b3 = (tempInt32 >> 24) & 0xFF;
    b2 = (tempInt32 >> 16) & 0xFF;
    b1 = (tempInt32 >> 8) & 0xFF;
    b0 = tempInt32 & 0xFF;
    FlashBufferA[15] = b3;
    FlashBufferA[16] = b2;
    FlashBufferA[17] = b1;
    FlashBufferA[18] = b0;

    //搜索范围
    tempInt32 = SatellitesInfo.searchRange;
    b3 = (tempInt32 >> 24) & 0xFF;
    b2 = (tempInt32 >> 16) & 0xFF;
    b1 = (tempInt32 >> 8) & 0xFF;
    b0 = tempInt32 & 0xFF;
    FlashBufferA[19] = b3;
    FlashBufferA[20] = b2;
    FlashBufferA[21] = b1;
    FlashBufferA[22] = b0;

    //预留参数
    tempInt32 = 0x11223344;
    b3 = (tempInt32 >> 24) & 0xFF;
    b2 = (tempInt32 >> 16) & 0xFF;
    b1 = (tempInt32 >> 8) & 0xFF;
    b0 = tempInt32 & 0xFF;
    FlashBufferA[23] = b3;
    FlashBufferA[24] = b2;
    FlashBufferA[25] = b1;
    FlashBufferA[26] = b0;

    WriteSateInfo(FlashBufferA, SATELINFO_LENGTH);
}

// 从Flash中读取调试参数
void ReadDebugParams()
{
    const uint8_t paramsCount = 50;
    uint8_t i = 0;
    uint8_t b3, b2, b1, b0;
    long int tempInt32 = 0;

    Flash_Read8(ADDR_FLASH_SECTOR[11], FlashBufferB, paramsCount * 4);
    for (i = 0; i < paramsCount; i++) {
        b3 = FlashBufferB[i * 4];
        b2 = FlashBufferB[i * 4 + 1];
        b1 = FlashBufferB[i * 4 + 2];
        b0 = FlashBufferB[i * 4 + 3];
        tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
        DebugParams.Params[i] = tempInt32 * 0.0001;
    }
}

// 将调试参数写入flash
void WriteDebugParam()
{
    const uint8_t paramsCount = 50;
    uint8_t i = 0;
    uint8_t b3, b2, b1, b0;
    long int tempInt32 = 0;

    for (i = 0; i < paramsCount; i++) {
        tempInt32 = DebugParams.Params[i] * 10000;
        b3 = (tempInt32 & 0xFF000000) >> 24;
        b2 = (tempInt32 & 0x00FF0000) >> 16;
        b1 = (tempInt32 & 0x0000FF00) >> 8;
        b0 = tempInt32 & 0x000000FF;
        FlashBufferB[i * 4] = b3;
        FlashBufferB[i * 4 + 1] = b2;
        FlashBufferB[i * 4 + 2] = b1;
        FlashBufferB[i * 4 + 3] = b0;
    }
    Flash_Write8(ADDR_FLASH_SECTOR[11], FlashBufferB, paramsCount * 4);
}

// 零位校准参数初始化
void ParamsInit_ZeroCheck()
{
    AZFindZeroParams.State = 0;
    ELFindZeroParams.State = 0;
    ROLLFindZeroParams.State = 0;
    POLFindZeroParams.State = 0;

    AZFindZeroEndFlag = 0;
    ELFindZeroEndFlag = 0;
    ROLLFindZeroEndFlag = 0;
    POLFindZeroEndFlag = 0;
}

// 方位PID相关参数初始化
void ParamsInit_PIDAZ()
{
    MotoCtr.PIDAZ.err = 0;
    MotoCtr.PIDAZ.err_last = 0;
    MotoCtr.PIDAZ.err_last_before = 0;
    MotoCtr.PIDAZ.integral = 0;
    MotoCtr.PIDAZ.Value_Last = 0;
    MotoCtr.PIDAZ.PLast = 0;
}

// 俯仰PID相关参数初始化
void ParamsInit_PIDEL()
{
    MotoCtr.PIDEL.err = 0;
    MotoCtr.PIDEL.err_last = 0;
    MotoCtr.PIDEL.err_last_before = 0;
    MotoCtr.PIDEL.integral = 0;
    MotoCtr.PIDEL.Value_Last = 0;
    MotoCtr.PIDEL.PLast = 0;
}

// 交叉PID相关参数初始化
void ParamsInit_PIDROLL()
{
    MotoCtr.PIDROLL.err = 0;
    MotoCtr.PIDROLL.err_last = 0;
    MotoCtr.PIDROLL.err_last_before = 0;
    MotoCtr.PIDROLL.integral = 0;
    MotoCtr.PIDROLL.Value_Last = 0;
    MotoCtr.PIDROLL.PLast = 0;
}

// 参数初始化
void ParamsInit()
{
    int i = 0;
    for (i = 0; i < 10; i++) {
        Senser.Angle_AZ[i] = 0;
        Senser.Angle_EL[i] = 0;
        Senser.Angle_ROLL[i] = 0;
        Senser.Angle_POL[i] = 0;

        Senser.Head[i] = 0;
        Senser.Pitch[i] = 0;
        Senser.Roll[i] = 0;

        Senser.Agc[i] = 0;

        Senser.AZg[i] = 0;
        Senser.ELg[i] = 0;
    }

    Senser.AngleOffset_ROLL = 0;

    Senser.SpeedAZ = 0;
    Senser.Speed_EL = 0;
    Senser.SpeedROLL = 0;
    Senser.SpeedPOL = 0;

    Senser.AgcMAx.AZg = 0;
    Senser.AgcMAx.ELg = 0;
    Senser.AgcMAx.Agc = 0;

    Senser.GPSH = 0;
    Senser.GPSX = 0;
    Senser.GPSY = 0;

    Senser.AZEncoder.Count = AZ_EC_TIM->CNT;
    Senser.AZEncoder.Count_Last = AZ_EC_TIM->CNT;
    Senser.AZEncoder.PeriodCount = 0;
    Senser.AZEncoder.PeriodCount_Last = 0;

    Senser.TempHum.Temperature = 0;
    Senser.TempHum.Humidity = 0;

    Senser.HeadOffset = 0;

    ELFindZeroEndFlag = 0;
    ELFindZeroParams.PeriodCount = 0;
    ELFindZeroParams.State = 0;

    MotoCtr.AZSpeed = 0;
    MotoCtr.ESpeed = 0;

    MotoCtr.AZPerset = 0;
    MotoCtr.AZgPerset = 0;

    MotoCtr.ELPerset = 0;
    MotoCtr.ELgPerset = 0;

    MotoCtr.ROLLSpeed = 0;
    MotoCtr.ROLLPerset = 0;

    MotoCtr.POLSpeed = 0;
    MotoCtr.POLPerset = 0;

    HeadCheckFlag = 0;

    MotoCtr.PeriodCountTotal = 1;
    MotoCtr.PeriodCountSinAZ = 0;
    MotoCtr.PeriodCountSinEL = 0;
    MotoCtr.PeriodCountSinROLL = 0;
    MotoCtr.PeriodCountSinPRx = 0;

    ParamsInit_PIDAZ();
    ParamsInit_PIDEL();
    ParamsInit_PIDROLL();

    ReadDebugParams();
    ReadSatelliteInfo();

    msg.Mode = DEFAULTMSGTYPE;
    msg.PeriodsPerMsg = 100;
    msg.PeriodCount = 0;

    WorkMode = Mode_Stand_By;

    TrackParams.CircleTime = 0.5;
    TrackParams.RangeAZ = 0.05;
    TrackParams.RangeEL = 0.05;
    TrackParams.PeriodCountStepTotal = TrackParams.CircleTime * 1000 / 5;
    TrackParams.Timer2 = 0;
    TrackParams.RangeChangeFlag = 0;

    GDAmand.IsAmand = 0;
    GDAmand.Value = 0;

    InitEncoderData();

    HeadCheckParams.State = 0;

    ParamsInit_ZeroCheck(); //零位校准参数初始化

    if (XBBAUD_S == 0) {
        Senser.BC_COM_BAUD = 115200;
    } else {
        Senser.BC_COM_BAUD = 19200;
    }
    if (LOCOSC_S == 0) {
        Senser.BENZHEN = BENZHEN_106;
    } else {
        Senser.BENZHEN = BENZHEN_097;
    }

    // SatellitesInfo.localOsci = Senser.BENZHEN;
    // BeaconInfo.mode = RecvModeDefault;

    BeaconInfo.freq = SatellitesInfo.freq - SatellitesInfo.localOsci;
    HeadCheckFlag = (int)AUTOFS_S == 1 ? 0 : 1;
    BeaconInfo.IsSet = 0x0F;

    Senser.MemsFlag = (uint8_t)MEMSIN_S;

    memset(&BUC_Data, 0, sizeof(BUC_Data));
}

// 将数据存入Flash中
void FlashDataWrite(void)
{
    if (DebugParams.IsStore == 1) {
        //			rt_enter_critical();
        WriteDebugParam();
        //			rt_exit_critical();
        DebugParams.IsStore = 0;
    }

    if (SatellitesInfo.IsStoreData == 1) {
        WriteSatelliteInfo();
        SatellitesInfo.IsStoreData = 0;
    }
}
