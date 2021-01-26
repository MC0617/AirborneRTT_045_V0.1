
#include "DataAnalysis.h"

#include "stdlib.h"
#include "string.h"

#include "MotorControl.h"
#include "Parameters.h"
#include "ParametersDefine.h"
#include "Senser.h"
#include "buc.h"
#include "uart_api.h"

// 判断是否接收到新的上位机控制报文
unsigned char IsGetNewCMD_PC(void)
{
    uint8_t result = 0;
    if (PCRecvFlag == 1) {
        result = 1;
    } else {
        result = 0;
    }
    PCRecvFlag = 0;
    return result;
}

// 所有字节(除校验字节)累加取低8位
unsigned char CheckCMDHead_PC(void)
{
    unsigned char checkResult = 0;
    unsigned char i = 0;
    if (PCRecvBuff[0] != 0xEB || PCRecvBuff[1] != 0x90) {
        return 0;
    } else {
        for (i = 0; i < 23; i++) {
            checkResult += PCRecvBuff[i];
        }
        if (checkResult != PCRecvBuff[23]) {
            return 0;
        } else {
            return 1;
        }
    }
}

void ACU_SetSate()
{
    uint8_t bSearchFlag = 0;
    uint8_t u8Temp = 0;
    int32_t i32Temp = 0;
    uint32_t u32Temp = 0;

    u8Temp = PCRecvBuff[5]; //极化方式
    if (u8Temp != SatellitesInfo.polerMode) {
        // bSearchFlag = 1;
        SatellitesInfo.polerMode = u8Temp;
    }

    u8Temp = PCRecvBuff[6]; //是否使用参考星
    // if (u8Temp != SatellitesInfo.polerMode) {
    //     bSearchFlag = 1;
    //     SatellitesInfo.polerMode = u8Temp;
    // }

    i32Temp = ((PCRecvBuff[7] << 24) + (PCRecvBuff[8] << 16) + (PCRecvBuff[9] << 8) + (PCRecvBuff[10] << 0)); //卫星经度
    if (i32Temp / 1000.0 != SatellitesInfo.longitude) {
        bSearchFlag = 1;
        SatellitesInfo.longitude = i32Temp / 1000.0;
    }

    u8Temp = PCRecvBuff[11]; //接收机模式
    SatellitesInfo.recvMode = u8Temp;

    u32Temp = ((PCRecvBuff[12] << 24) + (PCRecvBuff[13] << 16) + (PCRecvBuff[14] << 8) + (PCRecvBuff[15] << 0)); //接收频率
    SatellitesInfo.freq = u32Temp;

    u32Temp = ((PCRecvBuff[16] << 24) + (PCRecvBuff[17] << 16) + (PCRecvBuff[18] << 8) + (PCRecvBuff[19] << 0)); //符号速率
    SatellitesInfo.symbolRate = u32Temp;

    u32Temp = ((PCRecvBuff[20] << 24) + (PCRecvBuff[21] << 16) + (PCRecvBuff[22] << 8) + (PCRecvBuff[23] << 0)); //搜索范围
    SatellitesInfo.searchRange = u32Temp;

    u8Temp = PCRecvBuff[24]; //滚降系数
    SatellitesInfo.roll = u8Temp;

    u8Temp = PCRecvBuff[25];
    if (u8Temp == 1) {
        WriteSatelliteInfo();
    }

    WorkMode = Mode_AimSatellite;
    //WorkMode = Mode_Stand_By;
    Senser.AgcMAx.AZg = 0;
    Senser.AgcMAx.ELg = 0;
    Senser.AgcMAx.Agc = 0;
    MotoCtr.Timer.az = 0;
    MotoCtr.Timer.el = 0;
    MotoCtr.PIDAZ.integral = 0;

    if (bSearchFlag == 1) {
        // WorkMode = Mode_HeadCheck;
        // HeadCheckFlag = 0;
        // HeadCheckParams.State = 0;
    }
}

void ACU_SetLnbLoFreq()
{
    uint8_t u8Temp = 0;
    uint32_t u32Temp = 0;

    u32Temp = ((PCRecvBuff[5] << 24) + (PCRecvBuff[6] << 16) + (PCRecvBuff[7] << 8) + (PCRecvBuff[8] << 0));
    Senser.BENZHEN = u32Temp;
    SatellitesInfo.localOsci = u32Temp;

    u8Temp = PCRecvBuff[9];
    switch (u8Temp) {
    case 0x00:
        break;
    case 0x01:
        break;
    case 0x02:
        BeaconInfo.feed = 1;
        BeaconInfo.mono = 0;
        break;
    case 0x03:
        BeaconInfo.feed = 1;
        BeaconInfo.mono = 1;
        break;
    case 0x04:
        BeaconInfo.feed = 2;
        BeaconInfo.mono = 0;
        break;
    case 0x05:
        BeaconInfo.feed = 2;
        BeaconInfo.mono = 1;
        break;
    default:
        BeaconInfo.feed = 1;
        BeaconInfo.mono = 0;
    }

    u8Temp = PCRecvBuff[10];
    if (u8Temp == 1) {
        WriteSatelliteInfo();
    }
}

void ACU_SetBSate()
{
}

void ACU_HandCtrl()
{
}

void ACU_SetMode()
{
    switch (PCRecvBuff[5]) {
    case 0x00:
        WorkMode = Mode_AimSatellite;
        Senser.AgcMAx.AZg = 0;
        Senser.AgcMAx.ELg = 0;
        Senser.AgcMAx.Agc = 0;
        MotoCtr.Timer.az = 0;
        MotoCtr.Timer.el = 0;
        MotoCtr.Timer.roll = 0;
        MotoCtr.Timer.pol = 0;
        MotoCtr.PIDAZ.integral = 0;
        break;
    case 0x01:
        WorkMode = Mode_Stand_By;
        StandbyTimeCount = 0;
        HeadCheckFlag = 1;
        AZFindZeroEndFlag = 1;
        ELFindZeroEndFlag = 1;
        ROLLFindZeroEndFlag = 1;
        POLFindZeroEndFlag = 1;
        break;
    case 0x02:

        break;
    case 0x03:

        break;
    default:
        break;
    }
}

void AnalysisACU(void)
{
    switch (PCRecvBuff[4]) {
    case 0x10:
        ACU_SetSate();
        break;
    case 0x11:
        ACU_SetLnbLoFreq();
        break;
    case 0x12:
        ACU_SetBSate();
        break;
    case 0x13:
        ACU_HandCtrl();
        break;
    case 0x14:
        ACU_SetMode();
        break;
    default:
        break;
    }
}

// 解析工作模式设置指令
void AnalysisWorkModeSet()
{
    unsigned char CtrData = PCRecvBuff[4];
    switch (CtrData) {
    case 0x02:
        //工作模式
        WorkMode = Mode_AimSatellite;
        Senser.AgcMAx.AZg = 0;
        Senser.AgcMAx.ELg = 0;
        Senser.AgcMAx.Agc = 0;
        MotoCtr.Timer.az = 0;
        MotoCtr.Timer.el = 0;
        MotoCtr.Timer.roll = 0;
        MotoCtr.Timer.pol = 0;
        MotoCtr.PIDAZ.integral = 0;
        break;
    case 0x01:
        //收藏模式
        WorkMode = Mode_Collection;
        MotoCtr.Timer.az = 0;
        MotoCtr.Timer.el = 0;
        MotoCtr.Timer.roll = 0;
        MotoCtr.Timer.pol = 0;
        HeadCheckFlag = 1;
        break;
    case 0x00:
        //待机模式
        WorkMode = Mode_Stand_By;
        StandbyTimeCount = 0;
        HeadCheckFlag = 1;
        AZFindZeroEndFlag = 1;
        ELFindZeroEndFlag = 1;
        ROLLFindZeroEndFlag = 1;
        POLFindZeroEndFlag = 1;
        break;
    }
}

// 上传报文设置
void AnalysisMsgSet()
{
    unsigned short int type = 0;
    unsigned char HByte = 0, LByte = 0;

    HByte = PCRecvBuff[3];
    LByte = PCRecvBuff[4];
    type = (HByte << 8) + LByte;

    msg.Mode = type;
}

// 解析手动控制使能指令
void AnalysisHandCtrEnable()
{
    //param1 = 0: 去使能
    //param1 = 1: 使能
    unsigned char Enabled = 0;
    unsigned char HByte = 0, LByte = 0;
    HByte = PCRecvBuff[3];
    LByte = PCRecvBuff[4];
    Enabled = (HByte << 8) + LByte;

    if (Enabled == 0) {
        HandControlEnable = 0;
        WorkMode = Mode_Stand_By;
    } else if (Enabled == 1) {
        HandControlEnable = 1;
        WorkMode = Mode_Hand_Control;
        MotoCtr.AZSpeed = 0;
        MotoCtr.ESpeed = 0;
        MotoCtr.ROLLSpeed = 0;
        MotoCtr.POLSpeed = 0;
    }
}

// 解析手动控制角速度设置指令
void AnalysisHandCtrSpeedSet()
{
    unsigned short int CtrIndex = 0;
    short int SpeedInt16 = 0;
    float Speed = 0;
    unsigned char HByte = 0, LByte = 0;

    //param1 = 0 : 方位
    //param1 = 1 : 俯仰
    //param1 = 2 : 横滚
    //param1 = 3 : 极化
    HByte = PCRecvBuff[3];
    LByte = PCRecvBuff[4];
    CtrIndex = (HByte << 8) + LByte;

    HByte = PCRecvBuff[5];
    LByte = PCRecvBuff[6];
    SpeedInt16 = (HByte << 8) + LByte;
    Speed = SpeedInt16 * 0.01;

    MotoCtr.AZSpeed = 0;
    MotoCtr.ESpeed = 0;
    MotoCtr.ROLLSpeed = 0;
    MotoCtr.POLSpeed = 0;

    switch (CtrIndex) {
    case 0:
        MotoCtr.AZSpeed = Speed;
        break;
    case 1:
        MotoCtr.ESpeed = Speed;
        break;
    case 2:
        MotoCtr.ROLLSpeed = Speed;
        break;
    case 3:
        MotoCtr.POLSpeed = Speed;
        break;
    }
}

//参数设置
void AnalysisParamsSet()
{
    unsigned char Mode = 0; // 参数设置功能模式;0:刷新数据;1:设置参数;2:保存参数
    long int tempInt32 = 0;
    unsigned short int Index = 0; //参数索引
    unsigned char b3 = 0, b2 = 0, b1 = 0, b0 = 0;
    float param = 0;

    Mode = PCRecvBuff[4];
    if (Mode == 0) {
        msg.Mode = 3; //全数据输出
    } else if (Mode == 2) {
        DebugParams.IsStore = 1;
    } else if (Mode == 1) {
        //获取数据索引值
        b1 = PCRecvBuff[5];
        b0 = PCRecvBuff[6];
        Index = (b1 << 8) + b0;

        b3 = PCRecvBuff[7];
        b2 = PCRecvBuff[8];
        b1 = PCRecvBuff[9];
        b0 = PCRecvBuff[10];
        tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
        param = tempInt32 * 0.0001f;
        DebugParams.Params[Index] = param;
        if (Index == 27) {
            TrackParams.PeriodCount = 0;
            TrackParams._value1 = 0;
            TrackParams._value2 = 0;
            TrackParams._value3 = 0;
            TrackParams._value4 = 0;
        }
    }
}

//解析卫星参数设置报文
void AnalysisSateInfoSet()
{
    short int tempInt16 = 0;
    long int tempInt32 = 0;
    unsigned char b3 = 0, b2 = 0, b1 = 0, b0 = 0;

    //获取卫星经度
    b1 = PCRecvBuff[3];
    b0 = PCRecvBuff[4];
    tempInt16 = (b1 << 8) + b0;
    SatellitesInfo.longitude = tempInt16 * 0.01f;

    //获取极化方式
    b1 = PCRecvBuff[5];
    b0 = PCRecvBuff[6];
    tempInt16 = (b1 << 8) + b0;
    SatellitesInfo.polerMode = tempInt16;

    //获取载波频率
    b3 = PCRecvBuff[7];
    b2 = PCRecvBuff[8];
    b1 = PCRecvBuff[9];
    b0 = PCRecvBuff[10];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
    SatellitesInfo.freq = tempInt32;

    //符码率
    b3 = PCRecvBuff[11];
    b2 = PCRecvBuff[12];
    b1 = PCRecvBuff[13];
    b0 = PCRecvBuff[14];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
    SatellitesInfo.symbolRate = tempInt32;

    //信标频率
    b3 = PCRecvBuff[15];
    b2 = PCRecvBuff[16];
    b1 = PCRecvBuff[17];
    b0 = PCRecvBuff[18];
    tempInt32 = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0;
    SatellitesInfo.freq = tempInt32;

    //DVB接收机设置
    Senser.SateInfo.freq = SatellitesInfo.freq;
    BeaconInfo.freq = SatellitesInfo.freq - SatellitesInfo.localOsci;

    SatellitesInfo.IsStoreData = 1;
    WorkMode = Mode_AimSatellite;
    //WorkMode = Mode_Stand_By;
    Senser.AgcMAx.AZg = 0;
    Senser.AgcMAx.ELg = 0;
    Senser.AgcMAx.Agc = 0;
    MotoCtr.Timer.az = 0;
    MotoCtr.Timer.el = 0;
    MotoCtr.PIDAZ.integral = 0;
}

// 获取调试值
void GetDebugValues(float* p1, float* p2, float* p3, float* p4)
{
    long int tempInt32 = 0;
    unsigned char b3 = 0, b2 = 0, b1 = 0, b0 = 0;
    b3 = PCRecvBuff[7];
    b2 = PCRecvBuff[8];
    b1 = PCRecvBuff[9];
    b0 = PCRecvBuff[10];
    tempInt32 = ((b3 << 24) & 0xFF000000)
        + ((b2 << 16) & 0x00FF0000)
        + ((b1 << 8) & 0x0000FF00)
        + b0;
    *p1 = tempInt32 * 0.01;

    b3 = PCRecvBuff[11];
    b2 = PCRecvBuff[12];
    b1 = PCRecvBuff[13];
    b0 = PCRecvBuff[14];
    tempInt32 = ((b3 << 24) & 0xFF000000)
        + ((b2 << 16) & 0x00FF0000)
        + ((b1 << 8) & 0x0000FF00)
        + b0;
    *p2 = tempInt32 * 0.01;

    b3 = PCRecvBuff[15];
    b2 = PCRecvBuff[16];
    b1 = PCRecvBuff[17];
    b0 = PCRecvBuff[18];
    tempInt32 = ((b3 << 24) & 0xFF000000)
        + ((b2 << 16) & 0x00FF0000)
        + ((b1 << 8) & 0x0000FF00)
        + b0;
    *p3 = tempInt32 * 0.01;

    b3 = PCRecvBuff[19];
    b2 = PCRecvBuff[20];
    b1 = PCRecvBuff[21];
    b0 = PCRecvBuff[22];
    tempInt32 = ((b3 << 24) & 0xFF000000)
        + ((b2 << 16) & 0x00FF0000)
        + ((b1 << 8) & 0x0000FF00)
        + b0;
    *p4 = tempInt32 * 0.01;
}

// 调试模式指令获取
void GetCtrObjects(unsigned char CtrObjects,
    unsigned char* ob1, unsigned char* ob2,
    unsigned char* ob3, unsigned char* ob4)
{
    //D7~D4	无效，保留
    //D0	方位轴;0=无效,1=有效
    //D1	俯仰轴;0=无效,1=有效
    //D2	横滚轴;0=无效,1=有效
    //D3	极化轴;0=无效,1=有效

    if ((CtrObjects & 0x01) == 0x01)
        *ob1 = 1;
    if ((CtrObjects & 0x02) == 0x02)
        *ob2 = 1;
    if ((CtrObjects & 0x04) == 0x04)
        *ob3 = 1;
    if ((CtrObjects & 0x08) == 0x08)
        *ob4 = 1;
}

// 速度环调试，值设置
void ParamsSetSpeed(unsigned char CtrObjects, float p1, float p2, float p3, float p4)
{
    unsigned char enAZ = 0, enEL = 0, enROLL = 0, enPOL = 0;
    GetCtrObjects(CtrObjects, &enAZ, &enEL, &enROLL, &enPOL);
    if (enAZ == 1) {
        MotoCtr.AZSpeed = p1;
    }
    if (enEL == 1) {
        MotoCtr.ESpeed = p2;
    }
    if (enROLL == 1) {
        MotoCtr.ROLLSpeed = p3;
    }
    if (enPOL == 1) {
        MotoCtr.POLSpeed = p4;
    }
}

// 位置环调试，值设置
void ParamsSetPosition(unsigned char CtrObjects, float p1, float p2, float p3, float p4)
{
    unsigned char enAZ = 0, enEL = 0, enROLL = 0, enPOL = 0;
    GetCtrObjects(CtrObjects, &enAZ, &enEL, &enROLL, &enPOL);

    if (enAZ == 1) {
        MotoCtr.AZEnalbe = 1;
        MotoCtr.AZPerset = p1;

        MotoCtr.PIDAZ.P = 0;
        MotoCtr.PIDAZ.I = 0;
        MotoCtr.PIDAZ.D = 0;
        MotoCtr.PIDAZ.count = 0;
        MotoCtr.PIDAZ.Value_Last = 0;
        MotoCtr.PIDAZ.startPosition = Senser.Angle_AZ[10];

        MotoCtr.Timer.az = 0;
    } else {
        MotoCtr.AZEnalbe = 0;
    }

    if (enEL == 1) {
        MotoCtr.ELEnalbe = 1;
        MotoCtr.ELPerset = p2;

        MotoCtr.PIDEL.P = 0;
        MotoCtr.PIDEL.I = 0;
        MotoCtr.PIDEL.D = 0;

        MotoCtr.Timer.el = 0;
    } else {
        MotoCtr.ELEnalbe = 0;
    }

    if (enROLL == 1) {
        MotoCtr.ROLLEnalbe = 1;
        MotoCtr.ROLLPerset = p3;

        MotoCtr.PIDROLL.P = 0;
        MotoCtr.PIDROLL.I = 0;
        MotoCtr.PIDROLL.D = 0;
    } else {
        MotoCtr.ROLLEnalbe = 0;
    }

    if (enPOL == 1) {
        MotoCtr.POLEnalbe = 1;
        MotoCtr.POLPerset = p4;
        MotoCtr.PIDPOL.P = 0;
        MotoCtr.PIDPOL.I = 0;
        MotoCtr.PIDPOL.D = 0;
    }

    //MotoCtr.ValueP_Pol = 0;
    //MotoCtr.ValueI_Pol = 0;
    //MotoCtr.ValueD_Pol = 0;
}

// 速度环正弦，设置值
void ParamsSetSpeedSin(unsigned char CtrObjects, float p1, float p2, float p3, float p4)
{
    unsigned char enAZ = 0, enEL = 0, enROLL = 0, enPOL = 0;
    GetCtrObjects(CtrObjects, &enAZ, &enEL, &enROLL, &enPOL);

    //	MotoCtr.PeriodCountTotal = 1;
    //	MotoCtr.PeriodCountSinAZ = 0;
    //	MotoCtr.PeriodCountSinEL = 0;
    //	MotoCtr.PeriodCountSinPTx = 0;
    //	MotoCtr.PeriodCountSinPRx = 0;
}

// 位置环正弦，设置值
void ParamsSetPositionSin(unsigned char CtrObjects, float p1, float p2, float p3, float p4)
{
    unsigned char enAZ = 0, enEL = 0, enROLL = 0, enPOL = 0;
    GetCtrObjects(CtrObjects, &enAZ, &enEL, &enROLL, &enPOL);

    if (enAZ == 1) {
        MotoCtr.AZEnalbe = 1;
        MotoCtr.PeriodCountTotal = 1;
        MotoCtr.PeriodCountSinAZ = 0;

        MotoCtr.AZCenter = p1 - SSINRANGE;
        TrackParams.StartTimeCountAZ = 0;
        MotoCtr.PIDAZ.P = 0;
        MotoCtr.PIDAZ.I = 0;
        MotoCtr.PIDAZ.D = 0;
        MotoCtr.Timer.az = 0;
    } else {
        MotoCtr.AZEnalbe = 0;
    }

    if (enEL == 1) {
        MotoCtr.ELEnalbe = 1;
        MotoCtr.ELCenter = p2;

        MotoCtr.PeriodCountTotal = 1;
        MotoCtr.PeriodCountSinEL = 0;
        TrackParams.StartTimeCountEL = 0;

        MotoCtr.PIDEL.P = 0;
        MotoCtr.PIDEL.I = 0;
        MotoCtr.PIDEL.D = 0;

        MotoCtr.Timer.el = 0;
    } else {
        MotoCtr.ELEnalbe = 0;
    }

    if (enROLL == 1) {
        MotoCtr.ROLLEnalbe = 1;
        MotoCtr.ROLLCenter = p3;

        MotoCtr.PeriodCountTotal = 1;
        MotoCtr.PeriodCountSinROLL = 0;

        MotoCtr.PIDROLL.P = 0;
        MotoCtr.PIDROLL.I = 0;
        MotoCtr.PIDROLL.D = 0;
    } else {
        MotoCtr.ROLLEnalbe = 0;
    }

    if (enPOL == 1) {
        MotoCtr.POLEnalbe = 1;
        MotoCtr.PolRxCenter = p4;

        MotoCtr.PeriodCountTotal = 1;
        MotoCtr.PeriodCountSinPRx = 0;
        MotoCtr.PIDPOL.P = 0;
        MotoCtr.PIDPOL.I = 0;
        MotoCtr.PIDPOL.D = 0;
    } else {
        MotoCtr.POLEnalbe = 0;
    }

    //MotoCtr.PeriodCountSinPol = 0;
    //MotoCtr.ValueP_Pol = 0;
    //MotoCtr.ValueI_Pol = 0;
    //MotoCtr.ValueD_Pol = 0;
}

// 解析调试设置报文
void AnalysisDebugCMD()
{
    float p1 = 0, p2 = 0, p3 = 0, p4 = 0;
    unsigned char CtrObjects = 0;
    unsigned char DebugType = 0;

    //调试对象
    CtrObjects = PCRecvBuff[6];

    //取数据
    GetDebugValues(&p1, &p2, &p3, &p4);

    //取调试模式
    DebugType = PCRecvBuff[4];
    switch (DebugType) {
    case 0x03:
        //角速度预置
        WorkMode = Mode_Debug_Speed;
        ParamsSetSpeed(CtrObjects, p1, p2, p3, p4);
        break;
    case 0x04:
        //位置预置
        WorkMode = Mode_Debug_Position;
        ParamsSetPosition(CtrObjects, p1, p2, p3, p4);
        break;
    case 0x05:
        //速度环正弦
        WorkMode = Mode_Debug_Speed_Sin;
        ParamsSetSpeedSin(CtrObjects, p1, p2, p3, p4);
        break;
    case 0x06:
        //位置环正弦
        WorkMode = Mode_Debug_Position_Sin;
        ParamsSetPositionSin(CtrObjects, p1, p2, p3, p4);
        break;
    case 0x07:
        //惯导修正
        GDAmand.IsAmand = 1;
        GDAmand.Value = p1;
        GDAmand.isFirst = 1;
        break;
    case 0x08:
        //航向校准
        HeadCheckFlag = 0;
        HeadCheckParams.State = 0;
        Senser.AgcMAx.Agc = 0;
        break;
    case 0x09:
        //接收机设置
        if (p1 == 0.0f) {
            //DVB 模式
            Senser.BeaconInfo.mode = RecvMode_DVB;
        } else if (p1 == 1.0f) {
            //信标模式
            Senser.BeaconInfo.mode = RecvMode_Xinbiao;
        }
        break;
    case 0x0A:
        //惯导零位标定
        MemsZeroCheck();
        break;
    case 0x0B:
        //设置接收机斜率
        break;
    case 0x0C:
        //功放开关
        if (p2 == 1) {
            BUC_Ctrl(SET_REBOOT, 0);
        } else {
            if (p1 == 0) {
                BUC_Ctrl(SET_SWITCH, BUC_OFF);
            } else if (p1 == 1) {
                BUC_Ctrl(SET_SWITCH, BUC_ON);
            }
        }
        break;
    case 0x0D:
        //功放增益
        BUC_Ctrl(SET_GAIN, BUC_CoverGain(p1));
        break;
    case 0x0E:
        //功放IP
        BUC_Ctrl(SET_IP_ADDR, BUC_CoverIP(p1, p2, p3, p4));
        break;
    default:
        break;
    }
}

// 解算上位机指令
void AnalysisDataPC(void)
{
    unsigned char CtrCMD;

    if (IsGetNewCMD_PC() != 1)
        return;

    if (PCRecvBuff[0] == 0xAA && PCRecvBuff[1] == 0x55) {
        AnalysisACU();
        memset(PCRecvBuff, 0, 25 * sizeof(uint8_t));
        PCRecvCount = 0;
        return;
    }
    //校验和计算
    if (CheckCMDHead_PC() != 1)
        return;

    //取控制字
    CtrCMD = PCRecvBuff[2];

    //若工作模式处于手动控制模式
    //此时只能响应手动控制使能指令
    //及手动控制角速度指令
    if (HandControlEnable == 1) {
        switch (CtrCMD) {
        case 0x04:
            //手动控制使能指令
            AnalysisHandCtrEnable();
            break;
        case 0x05:
            //手动控制角速度设置指令
            AnalysisHandCtrSpeedSet();
            break;
        }
    } else {
        switch (CtrCMD) {
        case 0x00:
            //上传报文设置
            AnalysisMsgSet();
            break;
        case 0x01:
            //参数设置
            AnalysisParamsSet();
            break;
        case 0x02:
            //解析工作模式设置报文
            AnalysisWorkModeSet();
            break;
        case 0x03:
            //解析调试设置报文
            AnalysisDebugCMD();
            break;
        case 0x04:
            //解析手动控制使能报文
            AnalysisHandCtrEnable();
            break;
        case 0x06:
            //解析卫星参数设置报文
            AnalysisSateInfoSet();
            break;
        default:
            AnalysisDebugCMD();
            break;
        }
    }
}
