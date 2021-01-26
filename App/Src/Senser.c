#include "Senser.h"

#include "math.h"
#include "stdlib.h"
#include "string.h"

#include "MainDefine.h"
#include "Parameters.h"
#include "ParametersDefine.h"
#include "beacon.h"
#include "buc.h"
#include "main.h"
#include "uart_api.h"

SENSER Senser;
float AGC_Filter_Temp[51];

uint8_t IN_GetBuffer[MEMS_RECV_LENGTH];

volatile int IN_RX_Completed;

// 解算MEMS惯导数据
void AnalysisMEMS(void);
// 解算接收机数据
void AnalysisBC(void);

// 解算方位编码器
void AnalysisAZEncoder(void);
// 解算俯仰编码器数据
void AnalysisELEncoder(void);
// 解算极化编码器数据
void AnalysisPOLEncoder(void);
// 解算横滚编码器数据
void AnalysisROLLEncoder(void);

//计算方位角速度
void GetSpeedRealAZ(void);
//计算俯仰角速度
void GetSpeedRealEL(void);
//计算横滚角速度
void GetSpeedRealROLL(void);
//计算极化角速度
void GetSpeedRealPOL(void);

extern uint8_t MDRecvFlag;
// 解算各传感器数据，包括编码器、MEMS、DVB
void DataCollection(void)
{
    if (DVBRecvFlag == 1) {
        AnalysisBC();
        memset(DVBRecvBuff, 0, 50);
        DVBRecvFlag = 0;
    } else {
        Senser.BeaconInfo.LostCount++;
        if (Senser.BeaconInfo.LostCount > 65500) {
            Senser.BeaconInfo.LostCount = 0;
        }
        Senser.BeaconInfo.BugTime++;
    }

    if (BUC_RecvFlag == 1) {
        AnalysisBUC(BUC_RecvBuff, BUC_RecvLength);
        memset(BUC_RecvBuff, 0, 50);
        BUC_RecvFlag = 0;
        BUC_RecvCount = 0;
        BUC_RecvLength = 0;
    }

    //解算编码器数据
    AnalysisAZEncoder();
    AnalysisELEncoder();
    AnalysisROLLEncoder();
    AnalysisPOLEncoder();

    //计算角速度
    GetSpeedRealAZ();
    GetSpeedRealEL();
    GetSpeedRealROLL();
    GetSpeedRealPOL();

    NewConvertBTOG(Senser.Angle_AZ[10], Senser.Angle_EL[10],
        Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
        &Senser.AZg[10], &Senser.ELg[10]);

    //存储AGC信号最大值及其对应的空间位置
    StoreAgcMax();
}

// 解析MEMS惯导数据
void AnalysisMEMS(void)
{
    uint8_t HByte = 0;
    uint8_t LByte = 0;
    uint8_t CheckResult = 0;
    int16_t tempInt16 = 0;
    uint16_t tempUInt16 = 0;
    int32_t i32Temp = 0;
    int i = 0;
    uint8_t Byte1 = 0;
    uint8_t Byte2 = 0;
    uint8_t Byte3 = 0;
    uint8_t Byte4 = 0;

    //判断帧头
    if (IN_GetBuffer[0] != 0xAA || IN_GetBuffer[1] != 0x55) {
        Senser.MEMSInfo.ErrCount++;
        if (Senser.MEMSInfo.ErrCount > 200) {
            Senser.MEMSInfo.ErrCount = 0;
        }
        Senser.MEMSInfo.BugTime++;
        return;
    }

#if MEMS_TYPE == 1
    //校验和计算
    for (i = 0; i < MEMS_RECV_LENGTH - 1; i++) {
        CheckResult += IN_GetBuffer[i];
    }
    if (CheckResult != IN_GetBuffer[MEMS_RECV_LENGTH - 1]) {
        Senser.MEMSInfo.ErrCount++;
        if (Senser.MEMSInfo.ErrCount > 200) {
            Senser.MEMSInfo.ErrCount = 0;
        }
        return;
    }

    //获取惯导状态
    //Senser.MEMSInfo.Status = IN_GetBuffer[11];

    //纵摇角
    HByte = IN_GetBuffer[23];
    LByte = IN_GetBuffer[24];
    tempInt16 = (HByte << 8) + LByte;
    for (i = 0; i < 10; i++)
        Senser.Pitch[i] = Senser.Pitch[i + 1];
    Senser.Pitch[10] = tempInt16 * 0.01f + OFFSETPITCH;

    //横滚角
    HByte = IN_GetBuffer[25];
    LByte = IN_GetBuffer[26];
    tempInt16 = (HByte << 8) + LByte;
    for (i = 0; i < 10; i++)
        Senser.Roll[i] = Senser.Roll[i + 1];
    Senser.Roll[10] = tempInt16 * 0.01f + OFFSETMEMSROLL;

    //偏航角
    HByte = IN_GetBuffer[27];
    LByte = IN_GetBuffer[28];
    tempInt16 = (HByte << 8) + LByte;
    Senser.HeadNew = -tempInt16 * 0.01f;
    //	Senser.HeadNew = 360 - Senser.HeadNew;

    for (i = 0; i < 10; i++) {
        Senser.Head[i] = Senser.Head[i + 1];
    }
    // Senser.Head[10] = Senser.HeadNew;
    Senser.Head[10] = Senser.HeadNew + Senser.HeadOffset;
    if (Senser.Head[10] >= 360) {
        Senser.Head[10] -= 360;
    }
    if (Senser.Head[10] < 0) {
        Senser.Head[10] += 360;
    }

    if ((IN_GetBuffer[11] & 0x04) == 0x04) {
        //经度
        i32Temp = (IN_GetBuffer[12] << 24) + (IN_GetBuffer[13] << 16) + (IN_GetBuffer[14] << 8) + (IN_GetBuffer[15] << 0);
        Senser.GPSX = i32Temp / 10000000.0f;

        //纬度
        i32Temp = (IN_GetBuffer[16] << 24) + (IN_GetBuffer[17] << 16) + (IN_GetBuffer[18] << 8) + (IN_GetBuffer[19] << 0);
        Senser.GPSY = i32Temp / 10000000.0f;
        Senser.MEMSInfo.Status = 0xF0;
    }
#elif MEMS_TYPE == 0
    //校验和计算
    for (i = 0; i < MEMS_RECV_LENGTH; i++) {
        CheckResult += IN_GetBuffer[i];
    }
    if (CheckResult != 0) {
        Senser.MEMSInfo.ErrCount++;
        if (Senser.MEMSInfo.ErrCount > 200) {
            Senser.MEMSInfo.ErrCount = 0;
        }
        Senser.MEMSInfo.BugTime++;
        return;
    }

    Senser.MEMSInfo.BugTime = 0;
    //获取惯导状态
    Senser.MEMSInfo.Status = IN_GetBuffer[2];

    //纵摇角
    LByte = IN_GetBuffer[9];
    HByte = IN_GetBuffer[10];
    tempInt16 = (HByte << 8) + LByte;
    for (i = 0; i < 10; i++)
        Senser.Pitch[i] = Senser.Pitch[i + 1];
    Senser.Pitch[10] = -tempInt16 * 0.01f + OFFSETPITCH;

    //横滚角
    LByte = IN_GetBuffer[11];
    HByte = IN_GetBuffer[12];
    tempInt16 = (HByte << 8) + LByte;
    for (i = 0; i < 10; i++)
        Senser.Roll[i] = Senser.Roll[i + 1];
    Senser.Roll[10] = -tempInt16 * 0.01f + OFFSETMEMSROLL;

    //偏航角
    LByte = IN_GetBuffer[13];
    HByte = IN_GetBuffer[14];
    tempUInt16 = (HByte << 8) + LByte;
    Senser.HeadNew = tempUInt16 * 0.01f;
    for (i = 0; i < 10; i++) {
        Senser.Head[i] = Senser.Head[i + 1];
    }
    Senser.Head[10] = Senser.HeadNew + Senser.HeadOffset;
    if (Senser.Head[10] >= 360) {
        Senser.Head[10] -= 360;
    }
    if (Senser.Head[10] < 0) {
        Senser.Head[10] += 360;
    }

    //经度
    LByte = IN_GetBuffer[15];
    HByte = IN_GetBuffer[16];
    tempInt16 = (HByte << 8) + LByte;
    if (Senser.MemsFlag == 0) {
        Senser.GPSX = tempInt16 / 180.0f;
    } else {
        Senser.GPSX = tempInt16 * 0.01;
    }

    //纬度
    LByte = IN_GetBuffer[17];
    HByte = IN_GetBuffer[18];
    tempInt16 = (HByte << 8) + LByte;
    if (Senser.MemsFlag == 0) {
        Senser.GPSY = tempInt16 / 360.0f;
    } else {
        Senser.GPSY = tempInt16 * 0.01;
    }

    //高度
    LByte = IN_GetBuffer[19];
    HByte = IN_GetBuffer[20];
    tempInt16 = (HByte << 8) + LByte;
    Senser.GPSH = tempInt16;
    //Senser.GPSH = 70;

    //-------------------------------------//
    //注意！
    //惯导输出的X角速度对应横滚轴的角速度
    //惯导输出的Y角速度对应纵摇轴的角速度
    //此处取值将X、Y互换
    //------------------------------------//

    //Y轴角速率
    LByte = IN_GetBuffer[3];
    HByte = IN_GetBuffer[4];
    tempInt16 = (HByte << 8) + LByte;
    Senser.MemsY = tempInt16 * 0.0033f;

    //X轴角速率
    LByte = IN_GetBuffer[5];
    HByte = IN_GetBuffer[6];
    tempInt16 = (HByte << 8) + LByte;
    Senser.MemsX = tempInt16 * 0.0033f;

    //Z轴角速率
    LByte = IN_GetBuffer[7];
    HByte = IN_GetBuffer[8];
    tempInt16 = (HByte << 8) + LByte;
    Senser.MemsZ = tempInt16 * 0.0033f;
#elif MEMS_TYPE == 2
    //校验和计算
    for (i = 0; i < MEMS_RECV_LENGTH; i++) {
        CheckResult += IN_GetBuffer[i];
    }
    if (CheckResult != 0) {
        Senser.MEMSInfo.ErrCount++;
        if (Senser.MEMSInfo.ErrCount > 200) {
            Senser.MEMSInfo.ErrCount = 0;
        }
        Senser.MEMSInfo.BugTime++;
        return;
    }

    Senser.MEMSInfo.BugTime = 0;
    //获取惯导状态
    Senser.MEMSInfo.Status = IN_GetBuffer[2];

    //纵摇角
    LByte = IN_GetBuffer[9];
    HByte = IN_GetBuffer[10];
    tempInt16 = (HByte << 8) + LByte;
    for (i = 0; i < 10; i++)
        Senser.Pitch[i] = Senser.Pitch[i + 1];
    Senser.Pitch[10] = -tempInt16 * 0.01f + OFFSETPITCH;

    //横滚角
    LByte = IN_GetBuffer[11];
    HByte = IN_GetBuffer[12];
    tempInt16 = (HByte << 8) + LByte;
    for (i = 0; i < 10; i++)
        Senser.Roll[i] = Senser.Roll[i + 1];
    Senser.Roll[10] = -tempInt16 * 0.01f + OFFSETMEMSROLL;

    //偏航角
    LByte = IN_GetBuffer[13];
    HByte = IN_GetBuffer[14];
    tempUInt16 = (HByte << 8) + LByte;
    Senser.HeadNew = tempUInt16 * 0.01f;
    for (i = 0; i < 10; i++) {
        Senser.Head[i] = Senser.Head[i + 1];
    }
    Senser.Head[10] = Senser.HeadNew + Senser.HeadOffset;
    if (Senser.Head[10] >= 360) {
        Senser.Head[10] -= 360;
    }
    if (Senser.Head[10] < 0) {
        Senser.Head[10] += 360;
    }

    //经度
    Byte1 = IN_GetBuffer[15];
    Byte2 = IN_GetBuffer[16];
    Byte3 = IN_GetBuffer[17];
    Byte4 = IN_GetBuffer[18];
    Senser.GPSX = ((Byte1 << 0) + (Byte2 << 8) + (Byte3 << 16) + (Byte4 << 24)) / 10000000.0f;

    //纬度
    Byte1 = IN_GetBuffer[19];
    Byte2 = IN_GetBuffer[20];
    Byte3 = IN_GetBuffer[21];
    Byte4 = IN_GetBuffer[22];
    Senser.GPSY = ((Byte1 << 0) + (Byte2 << 8) + (Byte3 << 16) + (Byte4 << 24)) / 10000000.0f;

    //高度
    Byte1 = IN_GetBuffer[23];
    Byte2 = IN_GetBuffer[24];
    Senser.GPSH = ((Byte1 << 0) + (Byte2 << 8));
    //-------------------------------------//
    //注意！
    //惯导输出的X角速度对应横滚轴的角速度
    //惯导输出的Y角速度对应纵摇轴的角速度
    //此处取值将X、Y互换
    //------------------------------------//

    //Y轴角速率
    LByte = IN_GetBuffer[3];
    HByte = IN_GetBuffer[4];
    tempInt16 = (HByte << 8) + LByte;
    Senser.MemsY = tempInt16 * 0.0033f;

    //X轴角速率
    LByte = IN_GetBuffer[5];
    HByte = IN_GetBuffer[6];
    tempInt16 = (HByte << 8) + LByte;
    Senser.MemsX = tempInt16 * 0.0033f;

    //Z轴角速率
    LByte = IN_GetBuffer[7];
    HByte = IN_GetBuffer[8];
    tempInt16 = (HByte << 8) + LByte;
    Senser.MemsZ = tempInt16 * 0.0033f;
#endif
}

//测试数组，存储接收到的DVB接收数据，通过PC通信串口直接输出至上位机
unsigned char DVBRecvBuffTest[20];

void AnalysisBCBuff(uint8_t iCnt)
{
    if (DVBRecvBuff[2] != '=') {
        return;
    }
    switch (DVBRecvBuff[1 + iCnt]) {
    case 0x09: //帧头
        Senser.BeaconInfo.mode = DVBRecvBuff[3 + iCnt];
        break;
    case 0x0C:
        break;
    case 0x0A:
        if (DVBRecvBuff[3 + iCnt] != 10) {
            BC_SetRefreshRate(10);
        }
        break;
    case 0x10:
        if (DVBRecvBuff[4 + iCnt] != 1) {
            BC_SetVMapping(1);
        }
        break;
    case 0x05: //馈电
        Senser.BeaconInfo.feed = DVBRecvBuff[3 + iCnt];
        break;
    case 0x06: //22K
        Senser.BeaconInfo.mono = DVBRecvBuff[3 + iCnt];
        break;
    case 0x0F:
        Senser.Agc[10] = ((DVBRecvBuff[3 + iCnt] << 8) + DVBRecvBuff[4 + iCnt]) / 100.0;

        break;
    case 0x02:
        break;
    case 0x08:
        // New
        Senser.BeaconInfo.freq = ((DVBRecvBuff[3 + iCnt] << 24) + (DVBRecvBuff[4 + iCnt] << 16) + (DVBRecvBuff[5 + iCnt] << 8) + (DVBRecvBuff[6 + iCnt] << 0)) / 1000;

        break;
    case 0x07: //信标模式功率电平
        break;
    case 0x0D: //检波模式功率电平
        break;
    case 'V': //版本信息
        break;
    case 0x03: //DVB参数(高级)
        break;
    case 0x0B: //DVB参数(标准)
        Senser.BeaconInfo.freq = ((DVBRecvBuff[3 + iCnt] << 24)
                                     + (DVBRecvBuff[4 + iCnt] << 16)
                                     + (DVBRecvBuff[5 + iCnt] << 8)
                                     + (DVBRecvBuff[6 + iCnt] << 0))
            / 1000;

        Senser.BeaconInfo.symbolRate = ((DVBRecvBuff[7 + iCnt] << 24)
            + (DVBRecvBuff[8 + iCnt] << 16)
            + (DVBRecvBuff[9 + iCnt] << 8)
            + (DVBRecvBuff[10 + iCnt] << 0));
        break;
    case 0x11: //DVB 载噪比映射范围
        break;
    case 0x12: //信标电平映射范围
        break;
    case 0x13: //检波电平映射范围
        break;
    case 0x0E: //检波参数
        Senser.BeaconInfo.freq = ((DVBRecvBuff[3 + iCnt] << 24)
                                     + (DVBRecvBuff[4 + iCnt] << 16)
                                     + (DVBRecvBuff[5 + iCnt] << 8)
                                     + (DVBRecvBuff[6 + iCnt] << 0))
            / 1000;

        Senser.BeaconInfo.symbolRate = ((DVBRecvBuff[7 + iCnt] << 24)
            + (DVBRecvBuff[8 + iCnt] << 16)
            + (DVBRecvBuff[9 + iCnt] << 8)
            + (DVBRecvBuff[10 + iCnt] << 0));

        Senser.BeaconInfo.range = ((DVBRecvBuff[11 + iCnt] << 8)
            + (DVBRecvBuff[12 + iCnt] << 0));

        Senser.BeaconInfo.roll = DVBRecvBuff[13];
        break;
    default:
        break;
    }

    memset(DVBRecvBuff, 0, 50);
}

void AnalysisBC(void)
{
    uint8_t iCnt = 0;
    for (iCnt = 0; iCnt < DVBRecvBuffLenth && iCnt < (50 - 14); iCnt++) {
        if (DVBRecvBuff[iCnt] == 0x7B && DVBRecvBuff[iCnt + DVBRecvBuffLenth - 1] == 0x7D) {
            AnalysisBCBuff(iCnt);
            Senser.BeaconInfo.BugTime = 0;
            return;
        }
    }
    Senser.BeaconInfo.ErrCount++;
    if (Senser.BeaconInfo.ErrCount > 9999) {
        Senser.BeaconInfo.ErrCount = 0;
    }
    Senser.BeaconInfo.BugTime++;

    memset(DVBRecvBuff, 0, 50);
}

//自跟踪模式下AGC滤波
void AGCFilter()
{
    uint8_t i = 0;
    float sum = 0;
    sum = 0;
    for (i = 0; i < 10; i++) {
        sum += Senser.Agc[i];
    }
    if (Senser.Agc[10] < (sum / 10.0f - 2.0f)) {
        Senser.Agc[10] = Senser.Agc[9];
    }
}

extern uint8_t MDRecvBuff[20];
extern uint8_t MDRecvBuffLenth;
// 解算方位编码器数据
void AnalysisAZEncoder()
{
    int32_t _Count = 0;
    int32_t TempInt32 = 0;
    unsigned char i = 0;

    unsigned short int xianshu = 1024; //编码器线数
    unsigned char beipinshu = 4; //编码器4倍频
    float ratRdt = 43.56; //减速器减速比
    float ratGear = (180 / 25.0); //齿轮齿数比
    long int bili = 0;
    for (i = 0; i < 10; i++) {
        Senser.Angle_AZ[i] = Senser.Angle_AZ[i + 1];
    }

    Senser.AZEncoder.Count = AZ_EC_TIM->CNT;
    _Count = -Senser.AZEncoder.Count + Senser.AZEncoder.PeriodCount * 40000;
    //关于为什么乘以40000的说明：
    //在编码器count计数定时器中配置为40000溢出，即Senser.AZEncoder.PeriodCount没40000个Count加一或减一
    //与编码器分辨率无关

    bili = xianshu * beipinshu * ratRdt * ratGear;
    TempInt32 = (_Count - Senser.AngleOffset_AZ) % bili;
    Senser.Angle_AZ[10] = TempInt32 * (360 / (bili * 1.0f)) + OFFSETAZ;
    if (Senser.Angle_AZ[10] < 0) {
        Senser.Angle_AZ[10] += 360;
    }
    if (Senser.Angle_AZ[10] >= 360) {
        Senser.Angle_AZ[10] -= 360;
    }
}

// 解算俯仰编码器数据
void AnalysisELEncoder(void)
{
    int32_t _CountEL = 0;
    int32_t TempInt32 = 0;
    int i = 0;

    unsigned short int xianshu = 1024; //编码器线数
    unsigned char beipinshu = 4; //编码器4倍频
    float ratRdt = 29.7; //减速器减速比
    float ratGear = (130 / 25.0); //齿轮齿数比
    long int bili = 0;

    Senser.ELEncoder.Count = EL_EC_TIM->CNT;
    _CountEL = Senser.ELEncoder.Count + Senser.ELEncoder.PeriodCount * 40000;
    /*关于为什么乘以40000的说明:
	*在编码器Count计数器定时器中配置为40000溢出
	*即Senser.ELEncoder.PeriodCount每40000个Count加一或减一
	*与编码器分辨率无关
	*/
    for (i = 0; i < 10; i++)
        Senser.Angle_EL[i] = Senser.Angle_EL[i + 1];

    bili = xianshu * beipinshu * ratRdt * ratGear;
    TempInt32 = (_CountEL - Senser.AngleOffset_EL) % bili;
    Senser.Angle_EL[10] = TempInt32 * (360 / (bili * 1.0f)) + OFFSETEL;
    if (Senser.Angle_EL[10] >= 360) {
        Senser.Angle_EL[10] -= 360;
    }
}

// 解算极化编码器数据
void AnalysisPOLEncoder(void)
{
    int32_t _Count = 0;
    int32_t TempInt32 = 0;
    int i = 0;

    unsigned short int xianshu = 512; //编码器线数
    unsigned char beipinshu = 4; //编码器4倍频
    float ratRdt = 1; //减速器减速比
    float ratGear = 1; //齿轮齿数比
    long int bili = 0;

    Senser.POLEncoder.Count = POL_EC_TIM->CNT;
    _Count = Senser.POLEncoder.Count + Senser.POLEncoder.PeriodCount * 40000;
    /*关于为什么乘以40000的说明:
	*在编码器Count计数器定时器中配置为40000溢出
	*即Senser.POLEncoder.PeriodCount每40000个Count加一或减一
	*与编码器分辨率无关
	*/

    for (i = 0; i < 10; i++)
        Senser.Angle_POL[i] = Senser.Angle_POL[i + 1];

    bili = xianshu * beipinshu * ratRdt * ratGear;
    TempInt32 = (_Count - Senser.AngleOffset_POL) % bili;
    Senser.Angle_POL[10] = TempInt32 * (360 / (bili * 1.0f)) + OFFSETPOL;
    if (Senser.Angle_POL[10] >= 360) {
        Senser.Angle_POL[10] -= 360;
    }

    if (Senser.Angle_POL[10] < 0) {
        Senser.Angle_POL[10] += 360;
    }
}

// 解算横滚编码器数据
void AnalysisROLLEncoder(void)
{
    int32_t _Count = 0;
    int32_t TempInt32 = 0;
    int i = 0;

    unsigned short int xianshu = 512; //编码器线数
    unsigned char beipinshu = 4; //编码器4倍频
    float ratRdt = 1; //减速器减速比
    float ratGear = 1; //齿轮齿数比
    long int bili = 0;

    Senser.ROLLEncoder.Count = RL_EC_TIM->CNT;
    _Count = Senser.ROLLEncoder.Count + Senser.ROLLEncoder.PeriodCount * 40000;
    /*关于为什么乘以40000的说明:
	*在编码器Count计数器定时器中配置为40000溢出
	*即Senser.ROLLEncoder.PeriodCount每40000个Count加一或减一
	*与编码器分辨率无关
	*/

    for (i = 0; i < 10; i++)
        Senser.Angle_ROLL[i] = Senser.Angle_ROLL[i + 1];

    bili = xianshu * beipinshu * ratRdt * ratGear;
    TempInt32 = (_Count - Senser.AngleOffset_ROLL) % bili;
    Senser.Angle_ROLL[10] = TempInt32 * (360 / (bili * 1.0f)) + OFFSETROLL;
    if (Senser.Angle_ROLL[10] >= 360) {
        Senser.Angle_ROLL[10] -= 360;
    }

    if (Senser.Angle_ROLL[10] < 0) {
        Senser.Angle_ROLL[10] += 360;
    }
}

// 计算方位角速度
void GetSpeedRealAZ()
{
    float tempspeed = 0;
    tempspeed = Senser.Angle_AZ[10] - Senser.Angle_AZ[9];
    if (tempspeed > 200)
        tempspeed -= 360;
    else if (tempspeed < -200)
        tempspeed += 360;
    Senser.SpeedAZ = tempspeed * _CTRCLK;
}

// 计算俯仰角速度
void GetSpeedRealEL()
{
    float tempspeed = 0;
    tempspeed = Senser.Angle_EL[10] - Senser.Angle_EL[9];
    if (tempspeed > 200)
        tempspeed -= 360;
    else if (tempspeed < -200)
        tempspeed += 360;
    Senser.Speed_EL = tempspeed * _CTRCLK;
}

// 计算横滚角速度
void GetSpeedRealROLL()
{
    float tempspeed = 0;
    tempspeed = (Senser.Angle_ROLL[10] - Senser.Angle_ROLL[9]);
    if (tempspeed > 200) {
        tempspeed -= 360;
    } else if (tempspeed < -200) {
        tempspeed += 360;
    }
    Senser.SpeedROLL = tempspeed * _CTRCLK;
}

// 计算极化角速度
void GetSpeedRealPOL()
{
    float tempspeed = 0;
    tempspeed = Senser.Angle_POL[10] - Senser.Angle_POL[9];
    if (tempspeed > 200)
        tempspeed -= 360;
    else if (tempspeed < -200)
        tempspeed += 360;
    Senser.SpeedPOL = tempspeed * _CTRCLK;
}

// 当惯导数据丢帧时，进行航向数据的预测
// 五点外推法
void HeadForecast()
{
    uint8_t i = 0;
    int _lookHeadTime = 10;
    float temp = 0.0;
    float tempHead[11] = { 0 };
    //方位预测
    //过0判断
    if (Senser.Head[10] > 300 || Senser.Head[10] < 60) {
        for (i = 0; i < 11; i++) {
            if (Senser.Head[i] > 180) {
                tempHead[i] = Senser.Head[i] - 360;
            } else {
                tempHead[i] = Senser.Head[i];
            }
        }
        temp = Extrapolate(tempHead, _lookHeadTime, _CTRCLK, 10);
        if (temp < 0) {
            temp += 360;
        }
    } else {
        temp = Extrapolate(Senser.Head, _lookHeadTime, _CTRCLK, 10);
    }

    for (i = 0; i < 10; i++) {
        Senser.Head[i] = Senser.Head[i + 1];
    }

    Senser.Head[10] = temp;
    temp = 0.0;

    //方位角度转换
    if (Senser.Head[10] >= 360) {
        Senser.Head[10] -= 360;
    } else if (Senser.Head[10] < 0) {
        Senser.Head[10] += 360;
    }

    //俯仰预测
    temp = Extrapolate(Senser.Pitch, _lookHeadTime, _CTRCLK, 10);

    for (i = 0; i < 10; i++) {
        Senser.Pitch[i] = Senser.Pitch[i + 1];
    }

    Senser.Pitch[10] = temp;
    temp = 0.0;
    //--

    //横滚预测
    temp = Extrapolate(Senser.Roll, _lookHeadTime, _CTRCLK, 10);

    for (i = 0; i < 10; i++) {
        Senser.Roll[i] = Senser.Roll[i + 1];
    }

    Senser.Roll[10] = temp;
    temp = 0.0;
    //--
}

// 等待惯导数据，并作为一个控制周期的开始
void GetMemsData()
{
    unsigned char recvedflag = 1;
    Control_Period_Flag = 0;

    while (IN_RX_Completed != 1) {
        if (Control_Period_Flag > 2) {
            Control_Period_Flag = 0;
            Senser.MEMSInfo.LostCount++;
            if (Senser.MEMSInfo.LostCount > 200) {
                Senser.MEMSInfo.LostCount = 0;
            }
            HeadForecast();
            recvedflag = 0;
            Senser.MEMSInfo.BugTime++;
            break;
        }
    }

    if (recvedflag == 1) {
        AnalysisMEMS();
        IN_RX_Completed = 0;
    }
}

// 保存故障信息
void BugSys(void)
{
    float AZError = 0.0, ELError = 0.0, ROLLError = 0.0, POLError = 0.0;

    Senser.BugInfo = 0;

    //方位Bug计时
    if (WorkMode != Mode_Find_Zero
        // && HeadCheckParams.State >= 3
    ) {
        AZError = (MotoCtr.AZPerset - Senser.Angle_AZ[10]);
        if (AZError > 3.0f || AZError < -3.0f) {
            Senser.AZBugTime++;
        } else {
            Senser.AZBugTime = 0;
        }

        //俯仰Bug计时
        ELError = (MotoCtr.ELPerset - Senser.Angle_EL[10]);
        if (ELError > 3.0f || ELError < -3.0f) {
            Senser.ELBugTime++;
        } else {
            Senser.ELBugTime = 0;
        }

        //横滚Bug计时
        ROLLError = (MotoCtr.ROLLPerset - Senser.Angle_ROLL[10]);
        if (ROLLError > 3.0f || ROLLError < -3.0f) {
            Senser.ROLLBugTime++;
        } else {
            Senser.ROLLBugTime = 0;
        }

        //极化Bug计时
        POLError = (MotoCtr.POLPerset - Senser.Angle_POL[10]);
        if (POLError > 3.0f || POLError < -3.0f) {
            Senser.POLBugTime++;
        } else {
            Senser.POLBugTime = 0;
        }
    } else {
        Senser.AZBugTime = 0;
        Senser.ELBugTime = 0;
        Senser.ROLLBugTime = 0;
        Senser.POLBugTime = 0;
    }

    //故障信息处理
    if (Senser.AZBugTime > 20 * _CTRCLK) {
        Senser.BugInfo |= (1 << BUG_AZ);
    }
    if (Senser.ELBugTime > 5 * _CTRCLK) {
        Senser.BugInfo |= (1 << BUG_EL);
    }
    if (Senser.ROLLBugTime > 5 * _CTRCLK) {
        Senser.BugInfo |= (1 << BUG_ROLL);
    }
    if (Senser.POLBugTime > 5 * _CTRCLK) {
        Senser.BugInfo |= (1 << BUG_POL);
    }

    if (Senser.MEMSInfo.BugTime > 5 * _CTRCLK) {
        Senser.BugInfo |= (1 << BUG_MEMS);
    }

    if (Senser.BeaconInfo.BugTime > 5 * _CTRCLK) {
        Senser.BugInfo |= (1 << BUG_RECVER);
    }

    if (Senser.AgcFindBugTime > 5 * 60 * _CTRCLK) {
        Senser.BugInfo |= (1 << BUG_AGCNOTFOUND);
    }

    if (Senser.AgcLostFlag == 1) {
        Senser.BugInfo |= (1 << BUG_AGCLOST);
    }

    if ((Senser.MEMSInfo.Status >> 4) != 9) {
        Senser.BugInfo |= (1 << BUG_GPSLOST);
    }

    if ((Senser.BugInfo & 0x1f) != 0) {
        // WorkMode = Mode_Stand_By;
    }
}
