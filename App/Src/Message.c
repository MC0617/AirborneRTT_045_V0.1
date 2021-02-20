#include "Message.h"

#include "string.h"

#include "MotorControl.h"
#include "ParametersDefine.h"
#include "Senser.h"
#include "main.h"
#include "udp_echoserver.h"

MSG msg;

#define ANT_NUM_CODE "060201811002"

#define ROLLIO (/*0x01 ^ */ HAL_GPIO_ReadPin(RX_LIM_GPIO_Port, RX_LIM_Pin))
#define ELIO (0x01 ^ HAL_GPIO_ReadPin(EL_LIM_GPIO_Port, EL_LIM_Pin))
#define AZIO (0x01 ^ HAL_GPIO_ReadPin(AZ_LIM_GPIO_Port, AZ_LIM_Pin))
#define POLIO (/*0x01 ^ */ HAL_GPIO_ReadPin(TX_LIM_GPIO_Port, TX_LIM_Pin))

extern unsigned char PCSendBuff[400];

// 拼装报文状态字（报文类型及工作模式）
unsigned char GetCh(unsigned char cmdtype, unsigned char workmode)
{
    unsigned ch1, ch2, ch;
    ch1 = (cmdtype & 0x0F) << 4;
    ch2 = workmode & 0x0F;
    ch = ch1 + ch2;
    return ch;
}

unsigned short int AllData()
{
    unsigned short int tempUInt16 = 0;
    unsigned long int tempUint32 = 0;
    long int tempInt32 = 0;
    short int tempInt16 = 0;
    unsigned char i = 0;
    unsigned char offset = 0;

    //帧头
    PCSendBuff[0] = 0xEB;
    //报文类型
    PCSendBuff[1] = GetCh(tAllData, WorkMode);
    //PCSendBuff[1] = 0x30;

    //AGC
    tempUInt16 = Senser.Agc[10] * 100;
    PCSendBuff[2] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[3] = tempUInt16 & 0x00FF;

    //方位轴角
    tempUInt16 = Senser.Angle_AZ[10] * 65535 / 360.0f;
    PCSendBuff[4] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[5] = tempUInt16 & 0x00FF;

    //俯仰轴角
    tempUInt16 = (Senser.Angle_EL[10] + 180) * 65535 / 360.0f;
    PCSendBuff[6] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[7] = tempUInt16 & 0x00FF;

    //横滚轴角
    tempUInt16 = (Senser.Angle_ROLL[10] + 180) * 65535 / 360.0f;
    PCSendBuff[8] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[9] = tempUInt16 & 0x00FF;

    //极化轴角
    tempUInt16 = Senser.Angle_POL[10] * 65535 / 360.0f;
    PCSendBuff[10] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[11] = tempUInt16 & 0x00FF;

    //方位预置
    tempUInt16 = MotoCtr.AZPerset * 65535 / 360.0f;
    PCSendBuff[12] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[13] = tempUInt16 & 0x00FF;

    //俯仰预置
    tempUInt16 = (MotoCtr.ELPerset + 180) * 65535 / 360.0f;
    PCSendBuff[14] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[15] = tempUInt16 & 0x00FF;

    //横滚预置
    tempUInt16 = (MotoCtr.ROLLPerset + 180) * 65535 / 360.0f;
    PCSendBuff[16] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[17] = tempUInt16 & 0x00FF;

    //极化预置
    tempUInt16 = MotoCtr.POLPerset * 65535 / 360.0f;
    PCSendBuff[18] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[19] = tempUInt16 & 0x00FF;

    //方位角速度
    tempInt16 = Senser.SpeedAZ * 100;
    PCSendBuff[20] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[21] = tempInt16 & 0x00FF;

    //俯仰角速度
    tempInt16 = Senser.Speed_EL * 100;
    PCSendBuff[22] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[23] = tempInt16 & 0x00FF;

    //横滚角速度
    tempInt16 = Senser.SpeedROLL * 100;
    PCSendBuff[24] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[25] = tempInt16 & 0x00FF;

    //极化角速度
    tempInt16 = Senser.SpeedPOL * 100;
    PCSendBuff[26] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[27] = tempInt16 & 0x00FF;

    //卫星经度
    tempUInt16 = SatellitesInfo.longitude * 100;
    PCSendBuff[28] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[29] = tempUInt16 & 0x00FF;

    //极化方式
    tempUInt16 = SatellitesInfo.polerMode;
    PCSendBuff[30] = tempUInt16 & 0x00FF;

    //方位指向角
    tempUInt16 = MotoCtr.AZgPerset * 65535 / 360;
    PCSendBuff[31] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[32] = tempUInt16 & 0x00FF;

    //俯仰指向角
    tempUInt16 = MotoCtr.ELgPerset * 65535 / 360;
    PCSendBuff[33] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[34] = tempUInt16 & 0x00FF;

    //DVB载波频率
    tempUint32 = Senser.BeaconInfo.freq;
    PCSendBuff[35] = (tempUint32 & 0xFF000000) >> 24;
    PCSendBuff[36] = (tempUint32 & 0x00FF0000) >> 16;
    PCSendBuff[37] = (tempUint32 & 0x0000FF00) >> 8;
    PCSendBuff[38] = tempUint32 & 0x000000FF;

    //DVB符码率
    tempUInt16 = 0;
    PCSendBuff[39] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[40] = tempUInt16 & 0x00FF;

    //DVB丢帧数
    tempUInt16 = Senser.BeaconInfo.LostCount;
    PCSendBuff[41] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[42] = tempUInt16 & 0x00FF;

    //DVB错帧数
    tempUInt16 = Senser.BeaconInfo.ErrCount;
    PCSendBuff[43] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[44] = tempUInt16 & 0x00FF;

    //惯导状态
    tempUInt16 = Senser.MEMSInfo.Status;
    PCSendBuff[45] = tempUInt16 & 0x00FF;

    //Mems经度
    tempUInt16 = Senser.GPSX * 100;
    PCSendBuff[46] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[47] = tempUInt16 & 0x00FF;

    //Mems纬度
    tempUInt16 = Senser.GPSY * 100;
    PCSendBuff[48] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[49] = tempUInt16 & 0x00FF;

    //Mems高度
    tempUInt16 = Senser.GPSH * 100;
    PCSendBuff[50] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[51] = tempUInt16 & 0x00FF;

    //Mems航向角
    tempUInt16 = Senser.Head[10] * 100;
    PCSendBuff[52] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[53] = tempUInt16 & 0x00FF;

    //Mems纵摇角
    tempInt16 = Senser.Pitch[10] * 100;
    PCSendBuff[54] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[55] = tempInt16 & 0x00FF;

    //Mems横滚角
    tempInt16 = Senser.Roll[10] * 100;
    PCSendBuff[56] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[57] = tempInt16 & 0x00FF;

    //Mems丢帧数
    tempUInt16 = Senser.MEMSInfo.LostCount;
    PCSendBuff[58] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[59] = tempUInt16 & 0x00FF;

    //Mems错帧数
    tempUInt16 = Senser.MEMSInfo.ErrCount;
    PCSendBuff[60] = (tempUInt16 & 0xFF00) >> 8;
    PCSendBuff[61] = tempUInt16 & 0x00FF;

    //限位指示灯
    tempUInt16 = AZIO
        | (ELIO << 1)
        | (ROLLIO << 2)
        | (POLIO << 3)
        | (1 << 4);

    PCSendBuff[62] = tempUInt16 & 0x00FF;

    //方位角速度预置
    tempInt16 = MotoCtr.AZSpeed * 100;
    PCSendBuff[63] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[64] = tempInt16 & 0x00FF;

    //俯仰角速度预置
    tempInt16 = MotoCtr.ESpeed * 100;
    PCSendBuff[65] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[66] = tempInt16 & 0x00FF;

    //横滚角速度预置
    tempInt16 = MotoCtr.ROLLSpeed * 100;
    PCSendBuff[67] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[68] = tempInt16 & 0x00FF;

    //极化角速度预置
    tempInt16 = MotoCtr.POLSpeed * 100;
    PCSendBuff[69] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[70] = tempInt16 & 0x00FF;

    for (i = 0; i < 20; i++) {
        PCSendBuff[i + 71] = DVBRecvBuffTest[i];
    } //end PCSendBuff[90]

    //接收机模式
    PCSendBuff[91] = Senser.BeaconInfo.mode;

    //信标频率
    tempUint32 = Senser.BeaconInfo.freq;
    PCSendBuff[92] = (tempUint32 & 0xFF000000) >> 24;
    PCSendBuff[93] = (tempUint32 & 0x00FF0000) >> 16;
    PCSendBuff[94] = (tempUint32 & 0x0000FF00) >> 8;
    PCSendBuff[95] = tempUint32 & 0x000000FF;

    offset = 96;
    for (i = 0; i < 50; i++) {
        tempInt32 = DebugParams.Params[i] * 10000;
        PCSendBuff[i * 4 + offset] = (tempInt32 & 0xFF000000) >> 24;
        PCSendBuff[i * 4 + offset + 1] = (tempInt32 & 0x00FF0000) >> 16;
        PCSendBuff[i * 4 + offset + 2] = (tempInt32 & 0x0000FF00) >> 8;
        PCSendBuff[i * 4 + offset + 3] = tempInt32 & 0x000000FF;
    } //295

    memcpy(PCSendBuff + 296, ANT_NUM_CODE, sizeof(ANT_NUM_CODE));

    PCSendBuff[296 + sizeof(ANT_NUM_CODE)] = 0xFF;
    PCSendBuff[297 + sizeof(ANT_NUM_CODE)] = 0xFF;
    PCSendBuff[298 + sizeof(ANT_NUM_CODE)] = 0xFF;
    PCSendBuff[299 + sizeof(ANT_NUM_CODE)] = 0xFF;

    return 312;
}

unsigned char AZData(void)
{
    unsigned int tempUint16 = 0;
    short int tempInt16 = 0;
    //帧头
    PCSendBuff[0] = 0xEB;
    PCSendBuff[1] = GetCh(tAZAll, WorkMode);

    //角度预置
    tempUint16 = (MotoCtr.AZPerset / 360.0f) * 65535;
    PCSendBuff[2] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[3] = tempUint16 & 0x00FF;

    //角度实际
    tempUint16 = (Senser.Angle_AZ[10] / 360.0f) * 65535;
    PCSendBuff[4] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[5] = tempUint16 & 0x00FF;

    //角速度预置
    tempInt16 = MotoCtr.AZSpeed * 100;
    PCSendBuff[6] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[7] = tempInt16 & 0x00FF;

    //角速度实际
    tempInt16 = Senser.SpeedAZ * 100;
    PCSendBuff[8] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[9] = tempInt16 & 0x00FF;

    //PIDValue P
    tempInt16 = MotoCtr.PIDAZ.P * 100;
    PCSendBuff[10] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[11] = tempInt16 & 0x00FF;

    //PIDValue I
    tempInt16 = MotoCtr.PIDAZ.I * 100;
    PCSendBuff[12] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[13] = tempInt16 & 0x00FF;

    //PIDValue D
    tempInt16 = MotoCtr.PIDAZ.D * 100;
    PCSendBuff[14] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[15] = tempInt16 & 0x00FF;

    //PIDValue 前馈
    tempInt16 = MotoCtr.PIDAZ.K * 100;
    PCSendBuff[16] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[17] = tempInt16 & 0x00FF;

    //PIDValue 前馈值(限制前)
    tempInt16 = MotoCtr.PIDAZ.ValueBeforeL * 100;
    PCSendBuff[18] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[19] = tempInt16 & 0x00FF;

    //PIDValue
    tempInt16 = MotoCtr.PIDAZ.Value * 100;
    PCSendBuff[20] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[21] = tempInt16 & 0x00FF;

    //PIDValue 外推
    tempUint16 = (MotoCtr.PIDAZ.LookHead / 360.0f) * 65535;
    PCSendBuff[22] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[23] = tempUint16 & 0x00FF;

    //PIDValue 外推(限制前)
    tempUint16 = (MotoCtr.PIDAZ.LookHeadBeforeL / 360.0f) * 65535;
    PCSendBuff[24] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[25] = tempUint16 & 0x00FF;

    //AGC
    tempUint16 = (Senser.Agc[10] / 360.0f) * 65535;
    PCSendBuff[26] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[27] = tempUint16 & 0x00FF;

    //MEMS X轴角速度
    tempInt16 = Senser.MemsX * 100;
    PCSendBuff[28] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[29] = tempInt16 & 0x00FF;

    //MEMS Y轴角速度
    tempInt16 = Senser.MemsY * 100;
    PCSendBuff[30] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[31] = tempInt16 & 0x00FF;

    //MEMS Z轴角速度
    tempInt16 = Senser.MemsZ * 100;
    PCSendBuff[32] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[33] = tempInt16 & 0x00FF;

    //航向
    tempUint16 = Senser.Head[10] * 100;
    PCSendBuff[34] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[35] = tempUint16 & 0x00FF;

    //纵摇
    tempInt16 = Senser.Pitch[10] * 100;
    PCSendBuff[36] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[37] = tempInt16 & 0x00FF;

    //横滚
    tempInt16 = Senser.Roll[10] * 100;
    PCSendBuff[38] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[39] = tempInt16 & 0x00FF;

    //方位空间预置
    tempUint16 = Senser.AZEncoder.Count;
    PCSendBuff[40] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[41] = tempUint16 & 0x00FF;

    //方位空间实际
    //	tempUint16 = Senser.AZEncoder.PeriodCount;
    tempUint16 = Senser.AZEncoder.PeriodCountTest;
    PCSendBuff[42] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[43] = tempUint16 & 0x00FF;

    //PWM输出频率
    PCSendBuff[44] = (MotoCtr.PWMFre.el & 0xFF000000) >> 24;
    PCSendBuff[45] = (MotoCtr.PWMFre.el & 0x00FF0000) >> 16;
    PCSendBuff[46] = (MotoCtr.PWMFre.el & 0x0000FF00) >> 8;
    PCSendBuff[47] = MotoCtr.PWMFre.el & 0x00000000FF;

    //PWM定时器周期寄存器的值
    PCSendBuff[48] = (MotoCtr.TIMPeriod.el & 0xFF000000) >> 24;
    PCSendBuff[49] = (MotoCtr.TIMPeriod.el & 0x00FF0000) >> 16;
    PCSendBuff[50] = (MotoCtr.TIMPeriod.el & 0x0000FF00) >> 8;
    PCSendBuff[51] = MotoCtr.TIMPeriod.el & 0x00000000FF;

    return 52;
}

unsigned char ELData(void)
{
    unsigned int tempUint16 = 0;
    short int tempInt16 = 0;

    //帧头
    PCSendBuff[0] = 0xEB;
    PCSendBuff[1] = GetCh(tELAll, WorkMode);

    //角度预置
    tempUint16 = ((MotoCtr.ELPerset + 180) / 360.0f) * 65535;
    PCSendBuff[2] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[3] = tempUint16 & 0x00FF;

    //角度实际
    tempUint16 = ((Senser.Angle_EL[10] + 180) / 360.0f) * 65535;
    PCSendBuff[4] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[5] = tempUint16 & 0x00FF;

    //角速度预置
    tempInt16 = MotoCtr.ESpeed * 100;
    PCSendBuff[6] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[7] = tempInt16 & 0x00FF;

    //角速度实际
    tempInt16 = Senser.Speed_EL * 100;
    PCSendBuff[8] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[9] = tempInt16 & 0x00FF;

    //PIDValue P
    tempInt16 = MotoCtr.PIDEL.P * 100;
    PCSendBuff[10] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[11] = tempInt16 & 0x00FF;

    //PIDValue I
    tempInt16 = MotoCtr.PIDEL.I * 100;
    PCSendBuff[12] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[13] = tempInt16 & 0x00FF;

    //PIDValue D
    tempInt16 = MotoCtr.PIDEL.D * 100;
    PCSendBuff[14] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[15] = tempInt16 & 0x00FF;

    //PIDValue 前馈
    tempInt16 = MotoCtr.PIDEL.K * 100;
    PCSendBuff[16] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[17] = tempInt16 & 0x00FF;

    //PIDValue 前馈值(限制前)
    tempInt16 = MotoCtr.PIDEL.ValueBeforeL * 100;
    PCSendBuff[18] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[19] = tempInt16 & 0x00FF;

    //PIDValue
    tempInt16 = MotoCtr.PIDEL.Value * 100;
    PCSendBuff[20] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[21] = tempInt16 & 0x00FF;

    //PIDValue 外推
    tempUint16 = ((MotoCtr.PIDEL.LookHead + 180) / 360.0f) * 65535;
    PCSendBuff[22] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[23] = tempUint16 & 0x00FF;

    //PIDValue 外推(限制前)
    tempUint16 = ((MotoCtr.PIDEL.LookHeadBeforeL + 180) / 360.0f) * 65535;
    PCSendBuff[24] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[25] = tempUint16 & 0x00FF;

    //AGC
    tempUint16 = (Senser.Agc[10] / 360.0f) * 65535;
    PCSendBuff[26] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[27] = tempUint16 & 0x00FF;

    //MEMS X轴角速度
    tempInt16 = Senser.MemsX * 100;
    PCSendBuff[28] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[29] = tempInt16 & 0x00FF;

    //MEMS Y轴角速度
    tempInt16 = Senser.MemsY * 100;
    PCSendBuff[30] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[31] = tempInt16 & 0x00FF;

    //MEMS Z轴角速度
    tempInt16 = Senser.MemsZ * 100;
    PCSendBuff[32] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[33] = tempInt16 & 0x00FF;

    //航向
    tempUint16 = Senser.Head[10] * 100;
    PCSendBuff[34] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[35] = tempUint16 & 0x00FF;

    //纵摇
    tempInt16 = Senser.Pitch[10] * 100;
    PCSendBuff[36] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[37] = tempInt16 & 0x00FF;

    //横滚
    tempInt16 = Senser.Roll[10] * 100;
    PCSendBuff[38] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[39] = tempInt16 & 0x00FF;

    //PWM输出频率
    PCSendBuff[40] = (MotoCtr.PWMFre.el & 0xFF000000) >> 24;
    PCSendBuff[41] = (MotoCtr.PWMFre.el & 0x00FF0000) >> 16;
    PCSendBuff[42] = (MotoCtr.PWMFre.el & 0x0000FF00) >> 8;
    PCSendBuff[43] = MotoCtr.PWMFre.el & 0x00000000FF;

    //PWM定时器周期寄存器的值
    PCSendBuff[44] = (MotoCtr.TIMPeriod.el & 0xFF000000) >> 24;
    PCSendBuff[45] = (MotoCtr.TIMPeriod.el & 0x00FF0000) >> 16;
    PCSendBuff[46] = (MotoCtr.TIMPeriod.el & 0x0000FF00) >> 8;
    PCSendBuff[47] = MotoCtr.TIMPeriod.el & 0x00000000FF;

    return 48;
}

unsigned char ROLLData(void)
{
    unsigned int tempUint16 = 0;
    short int tempInt16 = 0;
    //帧头
    PCSendBuff[0] = 0xEB;
    PCSendBuff[1] = GetCh(tROLLAll, WorkMode);

    //角度预置
    tempUint16 = ((MotoCtr.ROLLPerset + 180) / 360.0f) * 65535;
    PCSendBuff[2] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[3] = tempUint16 & 0x00FF;

    //角度实际
    tempUint16 = ((Senser.Angle_ROLL[10] + 180) / 360.0f) * 65535;
    PCSendBuff[4] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[5] = tempUint16 & 0x00FF;

    //角速度预置
    tempInt16 = MotoCtr.ROLLSpeed * 100;
    PCSendBuff[6] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[7] = tempInt16 & 0x00FF;

    //角速度实际
    tempInt16 = Senser.SpeedROLL * 100;
    PCSendBuff[8] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[9] = tempInt16 & 0x00FF;

    //PIDValue P
    tempInt16 = MotoCtr.PIDROLL.P * 100;
    PCSendBuff[10] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[11] = tempInt16 & 0x00FF;

    //PIDValue I
    tempInt16 = MotoCtr.PIDROLL.I * 100;
    PCSendBuff[12] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[13] = tempInt16 & 0x00FF;

    //PIDValue D
    tempInt16 = MotoCtr.PIDROLL.D * 100;
    PCSendBuff[14] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[15] = tempInt16 & 0x00FF;

    //PIDValue 前馈
    tempInt16 = MotoCtr.PIDROLL.K * 100;
    PCSendBuff[16] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[17] = tempInt16 & 0x00FF;

    //PIDValue 前馈值(限制前)
    tempInt16 = MotoCtr.PIDROLL.ValueBeforeL * 100;
    PCSendBuff[18] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[19] = tempInt16 & 0x00FF;

    //PIDValue
    tempInt16 = MotoCtr.PIDROLL.Value * 100;
    PCSendBuff[20] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[21] = tempInt16 & 0x00FF;

    //PIDValue 外推
    tempUint16 = ((MotoCtr.PIDROLL.LookHead + 180) / 360.0f) * 65535;
    PCSendBuff[22] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[23] = tempUint16 & 0x00FF;

    //PIDValue 外推(限制前)
    tempUint16 = ((MotoCtr.PIDROLL.LookHeadBeforeL + 180) / 360.0f) * 65535;
    PCSendBuff[24] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[25] = tempUint16 & 0x00FF;

    //AGC
    tempUint16 = (Senser.Agc[10] / 360.0f) * 65535;
    PCSendBuff[26] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[27] = tempUint16 & 0x00FF;

    //MEMS X轴角速度
    tempInt16 = Senser.MemsX * 100;
    PCSendBuff[28] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[29] = tempInt16 & 0x00FF;

    //MEMS Y轴角速度
    tempInt16 = Senser.MemsY * 100;
    PCSendBuff[30] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[31] = tempInt16 & 0x00FF;

    //MEMS Z轴角速度
    tempInt16 = Senser.MemsZ * 100;
    PCSendBuff[32] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[33] = tempInt16 & 0x00FF;

    //航向
    tempUint16 = Senser.Head[10] * 100;
    PCSendBuff[34] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[35] = tempUint16 & 0x00FF;

    //纵摇
    tempInt16 = Senser.Pitch[10] * 100;
    PCSendBuff[36] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[37] = tempInt16 & 0x00FF;

    //横滚
    tempInt16 = Senser.Roll[10] * 100;
    PCSendBuff[38] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[39] = tempInt16 & 0x00FF;

    return 40;
}

unsigned char POLData(void)
{
    unsigned int tempUint16 = 0;
    short int tempInt16 = 0;
    //帧头
    PCSendBuff[0] = 0xEB;
    PCSendBuff[1] = GetCh(tPOLAll, WorkMode);

    //角度预置
    tempUint16 = (MotoCtr.POLPerset / 360.0f) * 65535;
    PCSendBuff[2] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[3] = tempUint16 & 0x00FF;

    //角度实际
    tempUint16 = Senser.Angle_POL[10] / 360.0f * 65535;
    PCSendBuff[4] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[5] = tempUint16 & 0x00FF;

    //角速度预置
    tempInt16 = MotoCtr.POLSpeed * 100;
    PCSendBuff[6] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[7] = tempInt16 & 0x00FF;

    //角速度实际
    tempInt16 = Senser.SpeedPOL * 100;
    PCSendBuff[8] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[9] = tempInt16 & 0x00FF;

    //PIDValue P
    tempInt16 = MotoCtr.PIDPOL.P * 100;
    PCSendBuff[10] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[11] = tempInt16 & 0x00FF;

    //PIDValue I
    tempInt16 = MotoCtr.PIDPOL.I * 100;
    PCSendBuff[12] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[13] = tempInt16 & 0x00FF;

    //PIDValue D
    tempInt16 = MotoCtr.PIDPOL.D * 100;
    PCSendBuff[14] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[15] = tempInt16 & 0x00FF;

    //PIDValue 前馈
    tempInt16 = MotoCtr.PIDPOL.K * 100;
    PCSendBuff[16] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[17] = tempInt16 & 0x00FF;

    //PIDValue 前馈值(限制前)
    tempInt16 = MotoCtr.PIDPOL.ValueBeforeL * 100;
    PCSendBuff[18] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[19] = tempInt16 & 0x00FF;

    //PIDValue
    tempInt16 = MotoCtr.PIDPOL.Value * 100;
    PCSendBuff[20] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[21] = tempInt16 & 0x00FF;

    //PIDValue 外推
    tempUint16 = ((MotoCtr.PIDPOL.LookHead + 180) / 360.0f) * 65535;
    PCSendBuff[22] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[23] = tempUint16 & 0x00FF;

    //PIDValue 外推(限制前)
    tempUint16 = ((MotoCtr.PIDPOL.LookHeadBeforeL + 180) / 360.0f) * 65535;
    PCSendBuff[24] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[25] = tempUint16 & 0x00FF;

    //AGC
    tempUint16 = (Senser.Agc[10] / 360.0f) * 65535;
    PCSendBuff[26] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[27] = tempUint16 & 0x00FF;

    //MEMS X轴角速度
    tempInt16 = Senser.MemsX * 100;
    PCSendBuff[28] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[29] = tempInt16 & 0x00FF;

    //MEMS Y轴角速度
    tempInt16 = Senser.MemsY * 100;
    PCSendBuff[30] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[31] = tempInt16 & 0x00FF;

    //MEMS Z轴角速度
    tempInt16 = Senser.MemsZ * 100;
    PCSendBuff[32] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[33] = tempInt16 & 0x00FF;

    //航向
    tempUint16 = Senser.Head[10] * 100;
    PCSendBuff[34] = (tempUint16 & 0xFF00) >> 8;
    PCSendBuff[35] = tempUint16 & 0x00FF;

    //纵摇
    tempInt16 = Senser.Pitch[10] * 100;
    PCSendBuff[36] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[37] = tempInt16 & 0x00FF;

    //横滚
    tempInt16 = Senser.Roll[10] * 100;
    PCSendBuff[38] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[39] = tempInt16 & 0x00FF;

    return 40;
}

unsigned char TrackMsg1(void)
{
    int temp = 0;
    unsigned int tempUint = 0;
    int tempInt16 = 0;
    unsigned short int tempUInt16 = 0;
    //帧头
    PCSendBuff[0] = 0xEB;
    PCSendBuff[1] = GetCh(tTrack1, WorkMode);

    //方位指向预置
    tempUint = (MotoCtr.AZgPerset / 360.0f) * 65535;
    PCSendBuff[2] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[3] = tempUint & 0x00FF;

    //方位指向实际
    tempUint = (Senser.AZg[10] / 360.0f) * 65535;
    PCSendBuff[4] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[5] = tempUint & 0x00FF;

    //俯仰指向预置
    tempUint = (MotoCtr.ELgPerset / 360.0f) * 65535;
    PCSendBuff[6] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[7] = tempUint & 0x00FF;

    //俯仰指向实际
    tempUint = (Senser.ELg[10] / 360.0f) * 65535;
    PCSendBuff[8] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[9] = tempUint & 0x00FF;

    //AGC
    extern float HeadCheckStatus;
    temp = Senser.Agc[10] * 1000;
    // temp = HeadCheckStatus * 1000;
    PCSendBuff[10] = (temp & 0xFF00) >> 8;
    PCSendBuff[11] = temp & 0x00FF;

    //方位预置
    tempUint = (MotoCtr.AZPerset / 360.0f) * 65535;
    PCSendBuff[12] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[13] = tempUint & 0x00FF;

    //方位实际
    tempUint = (Senser.Angle_AZ[10] / 360.0f) * 65535;
    PCSendBuff[14] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[15] = tempUint & 0x00FF;

    //方位角速度预置
    temp = MotoCtr.AZSpeed * 100;
    PCSendBuff[16] = (temp & 0xFF00) >> 8;
    PCSendBuff[17] = temp & 0x00FF;

    //方位角速度实际
    temp = AZEncoderSpeed();
    PCSendBuff[18] = (temp & 0xFF00) >> 8;
    PCSendBuff[19] = temp & 0x00FF;

    //俯仰预置
    tempUint = ((MotoCtr.ELPerset + 180) / 360.0f) * 65535;
    PCSendBuff[20] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[21] = tempUint & 0x00FF;

    //俯仰实际
    tempUint = ((Senser.Angle_EL[10] + 180) / 360.0f) * 65535;
    PCSendBuff[22] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[23] = tempUint & 0x00FF;

    //俯仰角速度预置
    temp = MotoCtr.ESpeed * 100;
    PCSendBuff[24] = (temp & 0xFF00) >> 8;
    PCSendBuff[25] = temp & 0x00FF;

    //俯仰角速度实际
    temp = ELEncoderSpeed();
    PCSendBuff[26] = (temp & 0xFF00) >> 8;
    PCSendBuff[27] = temp & 0x00FF;

    //交叉角预置
    temp = ((MotoCtr.ROLLPerset + 180) / 360.0f) * 65535;
    PCSendBuff[28] = (temp & 0xFF00) >> 8;
    PCSendBuff[29] = temp & 0x00FF;

    //交叉角实际
    temp = ((Senser.Angle_ROLL[10] + 180) / 360.0f) * 65535;
    PCSendBuff[30] = (temp & 0xFF00) >> 8;
    PCSendBuff[31] = temp & 0x00FF;

    //交叉角速度预置
    tempInt16 = MotoCtr.ROLLSpeed * 100;
    PCSendBuff[32] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[33] = tempInt16 & 0x00FF;

    //交叉角速度
    tempInt16 = Senser.SpeedROLL * 100;
    PCSendBuff[34] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[35] = tempInt16 & 0x00FF;

    //极化角预置
    temp = ((MotoCtr.POLPerset + 180) / 360.0f) * 65535;
    PCSendBuff[36] = (temp & 0xFF00) >> 8;
    PCSendBuff[37] = temp & 0x00FF;

    //极化角实际
    temp = ((Senser.Angle_POL[10] + 180) / 360.0f) * 65535;
    PCSendBuff[38] = (temp & 0xFF00) >> 8;
    PCSendBuff[39] = temp & 0x00FF;

    //极化角速度预置
    tempInt16 = MotoCtr.POLSpeed * 100;
    PCSendBuff[40] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[41] = tempInt16 & 0x00FF;

    //极化角速度
    tempInt16 = Senser.SpeedPOL * 100;
    PCSendBuff[42] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[43] = tempInt16 & 0x00FF;

    //航向
    tempUint = (Senser.HeadNew / 360.0f) * 65535;
    PCSendBuff[44] = (tempUint & 0xFF00) >> 8;
    PCSendBuff[45] = tempUint & 0x00FF;

    //航向总修正量
    temp = (Senser.HeadOffset * 65535 / 360.0f);
    PCSendBuff[46] = (temp & 0xFF00) >> 8;
    PCSendBuff[47] = temp & 0x00FF;

    //航向单次圆扫修正量
    temp = (MotoCtr.angleAdjustAZ / 360.0f) * 65535;
    PCSendBuff[48] = (temp & 0xFF00) >> 8;
    PCSendBuff[49] = temp & 0x00FF;

    //纵摇
    temp = (Senser.Pitch[10] / 360.0f) * 65535;
    PCSendBuff[50] = (temp & 0xFF00) >> 8;
    PCSendBuff[51] = temp & 0x00FF;

    //横滚
    temp = (Senser.Roll[10] / 360.0f) * 65535;
    PCSendBuff[52] = (temp & 0xFF00) >> 8;
    PCSendBuff[53] = temp & 0x00FF;

    //X轴角速率
    tempInt16 = Senser.MemsX * 100;
    PCSendBuff[54] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[55] = tempInt16 & 0x00FF;

    //Y轴角速率
    tempInt16 = Senser.MemsY * 100;
    PCSendBuff[56] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[57] = tempInt16 & 0x00FF;

    //Z轴角速率
    tempInt16 = Senser.MemsZ * 100;
    PCSendBuff[58] = (tempInt16 & 0xFF00) >> 8;
    PCSendBuff[59] = tempInt16 & 0x00FF;

    //AGCMax
    temp = Senser.AgcMAx.Agc * 100;
    PCSendBuff[60] = (temp & 0xFF00) >> 8;
    PCSendBuff[61] = temp & 0x00FF;

    //Mems丢帧数
    tempUInt16 = Senser.MEMSInfo.LostCount;
    PCSendBuff[62] = tempUInt16 & 0x00FF;

    //Mems错帧数
    tempUInt16 = Senser.MEMSInfo.ErrCount;
    PCSendBuff[63] = tempUInt16 & 0x00FF;

    return 64;
}

// 通过DMA方式向上位机发送报文
void SendMsgDMA(uint16_t timeClk)
{
    unsigned char i = 0;
    unsigned short int temp = 0;

    const int time_1000ms = 1 * timeClk;
    const int time_500ms = 0.5 * timeClk;
    const int time_100ms = 0.1 * timeClk;
    const int time_10ms = 0.01 * timeClk;
    const int time_track_period = TRACKMSGPERIOD / 1000.0f * timeClk;

    msg.PeriodCount++;
    if (msg.PeriodCount >= msg.PeriodsPerMsg) {
        switch (msg.Mode) {
        case 2: //ACU协议
            //TODO
        case 3: //全数据
            msg.BytesCount = AllData(); //报文长度
            msg.PeriodsPerMsg = time_1000ms; //报文发送周期
            break;
        case 4: //方位数据
            msg.BytesCount = AZData();
            msg.PeriodsPerMsg = time_10ms;
            break;
        case 5: //俯仰数据
            msg.BytesCount = ELData();
            msg.PeriodsPerMsg = time_10ms;
            break;
        case 6: //横滚数据
            msg.BytesCount = 40;
            msg.PeriodsPerMsg = time_10ms;
            ROLLData();
            break;
        case 7: //极化数据
            msg.BytesCount = 40;
            msg.PeriodsPerMsg = time_10ms;
            POLData();
            break;
        case 9: //跟踪监视1
            msg.BytesCount = 64;
            msg.PeriodsPerMsg = time_track_period;
            TrackMsg1();
            break;
        default: //全数据
            msg.BytesCount = AllData(); //报文长度
            msg.PeriodsPerMsg = time_500ms; //报文发送周期
            break;
        }

        temp = msg.BytesCount / 4;

        uint16_t cs = 0;
        for (uint32_t i = 0; i < msg.BytesCount; i++) {
            cs |= PCSendBuff[i];
        }

        PCSendBuff[msg.BytesCount] = (cs >> 8) & 0xFF;
        PCSendBuff[msg.BytesCount + 1] = cs & 0xFF;

        //DMA方式发送数据
        SendMsgUdp(PCSendBuff, msg.BytesCount + 2);

        msg.PeriodCount = 0;
    }
}

// 计算得到方位角速率
int AZEncoderSpeed()
{
    int tempInt = 0;
    float tempspeed = 0;
    tempspeed = Senser.Angle_AZ[10] - Senser.Angle_AZ[9];
    if (tempspeed > 200) {
        tempspeed -= 360;
    } else if (tempspeed < -200) {
        tempspeed += 360;
    }
    Senser.SpeedAZ = tempspeed * _CTRCLK;
    tempInt = Senser.SpeedAZ * 100;
    return tempInt;
}

// 计算得到俯仰角速率
int ELEncoderSpeed()
{
    int tempInt = 0;
    Senser.Speed_EL = (Senser.Angle_EL[10] - Senser.Angle_EL[9]) * _CTRCLK;
    tempInt = Senser.Speed_EL * 100;
    return tempInt;
}
