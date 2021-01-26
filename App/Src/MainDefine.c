#include "MainDefine.h"

#include "stdio.h"

#include "ParametersDefine.h"
#include "main.h"

#define AZPositionCtrInc 1

#define ROLLIO (/*0x01 ^ */ HAL_GPIO_ReadPin(RX_LIM_GPIO_Port, RX_LIM_Pin))
#define ELIO (0x01 ^ HAL_GPIO_ReadPin(EL_LIM_GPIO_Port, EL_LIM_Pin))
#define AZIO (0x01 ^ HAL_GPIO_ReadPin(AZ_LIM_GPIO_Port, AZ_LIM_Pin))
#define POLIO (/*0x01 ^ */ HAL_GPIO_ReadPin(TX_LIM_GPIO_Port, TX_LIM_Pin))

/// <summary>
/// 航向校准过程中，计算俯仰角
/// </summary>
/// <param name="ELg">俯仰指向角</param>
/// <param name="Pitch">纵摇</param>
/// <param name="Roll">横滚</param>
/// <param name="AZb">方位轴角</param>
/// <param name="ELb">俯仰轴角</param>
void GetElb_HeadCheck(float ELg, float Pitch, float Roll, float AZb, float* ELb);

//计算得到当前航向
void GetHead_HeadCheck(float AZg, float ELg, float AZb, float Pitch, float Roll, float* Head);

/// 工作指示灯闪烁，周期1s
void PilotLampControl()
{
    int _time_500ms = 0.5 * _CTRCLK;
    PeriodPilotLamp++;
    if (PeriodPilotLamp > _time_500ms) {
        PeriodPilotLamp = 0;
        if (Senser.MEMSInfo.BugTime < 100) {
            // LED_BLINK;
        }
    }
}

// 用于上电延时计数器
void WaitDriver()
{
    __IO uint16_t delayICount = 0, delayJCount = 0;
    //驱动器上电后需要一定的延时才能正常工作
    for (delayICount = 0; delayICount < 0x7FF; delayICount++) {
        for (delayJCount = 0; delayJCount < 1000; delayJCount++) {
        }
    }
}

//待机模式
void StandBy(void)
{
    // 待机减速时间
    const int StandByTime = 2 * _CTRCLK;
    if (StandbyTimeCount < StandByTime) {
        StandbyTimeCount++;
        MotoCtr.AZSpeed = MotoCtr.AZSpeed * 0.9f;
        MotoCtr.ESpeed = MotoCtr.ESpeed * 0.9f;
        MotoCtr.ROLLSpeed = MotoCtr.ROLLSpeed * 0.9f;
        MotoCtr.POLSpeed = MotoCtr.POLSpeed * 0.9f;
    } else {
        StandbyTimeCount = StandByTime;
        MotoCtr.AZSpeed = 0;
        MotoCtr.ESpeed = 0;
        MotoCtr.ROLLSpeed = 0;
        MotoCtr.POLSpeed = 0;
    }
}

// 寻零
void ZeroCheck()
{
    if (AZFindZeroEndFlag == 0) {
        WorkMode = Mode_Find_Zero;
        AZFindZero();
    }
    if (ELFindZeroEndFlag == 0) {
        WorkMode = Mode_Find_Zero;
        ELFindZero();
    }
    if (ROLLFindZeroEndFlag == 0) {
        WorkMode = Mode_Find_Zero;
        ROLLFindZero();
    }
    if (POLFindZeroEndFlag == 0) {
        WorkMode = Mode_Find_Zero;
        POLFindZero();
    }
}

// 电机使能
void MotoInit()
{
    WaitDriver();
    MotoEnablePOL();
    WaitDriver();
    MotoEnableROLL();
    WaitDriver();
    MotoEnableAZ();
    WaitDriver();
    MotoEnableEL();

    WaitDriver();

    MotoEnableAZ();
    MotoEnableEL();
    MotoEnableROLL();
    MotoEnablePOL();

    SpeedCtrl();
}

// 方位寻零
void AZFindZero(void)
{
    float Speed1 = 5;
    float Speed2 = 20;
    float Speed3 = -5;

    unsigned short int time_2sec = 2 * _CTRCLK;
    unsigned short int time_500ms = 0.5 * _CTRCLK;

    switch (AZFindZeroParams.State) {
    case 0:
        if (AZIO == 0x01) {
            //上电时停留于光电开关位置
            AZFindZeroParams.State = 1;
            AZFindZeroParams.PeriodCount = 0;
        } else {
            AZFindZeroParams.State = 4;
            AZFindZeroParams.PeriodCount = 0;
        }
        break;
    case 1:
        //以小速度逆时针转
        MotoCtr.AZSpeed = Speed3;
        if (AZIO == 0x00) {
            //离开光电开关位置
            AZFindZeroParams.State = 2;
            AZFindZeroParams.PeriodCount = 0;
        }
        break;
    case 2:
        //以小速度逆时针继续转动2s
        MotoCtr.AZSpeed = Speed3;
        AZFindZeroParams.PeriodCount++;
        if (AZFindZeroParams.PeriodCount > (int)time_2sec) {
            //状态跳
            AZFindZeroParams.State = 3;
            AZFindZeroParams.PeriodCount = 0;
        }
        break;
    case 3:
        //以小速度顺时针转
        MotoCtr.AZSpeed = Speed1;
        if (AZIO == 0x01) {
            //寻零结束
            AZFindZeroParams.State = 7;
            Senser.AZEncoder.Count = AZ_EC_TIM->CNT;
            Senser.AngleOffset_AZ = Senser.AZEncoder.Count + Senser.AZEncoder.PeriodCount * 40000;

            AZFindZeroParams.PeriodCount = 0;
        }
        break;
    case 4:
        MotoCtr.AZSpeed = Speed1;
        if (AZIO == 0x01) {
            //低速到达开关位置， 寻零结束
            AZFindZeroParams.State = 1;
            AZFindZeroParams.PeriodCount = 0;
        }

        if (AZFindZeroParams.PeriodCount++ > time_2sec) {
            //低速转动超过10°
            AZFindZeroParams.State = 6;
            AZFindZeroParams.PeriodCount = 0;
        }
        break;
    case 6:
        //向90方向高速转动
        MotoCtr.AZSpeed = Speed2;
        if (AZIO == 0x01) {
            AZFindZeroParams.State = 1;
        }
        break;
    case 7:
        MotoCtr.AZSpeed = 0;
        AZFindZeroParams.PeriodCount = 0;
        AZFindZeroEndFlag = 1;
        AZFindZeroParams.State = 999;
        WorkMode = Mode_Stand_By;
        break;
    default:
        break;
    }
}

//俯仰寻零
void ELFindZero(void)
{
    float Speed1 = -5;
    float Speed2 = -10;
    float Speed3 = 5;

    unsigned short int time_2sec = 2 * _CTRCLK;
    unsigned short int time_500ms = 0.5 * _CTRCLK;

    switch (ELFindZeroParams.State) {
    case 0:
        if (ELIO == 0x01) {
            //上电时俯仰停留于光电开关位置
            ELFindZeroParams.State = 1;
            ELFindZeroParams.PeriodCount = 0;
        } else {
            ELFindZeroParams.State = 4;
            ELFindZeroParams.PeriodCount = 0;
        }
        break;
    case 1:
        //以小速度向0方向转动
        MotoCtr.ESpeed = Speed3;
        if (ELIO == 0x00) {
            // 俯仰光电开关位置
            ELFindZeroParams.State = 2;
            ELFindZeroParams.PeriodCount = 0;
        }
        break;
    case 2:
        //以小速度向0方向继续转到2s
        MotoCtr.ESpeed = Speed3;
        ELFindZeroParams.PeriodCount++;
        if (ELFindZeroParams.PeriodCount > (int)time_2sec) {
            //状态跳
            ELFindZeroParams.State = 3;
            ELFindZeroParams.PeriodCount = 0;
        }
        break;
    case 3:
        //以小速度向90方向转动
        MotoCtr.ESpeed = Speed1;
        if (ELIO == 0x01) {
            //寻零结束
            //ELFindZeroEndFlag = 1;
            ELFindZeroParams.State = 7;
            Senser.ELEncoder.Count = EL_EC_TIM->CNT;
            Senser.AngleOffset_EL = Senser.ELEncoder.Count + Senser.ELEncoder.PeriodCount * 40000;

            ELFindZeroParams.PeriodCount = 0;
        }
        break;
    case 4:
        //向90方向低速转动
        //出现以下两种情况
        //低速直接到达开关位置
        //低速转动超过10°
        MotoCtr.ESpeed = Speed1;
        if (ELIO == 0x01) {
            //低速到达开关位置，寻零结束
            //ELFindZeroEndFlag = 1;
            ELFindZeroParams.State = 7;
            Senser.ELEncoder.Count = EL_EC_TIM->CNT;
            Senser.AngleOffset_EL = Senser.ELEncoder.Count + Senser.ELEncoder.PeriodCount * 40000;

            ELFindZeroParams.PeriodCount = 0;
        }

        if (ELFindZeroParams.PeriodCount++ > time_2sec) {
            //低速转动超过10°
            ELFindZeroParams.State = 6;
            ELFindZeroParams.PeriodCount = 0;
        }
        break;
    case 6:
        //向90方向高速转动
        MotoCtr.ESpeed = Speed2;
        if (ELIO == 0x01) {
            ELFindZeroParams.State = 1;
        }
        break;
    case 7:
        // 避免电机在极限位置受力，控制电机向0方向走一定位置
        ELFindZeroParams.PeriodCount++;
        if (ELFindZeroParams.PeriodCount < time_500ms)
            MotoCtr.ESpeed = (-Speed2) * ELFindZeroParams.PeriodCount / time_500ms;
        else if (ELFindZeroParams.PeriodCount < time_500ms + time_2sec)
            MotoCtr.ESpeed = (-Speed2);
        else if (ELFindZeroParams.PeriodCount < time_500ms + time_2sec + time_500ms)
            MotoCtr.ESpeed = (-Speed2) * (time_500ms - (ELFindZeroParams.PeriodCount - time_500ms - time_2sec)) / time_500ms;
        else {
            MotoCtr.ESpeed = 0;
            ELFindZeroParams.PeriodCount = 0;
            ELFindZeroEndFlag = 1;
            ELFindZeroParams.State = 999;
            WorkMode = Mode_Stand_By;
        }
        break;
    default:
        break;
    }
}

// 横滚寻零
void ROLLFindZero(void)
{
    float Speed1 = -10;
    float Speed2 = -5;
    float Speed3 = 5;
    float Speed4 = 10;

    int time_2sec = 2 * _CTRCLK;
    int time_500ms = 0.5 * _CTRCLK;

    switch (ROLLFindZeroParams.State) {
    case 0:
        if (ROLLIO == 0x01) {
            //上电时停留于开关位置
            ROLLFindZeroParams.State = 1;
            ROLLFindZeroParams.PeriodCount = 0;
        } else {
            ROLLFindZeroParams.State = 4;
            ROLLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 1:
        //低速向右方向转
        MotoCtr.ROLLSpeed = Speed3;
        if (ROLLIO == 0x00) {
            //离开开关位置
            ROLLFindZeroParams.State = 2;
            ROLLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 2:
        //低速向右方向继2s
        MotoCtr.ROLLSpeed = Speed3;
        if (ROLLFindZeroParams.PeriodCount++ > time_2sec) {
            ROLLFindZeroParams.State = 3;
            ROLLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 3:
        //低速向左方向转动直至开关位置
        MotoCtr.ROLLSpeed = Speed2;
        if (ROLLIO == 0x01) {
            //寻零结束
            ROLLFindZeroParams.State = 7;
            Senser.ROLLEncoder.Count = RL_EC_TIM->CNT;
            Senser.AngleOffset_ROLL = Senser.ROLLEncoder.Count + Senser.ROLLEncoder.PeriodCount * 40000;

            ROLLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 4:
        //低速向右方向转2s
        MotoCtr.ROLLSpeed = Speed2;
        //到达开关位置
        if (ROLLIO == 0x01) {
            ROLLFindZeroParams.State = 1;
            ROLLFindZeroParams.PeriodCount = 0;
        }

        //动时间超2s
        if (ROLLFindZeroParams.PeriodCount++ > time_2sec) {
            ROLLFindZeroParams.State = 5;
            ROLLFindZeroParams.PeriodCount = 0;
        }

        break;
    case 5:
        //高速向右方向转动直至到开关位置或者硬限位
        MotoCtr.ROLLSpeed = Speed1;

        //到达开
        if (ROLLIO == 0x01) {
            ROLLFindZeroParams.State = 1;
            ROLLFindZeroParams.PeriodCount = 0;
        }

        //到达限位
        //		if(fabs(Senser.Angle_ROLL[10])- fabs(Senser.Angle_ROLL[9]) < -0.1)
        //		{
        //			ROLLFindZeroParams.State = 6;
        //			ROLLFindZeroParams.PeriodCount = 0;
        //		}

        //通过角速度方向进行硬限位判断
        if (Senser.SpeedROLL / Speed1 < 0) {
            ROLLFindZeroParams.State = 6;
            ROLLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 6:
        //高速向左转动至开关位置
        MotoCtr.ROLLSpeed = Speed4;
        //到达开
        if (ROLLIO == 0x01) {
            ROLLFindZeroParams.State = 1;
            ROLLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 7:
        // 避免电机在极限位置受力，控制电机向0方向走一定位置
        ROLLFindZeroParams.PeriodCount++;
        if (ROLLFindZeroParams.PeriodCount < time_500ms)
            MotoCtr.ROLLSpeed = (Speed3 + Speed4) * ROLLFindZeroParams.PeriodCount / time_500ms;
        else if (ROLLFindZeroParams.PeriodCount < time_500ms + time_2sec)
            MotoCtr.ROLLSpeed = (Speed3 + Speed4);
        else if (ROLLFindZeroParams.PeriodCount < time_500ms + time_2sec + time_500ms)
            MotoCtr.ROLLSpeed = (Speed3 + Speed4) * (time_500ms - (ROLLFindZeroParams.PeriodCount - time_500ms - time_2sec)) / time_500ms;
        else {
            MotoCtr.ROLLSpeed = 0;
            ROLLFindZeroParams.PeriodCount = 0;
            ROLLFindZeroEndFlag = 1;
            ROLLFindZeroParams.State = 999;
            WorkMode = Mode_Stand_By;
        }
        break;
    default:
        break;
    }
}

#define POLZeroOn 1

// 极化寻零
void POLFindZero(void)
{
    float Speed1 = 10;
    float Speed2 = 5;
    float Speed3 = -5;
    float Speed4 = -10;

    unsigned short int time_2sec = 2 * _CTRCLK;
    unsigned short int time_3sec = 3 * _CTRCLK;
    unsigned short int time_5sec = 5 * _CTRCLK;

    //	MotoCtr.POLSpeed = Speed3;
    //	return;

    switch (POLFindZeroParams.State) {
    case 0:
        if (POLIO == POLZeroOn) {
            //上电时停留于开关位置
            POLFindZeroParams.State = 1;
            POLFindZeroParams.PeriodCount = 0;
        } else {
            POLFindZeroParams.State = 4;
            POLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 1:
        //低速反置2s，直至离开零位
        MotoCtr.POLSpeed = Speed2;
        if (POLIO != POLZeroOn) {
            //离开开关位置
            POLFindZeroParams.State = 2;
            POLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 2:
        //继续低速反转2s
        MotoCtr.POLSpeed = Speed2;
        if (POLFindZeroParams.PeriodCount++ > time_2sec) {
            POLFindZeroParams.State = 3;
            POLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 3:
        //低速转直至开关位置
        MotoCtr.POLSpeed = Speed3;
        if (POLIO == POLZeroOn) {
            //寻零结束
            POLFindZeroParams.State = 20;
            Senser.POLEncoder.Count = POL_EC_TIM->CNT;
            Senser.AngleOffset_POL = Senser.POLEncoder.Count + Senser.POLEncoder.PeriodCount * 40000;

            POLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 4:
        //低速转2s
        MotoCtr.POLSpeed = Speed3;
        //到达开关位置
        if (POLIO == POLZeroOn) {
            POLFindZeroParams.State = 1;
            POLFindZeroParams.PeriodCount = 0;
        }

        //转动时间超过2s
        if (POLFindZeroParams.PeriodCount++ > time_3sec) {
            POLFindZeroParams.State = 5;
            POLFindZeroParams.PeriodCount = 0;
        }

        break;
    case 5:
        //高速运转直至到开关位置或者硬限位
        MotoCtr.POLSpeed = Speed4;

        //到达光电开关
        if (POLIO == POLZeroOn) {
            POLFindZeroParams.State = 1;
            POLFindZeroParams.PeriodCount = 0;
        }

        if (Senser.SpeedPOL / Speed4 < 0) {
            POLFindZeroParams.State = 6;
            POLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 6:
        //高速反转至开关位置
        MotoCtr.POLSpeed = Speed1;
        //到达光电开关
        if (POLIO == POLZeroOn) {
            POLFindZeroParams.State = 1;
            POLFindZeroParams.PeriodCount = 0;
        }
        break;
    case 20:
        MotoCtr.POLSpeed = 0;
        POLFindZeroParams.PeriodCount = 0;
        POLFindZeroEndFlag = 1;
        POLFindZeroParams.State = 999;
        WorkMode = Mode_Stand_By;
        break;
    default:
        break;
    }
}

// 存储AGC信号最大值及其对应的空间位置
void StoreAgcMax()
{
    if (Senser.Agc[10] > Senser.AgcMAx.Agc) {
        Senser.AgcMAx.Agc = Senser.Agc[10];

        Senser.AgcMAx.azb = Senser.Angle_AZ[10];
        Senser.AgcMAx.elb = Senser.Angle_EL[10];
        Senser.AgcMAx.crb = Senser.Angle_ROLL[10];
        Senser.AgcMAx.head = Senser.Head[10];
        Senser.AgcMAx.pitch = Senser.Pitch[10];
        Senser.AgcMAx.roll = Senser.Roll[10];

        Senser.AgcMAx.AZg = Senser.AZg[10];
        Senser.AgcMAx.ELg = Senser.ELg[10];
    }
}

// 指向 标卫
void AimSatellite()
{
    float errAZ = 0, errEL = 0, errPOL = 0, errROLL;

    // 计算得到 标卫星的空间指向
    GetSatelliteAngle(&MotoCtr.AZgPerset, &MotoCtr.ELgPerset, &MotoCtr.PolgPerset);

    //通过空间指向计算天线坐标系方位置
    GtoB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
        Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
        &MotoCtr.AZPerset, &MotoCtr.ELPerset, &MotoCtr.ROLLPerset);

    //将空间指向转换为载体方位、俯仰角
    NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
        Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
        0, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

    //获取载体极化角
    GetPol(MotoCtr.AZPerset, MotoCtr.ELPerset, Senser.Head[10], Senser.Pitch[10], Senser.Roll[10], &MotoCtr.ROLLPerset);
    if (SatellitesInfo.polerMode == 0) {
        MotoCtr.ROLLPerset = MotoCtr.ROLLPerset;
    } else {
        MotoCtr.ROLLPerset = MotoCtr.ROLLPerset + 90;
        if (MotoCtr.ROLLPerset >= 360.0f) {
            MotoCtr.ROLLPerset -= 360.0f;
        }
    }

    MotoCtr.POLPerset = MotoCtr.ROLLPerset + 90;
    if (MotoCtr.POLPerset > 360) {
        MotoCtr.POLPerset = MotoCtr.POLPerset - 360;
    }
    if (MotoCtr.POLPerset < 0) {
        MotoCtr.POLPerset = MotoCtr.POLPerset + 360;
    }

    errAZ = fabs(MotoCtr.AZPerset - Senser.Angle_AZ[10]);
    errEL = fabs(MotoCtr.ELPerset - Senser.Angle_EL[10]);
    errROLL = fabs(MotoCtr.ROLLPerset - Senser.Angle_ROLL[10]);
    errPOL = fabs(MotoCtr.POLPerset - Senser.Angle_POL[10]);

    if (errAZ < 1.0f && errEL < 1.0f && errROLL < 3.0f && errPOL < 5.0f) {
        WorkMode = Mode_Tracking;
        TrackParams.CenterAZ = MotoCtr.AZgPerset;
        TrackParams.CenterEL = MotoCtr.ELgPerset;
        TrackParams.PeriodCount = 0;
        TrackParams.StartTimeCountAZ = 0;
        TrackParams.StartTimeCountEL = 0;
        TrackParams.Timer = 0;
        TrackParams.AmendEffectiv = 1;
        Senser.AgcMAx.Agc = 0;
        MotoCtr.Timer.az = 0;
        MotoCtr.Timer.el = 0;
        Senser.HeadOffset = 0;
    } else {
        if (1) {
            MotoCtr.AZSpeed = PositionControlAZPoint(MotoCtr.AZPerset, Senser.Angle_AZ);
            MotoCtr.ESpeed = PositionControlELPoint(MotoCtr.ELPerset, Senser.Angle_EL);
            MotoCtr.ROLLSpeed = PositionControlROLLPoint(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
            MotoCtr.POLSpeed = PositionControlPOLPoint(MotoCtr.POLPerset, Senser.Angle_POL);
        } else {
            MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
            MotoCtr.ESpeed = PositionControlEL(MotoCtr.ELPerset, Senser.Angle_EL);
            MotoCtr.ROLLSpeed = PositionControlROLL(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
            MotoCtr.POLSpeed = PositionControlPOLPoint(MotoCtr.POLPerset, Senser.Angle_POL);
        }
        //		MotoCtr.POLSpeed = PositionControlPOLPoint(MotoCtr.POLPerset, Senser.Angle_POL);
    }
}

/// 搜索扫描
/// <param name="AZCenter">方位角，地理坐标系</param>
/// <param name="ELCenter">俯仰角，地理坐标系</param>
/// <param name="azb">方位角，载体坐标系</param>
/// <param name="elb">俯仰角，载体坐标系</param>
void SearchCircle(float AZCenter, float ELCenter, float* AZb, float* ELb)
{
    //搜索策略：
    //在信号最大值附近进行方位来回扫描，扫描范围±5°，周期3s
    //当信号进入(最大值-1V)的范围内，认为扫描到信号，置AGCMax为0
    //在扫描到信号的附近进行方位来回扫描，扫描范围±0.5°，周期1s
    //指向信号最大值的点，反算航向，修正航向，转跟踪
    float a = 0;
    float e = 0;
    float errAZ = 0, errEL = 0;
    float headReal = 0;
    float HeadReal = 0, tempAZg = 0;

    switch (SearchParams.Step) {
    case 0:
        SearchParams.CenterAZ = AZCenter;
        SearchParams.CenterEL = ELCenter;
        SearchParams.PeriodCount = 0;
        SearchParams.RangeAZ = 10.0f;
        SearchParams.RangeEL = 0.0f;
        SearchParams.CircleTime = 5;
        SearchParams.PeriodCountStepTotal = SearchParams.CircleTime * _CTRCLK;
        SearchParams.Step = 1;
        //break;  //此break可以去除
    case 1:
        //一直来回扫描，直至信号进入AGCMax - 2
        if (SearchParams.PeriodCount < SearchParams.PeriodCountStepTotal) {
            a = SearchParams.RangeAZ * sin(2 * PI * SearchParams.PeriodCount / SearchParams.PeriodCountStepTotal);
            SearchParams.PeriodCount++;

            MotoCtr.AZgPerset = SearchParams.CenterAZ + a;

            GtoB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                &MotoCtr.AZPerset, &MotoCtr.ELPerset, &MotoCtr.ROLLPerset);

            //将空间指向转换为载体方位、俯仰角
            NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                0, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

            //获取载体极化角
            GetPol(MotoCtr.AZPerset, MotoCtr.ELPerset, Senser.Head[10], Senser.Pitch[10], Senser.Roll[10], &MotoCtr.ROLLPerset);
            if (SatellitesInfo.polerMode == 0) {
                MotoCtr.ROLLPerset = MotoCtr.ROLLPerset;
            } else {
                MotoCtr.ROLLPerset = MotoCtr.ROLLPerset + 90;
                if (MotoCtr.ROLLPerset >= 360.0f) {
                    MotoCtr.ROLLPerset -= 360.0f;
                }
            }

            if (Senser.Agc[10] > Senser.AgcMAx.Agc - 2) {
                SearchParams.Step = 2;
                SearchParams.CenterAZ = Senser.AZg[10];
                SearchParams.CenterEL = Senser.ELg[10];
                Senser.AgcMAx.Agc = 0;
                SearchParams.PeriodCount = 0;
                SearchParams.PeriodCountStepTotal = 1 * _CTRCLK;
                SearchParams.RangeAZ = 0.5;
            }
        } else {
            SearchParams.PeriodCount = 0;
        }
        break;
    case 2:
        //再进行一次小范围扫描
        if (SearchParams.PeriodCount < SearchParams.PeriodCountStepTotal) {
            a = SearchParams.RangeAZ * sin(2 * PI * SearchParams.PeriodCount / SearchParams.PeriodCountStepTotal);
            SearchParams.PeriodCount++;

            MotoCtr.AZgPerset = SearchParams.CenterAZ + a;

            GtoB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                &MotoCtr.AZPerset, &MotoCtr.ELPerset, &MotoCtr.ROLLPerset);

            //将空间指向转换为载体方位、俯仰角
            NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                0, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

            //获取载体极化角
            GetPol(MotoCtr.AZPerset, MotoCtr.ELPerset, Senser.Head[10], Senser.Pitch[10], Senser.Roll[10], &MotoCtr.ROLLPerset);
            if (SatellitesInfo.polerMode == 0) {
                MotoCtr.ROLLPerset = MotoCtr.ROLLPerset;
            } else {
                MotoCtr.ROLLPerset = MotoCtr.ROLLPerset + 90;
                if (MotoCtr.ROLLPerset >= 360.0f) {
                    MotoCtr.ROLLPerset -= 360.0f;
                }
            }

        } else {
            SearchParams.Step = 3;
        }
        break;
    case 3:
        //指向信号最大值角度
        GtoB(Senser.AgcMAx.AZg, Senser.AgcMAx.ELg,
            Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
            &MotoCtr.AZPerset, &MotoCtr.ELPerset, &MotoCtr.ROLLPerset);

        //将空间指向转换为载体方位、俯仰角
        NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
            Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
            0, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

        //获取载体极化角
        GetPol(MotoCtr.AZPerset, MotoCtr.ELPerset, Senser.Head[10], Senser.Pitch[10], Senser.Roll[10], &MotoCtr.ROLLPerset);
        if (SatellitesInfo.polerMode == 0) {
            MotoCtr.ROLLPerset = MotoCtr.ROLLPerset;
        } else {
            MotoCtr.ROLLPerset = MotoCtr.ROLLPerset + 90;
            if (MotoCtr.ROLLPerset >= 360.0f) {
                MotoCtr.ROLLPerset -= 360.0f;
            }
        }

        errAZ = fabs(MotoCtr.AZPerset - Senser.Angle_AZ[10]);
        errEL = fabs(MotoCtr.ELPerset - Senser.Angle_EL[10]);

        if (errAZ < 0.1f && errEL < 0.1f) {
            //通过载体数据、卫星数据计算空间指向
            GetSatelliteAngle(&MotoCtr.AZgPerset, &MotoCtr.ELgPerset, &MotoCtr.PolgPerset);

            HeadReal = GetH(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Angle_AZ[10],
                Senser.Pitch[10], Senser.Roll[10]);

            GDAmand.IsAmand = 1;
            GDAmand.Value = HeadReal;
            HeadCheckParams.PeriodCount = 0;
            WorkMode = Mode_Tracking;
        }
    default:
        break;
    }
}

/// <summary>
/// 搜索扫描，搜索到AGC最大值的方位、俯仰角度（地理坐标系下）
/// 当信标接收机锁定且AGC最大值大于跟踪阈值切入自跟踪模式；
/// 若AGC最大值小于跟踪阈值则切到初始对星模式
/// <param name="AZg">方位角，地理坐标系</param>
/// <param name="ELg">俯仰角，地理坐标系</param>
/// </summary>
void SearchAGCMax(float AZg, float ELg)
{
    SearchCircle(AZg, ELg, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

    MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
    MotoCtr.ESpeed = PositionControlEL(MotoCtr.ELPerset, Senser.Angle_EL);
    MotoCtr.ROLLSpeed = PositionControlROLL(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
    MotoCtr.POLSpeed = PositionControlPOLPoint(MotoCtr.POLPerset, Senser.Angle_POL);

    SearchParams.SearchTime++;
    if (SearchParams.SearchTime > 5 * 60 * _CTRCLK) {
        //若转入搜索12分钟后，仍未搜索到信号，则进行航向校准
        HeadCheckFlag = 0;
        HeadCheckParams.State = 0;
        Senser.AgcMAx.Agc = 0;
        MotoCtr.Timer.az = 0;
        MotoCtr.Timer.el = 0;
        Senser.AgcLostFlag = 1;
    }
}

// 程序引导
void Leading()
{
}

/// <summary>
/// 速度环调试
/// </summary>
void DebugSpeed()
{
}

/// <summary>
/// 位置环调试
/// </summary>
void DebugPosition()
{
    if (MotoCtr.AZEnalbe == 0) {
        MotoCtr.AZSpeed = 0;
    } else {
        MotoCtr.AZSpeed = PositionControlAZPoint(MotoCtr.AZPerset, Senser.Angle_AZ);
    }

    if (MotoCtr.ELEnalbe == 0) {
        MotoCtr.ESpeed = 0;
    } else {
        MotoCtr.ESpeed = PositionControlELPoint(MotoCtr.ELPerset, Senser.Angle_EL);
    }

    if (MotoCtr.ROLLEnalbe == 0) {
        MotoCtr.ROLLSpeed = 0;
    } else {
        MotoCtr.ROLLSpeed = PositionControlROLLPoint(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
    }

    if (MotoCtr.POLEnalbe == 0) {
        MotoCtr.POLSpeed = 0;
    } else {
        MotoCtr.POLSpeed = PositionControlPOLPoint(MotoCtr.POLPerset, Senser.Angle_POL);
    }
}

/// <summary>
/// 速度环正弦调试
/// </summary>
void DebugSpeedSin()
{
    float _Period = 0, _VelocityMax = 0;

    _Period = VSINPERIOD;
    _VelocityMax = VSINRANGE;

    if (MotoCtr.AZEnalbe == 0)
        //方位未使能
        MotoCtr.AZSpeed = 0;
    else {
        MotoCtr.AZSpeed = _VelocityMax * sin(2 * PI * MotoCtr.PeriodCountSinAZ++ / MotoCtr.PeriodCountTotal);
        if (MotoCtr.PeriodCountSinAZ > MotoCtr.PeriodCountTotal) {
            MotoCtr.PeriodCountSinAZ = 1;
            MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
        }
    }

    if (MotoCtr.ELEnalbe == 0)
        //俯仰未使能
        MotoCtr.ESpeed = 0;
    else {
        MotoCtr.ESpeed = _VelocityMax * sin(2 * PI * MotoCtr.PeriodCountSinEL++ / MotoCtr.PeriodCountTotal);
        if (MotoCtr.PeriodCountSinEL > MotoCtr.PeriodCountTotal) {
            MotoCtr.PeriodCountSinEL = 1;
            MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
        }
    }

    if (MotoCtr.ROLLEnalbe == 0)
        //横滚未使能
        MotoCtr.ROLLSpeed = 0;
    else {
        MotoCtr.ROLLSpeed = _VelocityMax * sin(2 * PI * MotoCtr.PeriodCountSinROLL++ / MotoCtr.PeriodCountTotal);
    }
    if (MotoCtr.PeriodCountSinROLL > MotoCtr.PeriodCountTotal) {
        MotoCtr.PeriodCountSinROLL = 1;
        MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
    }

    if (MotoCtr.POLEnalbe == 0)
        //极化未使能
        MotoCtr.POLSpeed = 0;
    else {
        MotoCtr.POLSpeed = _VelocityMax * sin(2 * PI * MotoCtr.PeriodCountSinPRx++ / MotoCtr.PeriodCountTotal);
    }
    if (MotoCtr.PeriodCountSinPRx > MotoCtr.PeriodCountTotal) {
        MotoCtr.PeriodCountSinPRx = 1;
        MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
    }
}

// 位置 正弦
void DebugPositionSin()
{
    float _Period = 0;
    float _Range = 0;

    _Range = SSINRANGE;
    _Period = SSINPERIOD;

    if (MotoCtr.AZEnalbe == 0) {
        MotoCtr.AZSpeed = 0;
    } else {
        MotoCtr.AZPerset = MotoCtr.AZCenter + _Range * cos(2 * PI * MotoCtr.PeriodCountSinAZ++ / MotoCtr.PeriodCountTotal);

        if (MotoCtr.PeriodCountSinAZ > MotoCtr.PeriodCountTotal) {
            MotoCtr.PeriodCountSinAZ = 1;
            MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
        }
        if (MotoCtr.AZPerset < 0) {
            MotoCtr.AZPerset += 360;
        }
        AZControl(MotoCtr.AZPerset);
    }

    if (MotoCtr.ELEnalbe == 0) {
        MotoCtr.ESpeed = 0;
    } else {
        MotoCtr.ELPerset = MotoCtr.ELCenter + _Range * sin(2 * PI * MotoCtr.PeriodCountSinEL++ / MotoCtr.PeriodCountTotal);
        if (MotoCtr.PeriodCountSinEL > MotoCtr.PeriodCountTotal) {
            MotoCtr.PeriodCountSinEL = 1;
            MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
        }
        ELControl(MotoCtr.ELPerset);
    }

    if (MotoCtr.ROLLEnalbe == 0) {
        MotoCtr.ROLLSpeed = 0;
    } else {
        MotoCtr.ROLLPerset = MotoCtr.ROLLCenter + _Range * sin(2 * PI * MotoCtr.PeriodCountSinROLL++ / MotoCtr.PeriodCountTotal);
        if (MotoCtr.PeriodCountSinROLL > MotoCtr.PeriodCountTotal) {
            MotoCtr.PeriodCountSinROLL = 1;
            MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
        }
        MotoCtr.ROLLSpeed = PositionControlROLL(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
    }

    if (MotoCtr.POLEnalbe == 0) {
        MotoCtr.POLSpeed = 0;
    } else {
        MotoCtr.POLPerset = MotoCtr.PolRxCenter + _Range * sin(2 * PI * MotoCtr.PeriodCountSinPRx++ / MotoCtr.PeriodCountTotal);
        if (MotoCtr.PeriodCountSinPRx > MotoCtr.PeriodCountTotal) {
            MotoCtr.PeriodCountSinPRx = 1;
            MotoCtr.PeriodCountTotal = _Period * _CTRCLK;
        }
        MotoCtr.POLSpeed = PositionControlPOL(MotoCtr.POLPerset, Senser.Angle_POL);
    }
}

/// 五点外推算法
/// <param name="PastGather">过去数据集合</param>
/// <param name="LookHeadTime">先行时间，单位ms</param>
/// <param name="FREQ">控制频率</param>
/// <param name="Index">最新数据索引</param>
float Extrapolate(float* PastGather, int LookHeadTime, int FREQ, int Index)
{
    float result = 0;
    int count = LookHeadTime / (1000 / FREQ);
    if ((Index - count * 4) >= 0) {
        float x0 = PastGather[Index - count * 4];
        float x1 = PastGather[Index - count * 3];
        float x2 = PastGather[Index - count * 2];
        float x3 = PastGather[Index - count * 1];
        float x4 = PastGather[Index];
        //x4 = 5;
        result = ((x3 - x0) - (x4 - x0) + (x1 - x0)) / 16.0f
            + (((x4 - x0) - (x1 - x0)) * 2 + (x4 - x0) - (x2 - x0)) / 4.0f
            + (x4 - x0) - (x1 - x0) + x0;
    }
    return result;
}

/// 方位位置环控制
/// <param name="PPerset">预置角</param>
void AZControl(float PPerset)
{
    if ((int)AZInc == 1) {
        if (WorkMode == Mode_Tracking) {
            MotoCtr.AZSpeed = uLimit(MotoCtr.AZSpeed, 50);
        } else {
            MotoCtr.AZSpeed = uLimit(MotoCtr.AZSpeed, 35);
        }
    } else {
        MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
    }
}

//  仰位置 控制
void ELControl(float PPerset)
{
    MotoCtr.ESpeed = PositionControlEL(MotoCtr.ELPerset, Senser.Angle_EL);
}

/// 计算预置与当前误差
/// <param name="PPersetAZ">方位预置角</param>
/// <param name="PPersetEL">俯仰预置角</param>
/// <param name="ErrAZ">方位误差角</param>
/// <param name="ErrEL">俯仰误差角</param>
void GetAngleErr(float PPersetAZ, float PPersetEL, float* ErrAZ, float* ErrEL)
{
    *ErrAZ = fabs(PPersetAZ - Senser.Angle_AZ[10]);

    *ErrEL = fabs(PPersetEL - Senser.Angle_EL[10]);
}

/// <summary>
/// 将角度转换为0~360°范围
/// </summary>
/// <param name="angle">角度</param>
float AngleConvert(float angle)
{
    float temp = angle;
    int multiple = temp / 360;

    if (multiple < 0)
        multiple -= 1;
    temp = temp + (-1) * multiple * 360;

    if (temp > 360)
        temp = temp - 360;
    if (temp < 0)
        temp = temp + 360;
    return temp;
}

int AmendCount = 0;
long LostCountInTrack = 0;

//圆扫 跟踪
float g_headReAmend = 0;
float azAdjustOld = 0;
void TrackCircle()
{
    float a = 0;
    float e = 0;

    float AgcPAZ = 0, AgcPEL = 0;
    float _azg = 0, _elg = 0;
    float _fre = 0;
    float CirclePeriodAZ = 0;
    float errAZ = 0, errEL = 0;
    float _azAdjust = 0;
    int agcDelay = 0;

    AgcPAZ = CIRCLERSTEPAZ;
    AgcPEL = CIRCLERSTEPEL;

    TrackParams.RangeAZ = CIRCLERANGEAZ;
    TrackParams.RangeEL = CIRCLERANGEEL;

    CirclePeriodAZ = CIRCLERPERIODAZ;

    if (CIRCLE_S != 1) {
        AgcPAZ = 0;
        AgcPEL = 0;
        TrackParams.RangeAZ = 0;
        TrackParams.RangeEL = 0;
    }

    TrackParams.PeriodCountStepTotal = CirclePeriodAZ * _CTRCLK;

    if (TrackParams.PeriodCount > 500 || TrackParams.PeriodCount < 0) {
        TrackParams.PeriodCount = 0;

        TrackParams._value1 = 0;
        TrackParams._value2 = 0;
        TrackParams._value3 = 0;
        TrackParams._value4 = 0;
    }

    a = TrackParams.RangeAZ * sin(2 * PI * TrackParams.PeriodCount / TrackParams.PeriodCountStepTotal);
    e = TrackParams.RangeEL * cos(2 * PI * TrackParams.PeriodCount / TrackParams.PeriodCountStepTotal);

    TrackParams.PeriodCount++;

    MotoCtr.AZgPerset = TrackParams.CenterAZ + a;
    MotoCtr.ELgPerset = TrackParams.CenterEL + e;
    GtoB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
        Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
        &MotoCtr.AZPerset, &MotoCtr.ELPerset, &MotoCtr.ROLLPerset);

    //将空间指向转换为载体方位、俯仰角
    NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
        Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
        0, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

    //获取载体极化角
    GetPol(MotoCtr.AZPerset, MotoCtr.ELPerset, Senser.Head[10], Senser.Pitch[10], Senser.Roll[10], &MotoCtr.ROLLPerset);
    if (SatellitesInfo.polerMode == 0) {
        MotoCtr.ROLLPerset = MotoCtr.ROLLPerset;
    } else {
        MotoCtr.ROLLPerset = MotoCtr.ROLLPerset + 90;
        if (MotoCtr.ROLLPerset >= 360.0f) {
            MotoCtr.ROLLPerset -= 360.0f;
        }
    }

    agcDelay = 5;
    if (TrackParams.PeriodCount >= (TrackParams.PeriodCountStepTotal * 1 / 4) - 2 + agcDelay
        && TrackParams.PeriodCount <= (TrackParams.PeriodCountStepTotal * 1 / 4) + 2 + agcDelay) {
        TrackParams._value1 += Senser.Agc[10];
    }

    if (TrackParams.PeriodCount >= (TrackParams.PeriodCountStepTotal * 3 / 4) - 2 + agcDelay
        && TrackParams.PeriodCount <= (TrackParams.PeriodCountStepTotal * 3 / 4) + 2 + agcDelay) {
        TrackParams._value2 += Senser.Agc[10];
    }

    if (TrackParams.PeriodCount >= (TrackParams.PeriodCountStepTotal * 1 / 2) - 2 + agcDelay
        && TrackParams.PeriodCount < TrackParams.PeriodCountStepTotal * 1 / 2 + 2 + agcDelay) {
        TrackParams._value3 += Senser.Agc[10];
    }

    if (TrackParams.PeriodCount >= -2 + agcDelay
        && TrackParams.PeriodCount < 2 + agcDelay) {
        TrackParams._value4 += Senser.Agc[10];
    }

    errAZ = MotoCtr.AZPerset - Senser.Angle_AZ[10];
    if (errAZ > 180) {
        errAZ = errAZ - 360;
    }
    if (errAZ < -180) {
        errAZ = errAZ + 360;
    }
    errEL = MotoCtr.ELPerset - Senser.Angle_EL[10];

    if (Senser.Agc[10] < Senser.AgcMAx.Agc - 3.0f * 1.0f) {
        TrackParams.AmendEffectiv = 0;
    }

    if (TrackParams.PeriodCount >= TrackParams.PeriodCountStepTotal) {
        TrackParams.PeriodCount = 0;

        MotoCtr.angleAdjustEL = AgcPEL * (TrackParams._value3 - TrackParams._value4) / 100.0f;
        MotoCtr.angleAdjustEL = uLimit(MotoCtr.angleAdjustEL, 0.2);
        if (TrackParams.AmendEffectiv != 0) {
            TrackParams.CenterEL = TrackParams.CenterEL + MotoCtr.angleAdjustEL;
        }
        MotoCtr.angleAdjustAZ = AgcPAZ * (TrackParams._value1 - TrackParams._value2) / 100.0f;
        _azAdjust = uLimit(MotoCtr.angleAdjustAZ, 0.1);
        if (TrackParams.AmendEffectiv != 0) {
            Senser.HeadOffset += _azAdjust;
            AmendCount++;
            if (AmendCount > 3) {
                AmendCount = 0;
#if MEMS_TYPE == 1
                if (CIRCLE_S == 1)
#endif
                {
                    GDAmand.IsAmand = 1;
                }
                GDAmand.Value = Senser.Head[10] + MotoCtr.angleAdjustAZ;
                if (GDAmand.Value >= 360) {
                    GDAmand.Value -= 360;
                }
                if (GDAmand.Value < 0) {
                    GDAmand.Value += 360;
                }

                if (AmendCount > 100) {
                    AmendCount = 3;
                }
            }
        }
        TrackParams._value1 = 0;
        TrackParams._value2 = 0;
        TrackParams._value3 = 0;
        TrackParams._value4 = 0;
        TrackParams.AmendEffectiv = 1;
    }

    if (Senser.Agc[10] < Senser.AgcMAx.Agc - 3 * 1.0f) {
        LostCountInTrack++;
    } else {
        LostCountInTrack = 0;
    }

    if (LostCountInTrack > 1 * 60 * _CTRCLK) {
        LostCountInTrack = 0;
        WorkMode = Mode_Searching;
        SearchParams.PeriodCount = 0;
        SearchParams.Step = 0;
        SearchParams.SearchTime = 0;
    }
}

float HeadCheckStatus = 0;
//初始航向校准
void HeadCalibrate()
{
    unsigned short int time_1sec = 1 * _CTRCLK;
    unsigned short int time_2sec = 2 * _CTRCLK;
    unsigned short int time_4sec = 4 * _CTRCLK;
    unsigned short int time_18sec = 36 * _CTRCLK;

    float errAZ = 0, errEL = 0, errROLL = 0, errPOL = 0;
    //计算得到参考卫星相对于天线的空间指向
    float azg = 0, elg = 0, polg = 0;
    //通过参考星得到真实
    float HeadReal = 0;
    //测试 ，用于存储ConverGToB计算得到的elbp
    float elbp = 0, rollbp = 0;

    if (AZFindZeroEndFlag != 1
        || ELFindZeroEndFlag != 1
        || ROLLFindZeroEndFlag != 1
        || POLFindZeroEndFlag != 1) {
        //若各轴寻零未完成，则直接跳出，不进入航向校准
        return;
    }

    Senser.AgcFindBugTime++;
    switch (HeadCheckParams.State) {
    case 0:
        HeadCheckParams.State = 1;
        HeadCheckStatus = 1;
        HeadCheckParams.PeriodCount = 0;
        break;
    case 1:
        //等待2s时间，保证DVB参数设置成功
        if (HeadCheckParams.PeriodCount++ > time_2sec) {
            HeadCheckParams.State = 2;
            HeadCheckStatus = 2;
            HeadCheckParams.PeriodCount = 0;
            MotoCtr.Timer.az = 0;
            MotoCtr.Timer.el = 0;
            MotoCtr.Timer.roll = 0;
            MotoCtr.Timer.pol = 0;
        }
        StandBy();
        AimSateOfNorm(0);
        Senser.AgcMAx.Agc = 0;
        break;
    case 2:
//等待惯导状态变为惯性导航或与GPS结合导航后，控制天线俯仰、横滚、极化转动到指向位置
#if (MEMS_TYPE == 0) || (MEMS_TYPE == 2)
        if (((Senser.MEMSInfo.Status >> 4) != 9) && ((Senser.MEMSInfo.Status >> 4) != 7)) {
#elif MEMS_TYPE == 1
        if ((Senser.MEMSInfo.Status) != 0xF0) {
#endif
            MotoCtr.AZSpeed = 0;
            AimSateOfNorm(0);
            MotoCtr.Timer.az = 0;
            MotoCtr.Timer.el = 0;
            MotoCtr.Timer.roll = 0;
            MotoCtr.Timer.pol = 0;
        } else {
            MotoCtr.AZSpeed = 0;
            AimSateOfNorm(1);

            errEL = fabs(MotoCtr.ELPerset - Senser.Angle_EL[10]);
            errROLL = fabs(MotoCtr.ROLLPerset - Senser.Angle_ROLL[10]);
            // errPOL = fabs(MotoCtr.POLPerset - Senser.Angle_POL[10]);

            if (errEL < 1.0f && errPOL < 3.0f && errROLL < 3.0f) {
                //天线俯仰、横滚、极化指向
                HeadCheckParams.State = 3;
                HeadCheckStatus = 3;
                HeadCheckParams.PeriodCount = 0;
                HeadCheckParams.StartAZg = Senser.AZg[10];
                Senser.AgcMAx.Agc = 0;
            }
        }
        break;
    case 3:
        //保持天线俯仰、横滚、极化
        AimSateOfNorm(1);

        if (HeadCheckParams.PeriodCount++ < time_18sec + time_2sec) {
            MotoCtr.AZgPerset = HeadCheckParams.StartAZg + 360.0 * HeadCheckParams.PeriodCount * 1.0 / time_18sec;
            if (MotoCtr.AZgPerset >= 360) {
                MotoCtr.AZgPerset -= 360;
            }
            if (MotoCtr.AZgPerset < 0) {
                MotoCtr.AZgPerset += 360;
            }
            MotoCtr.AZPerset = GetA_(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10]);
            //            NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
            //                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
            //                Senser.Angle_AZ[10],
            //                &MotoCtr.AZPerset, NULL);

            MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
        } else {
            if (Senser.AgcMAx.Agc > 3.0f) {
                HeadCheckParams.PeriodCount = 0;
                HeadCheckParams.State = 4;
                HeadCheckStatus = 4;
            } else {
                //若旋转一周未找到信号，则继续旋转
                HeadCheckParams.PeriodCount = time_2sec;
            }
        }
        break;
    case 4:
        //保持天线俯仰、横滚、极化
        AimSateOfNorm(1);

        //等待1s，天线指向旋转过程中AGC最大值对应的位置
        if (HeadCheckParams.PeriodCount++ < time_1sec) {

            MotoCtr.AZgPerset = Senser.AgcMAx.AZg;
            MotoCtr.ELgPerset = Senser.AgcMAx.ELg;
            MotoCtr.AZPerset = GetA_(Senser.AgcMAx.AZg, Senser.AgcMAx.ELg,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10]);
            //            NewConvertGTOB(Senser.AgcMAx.AZg, Senser.AgcMAx.ELg,
            //                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
            //                Senser.Angle_AZ[10],
            //                &MotoCtr.AZPerset, NULL);

            MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);

            HeadCheckStatus = 4.1;
        } else {
            MotoCtr.AZgPerset = Senser.AgcMAx.AZg;
            MotoCtr.ELgPerset = Senser.AgcMAx.ELg;
            MotoCtr.AZPerset = GetA_(Senser.AgcMAx.AZg, Senser.AgcMAx.ELg,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10]);
            //            NewConvertGTOB(Senser.AgcMAx.AZg, Senser.AgcMAx.ELg,
            //                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
            //                Senser.Angle_AZ[10],
            //                &MotoCtr.AZPerset, NULL);

            MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);

            errAZ = fabs(MotoCtr.AZPerset - Senser.Angle_AZ[10]);
            errEL = fabs(MotoCtr.ELPerset - Senser.Angle_EL[10]);
            errROLL = fabs(MotoCtr.ROLLPerset - Senser.Angle_ROLL[10]);
            // errPOL = fabs(MotoCtr.POLPerset - Senser.Angle_POL[10]);
            HeadCheckStatus = 4.2;
            if (errAZ < 1.0f && errEL < 1.0f && errROLL < 5.0f && errPOL < 5.0f && Senser.Agc[10] > Senser.AgcMAx.Agc * 0.85) {
                //指向参考卫星完成
                HeadCheckParams.State = 5;
                HeadCheckStatus = 5;
                HeadCheckParams.PeriodCount = 0;
            }

            if (HeadCheckParams.PeriodCount++ > time_4sec + time_4sec) {
                HeadCheckParams.State = 1;
                HeadCheckStatus = 1;
                HeadCheckParams.PeriodCount = 0;
            }
        }
        break;
    case 5:
        // 在信号最大位置进行一次指向参考卫星附近进行一次4s的搜索
        AimSateOfNorm(1); //保持俯仰、横滚、极化

        if (HeadCheckParams.PeriodCount < time_4sec) {
            MotoCtr.AZgPerset = Senser.AgcMAx.AZg + 10 * sin((1.0f * HeadCheckParams.PeriodCount++ / time_4sec) * 2 * PI);
            MotoCtr.AZPerset = GetA_(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10]);
            //            NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
            //                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
            //                Senser.Angle_AZ[10],
            //                &MotoCtr.AZPerset, NULL);

            MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
        } else {
            //搜索完成
            HeadCheckParams.State = 6;
            HeadCheckStatus = 7;
            HeadCheckParams.State = 7; //20190509
            HeadCheckParams.PeriodCount = 0;
        }
        break;
    case 6:
        //等待1s
        AimSateOfNorm(1); //保持俯仰、横滚、极化

        if (HeadCheckParams.PeriodCount < time_1sec) {
            MotoCtr.Timer.az = 0;
            MotoCtr.AZSpeed = MotoCtr.AZSpeed * 0.95f;
            HeadCheckParams.PeriodCount++;
        } else {
            HeadCheckParams.State = 7;
            HeadCheckStatus = 7;
            HeadCheckParams.PeriodCount = 0;
            MotoCtr.Timer.az = 0;
        }
        break;
    case 7:
        //保持俯仰、横滚、极化
        AimSateOfNorm(1);

        //天线指向搜索过程中AGC最大值对应的位置
        MotoCtr.AZgPerset = Senser.AgcMAx.AZg;
        MotoCtr.ELgPerset = Senser.AgcMAx.ELg;
        MotoCtr.AZPerset = GetA_(Senser.AgcMAx.AZg, Senser.AgcMAx.ELg,
            Senser.Head[10], Senser.Pitch[10], Senser.Roll[10]);
        //        NewConvertGTOB(Senser.AgcMAx.AZg, Senser.AgcMAx.ELg,
        //            Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
        //            Senser.Angle_AZ[10],
        //            &MotoCtr.AZPerset, NULL);

        MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);

        errAZ = fabs(MotoCtr.AZPerset - Senser.Angle_AZ[10]);
        errEL = fabs(MotoCtr.ELPerset - Senser.Angle_EL[10]);
        errROLL = fabs(MotoCtr.ROLLPerset - Senser.Angle_ROLL[10]);
        // errPOL = fabs(MotoCtr.POLPerset - Senser.Angle_POL[10]);
        if (errAZ < 1.0f && errEL < 1.0f && errROLL < 3.0f && Senser.Agc[10] > Senser.AgcMAx.Agc - 0.8f) {
            //计算得到参考卫星相对于天线的空间指向
            GetAngleGOfNormalSate(&azg, &elg, &polg);
            MotoCtr.AZgPerset = azg;
            MotoCtr.ELgPerset = elg;
            HeadReal = GetH(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Angle_AZ[10],
                Senser.Pitch[10], Senser.Roll[10]);

            GDAmand.IsAmand = 1;
            GDAmand.Value = HeadReal;
            GDAmand.isFirst = 1;
            HeadCheckParams.PeriodCount = 0;

            HeadCheckParams.State = 8;
            HeadCheckStatus = 8;
        }
        if (HeadCheckParams.PeriodCount++ > time_4sec + time_4sec) {
            HeadCheckParams.State = 1;
            HeadCheckStatus = 1;
            HeadCheckParams.PeriodCount = 0;
        }
        break;
    case 8:
        if (HeadCheckParams.PeriodCount++ < time_1sec + time_2sec) {
            if (HeadCheckParams.PeriodCount < time_1sec * 0.3) {
                AimSateOfNorm(1);
                GetAngleGOfNormalSate(&azg, &elg, &polg);
                MotoCtr.AZgPerset = azg;
                MotoCtr.ELgPerset = elg;
                MotoCtr.AZPerset = GetA_(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                    Senser.Head[10], Senser.Pitch[10], Senser.Roll[10]);
                //                NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                //                    Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                //                    Senser.Angle_AZ[10],
                //                    &MotoCtr.AZPerset, NULL);
                if (HeadCheckParams.PeriodCount > 2) {
                    MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
                }
            } else {

                GetSatelliteAngle(&MotoCtr.AZgPerset, &MotoCtr.ELgPerset, &MotoCtr.PolgPerset);
                GtoB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                    Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                    &MotoCtr.AZPerset, &MotoCtr.ELPerset, &MotoCtr.ROLLPerset);

                //将空间指向转换为载体方位、俯仰角
                NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                    Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                    0, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

                //获取载体极化角
                GetPol(MotoCtr.AZPerset, MotoCtr.ELPerset, Senser.Head[10], Senser.Pitch[10], Senser.Roll[10], &MotoCtr.ROLLPerset);
                if (SatellitesInfo.polerMode == 0) {
                    MotoCtr.ROLLPerset = MotoCtr.ROLLPerset;
                } else {
                    MotoCtr.ROLLPerset = MotoCtr.ROLLPerset + 90;
                    if (MotoCtr.ROLLPerset >= 360.0f) {
                        MotoCtr.ROLLPerset -= 360.0f;
                    }
                }

                MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
                MotoCtr.ESpeed = PositionControlEL(MotoCtr.ELPerset, Senser.Angle_EL);
                MotoCtr.ROLLSpeed = PositionControlROLL(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
            }
        } else {
            GtoB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                &MotoCtr.AZPerset, &MotoCtr.ELPerset, &MotoCtr.ROLLPerset);

            //将空间指向转换为载体方位、俯仰角
            NewConvertGTOB(MotoCtr.AZgPerset, MotoCtr.ELgPerset,
                Senser.Head[10], Senser.Pitch[10], Senser.Roll[10],
                0, &MotoCtr.AZPerset, &MotoCtr.ELPerset);

            //获取载体极化角
            GetPol(MotoCtr.AZPerset, MotoCtr.ELPerset, Senser.Head[10], Senser.Pitch[10], Senser.Roll[10], &MotoCtr.ROLLPerset);
            if (SatellitesInfo.polerMode == 0) {
                MotoCtr.ROLLPerset = MotoCtr.ROLLPerset;
            } else {
                MotoCtr.ROLLPerset = MotoCtr.ROLLPerset + 90;
                if (MotoCtr.ROLLPerset >= 360.0f) {
                    MotoCtr.ROLLPerset -= 360.0f;
                }
            }

            MotoCtr.AZSpeed = PositionControlAZ(MotoCtr.AZPerset, Senser.Angle_AZ);
            MotoCtr.ESpeed = PositionControlEL(MotoCtr.ELPerset, Senser.Angle_EL);
            MotoCtr.ROLLSpeed = PositionControlROLL(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
            MotoCtr.POLSpeed = PositionControlPOL(MotoCtr.POLPerset, Senser.Angle_POL);

            HeadCheckFlag = 1;
            HeadCheckParams.State = 99;
            HeadCheckStatus = 99;
            HeadCheckParams.PeriodCount = 0;

            // MotoCtr.Timer.az = 0;
            // MotoCtr.Timer.el = 0;
            // MotoCtr.Timer.roll = 0;
            // MotoCtr.Timer.pol = 0;

            if (AUTOTK_S == 1) {
                WorkMode = Mode_AimSatellite;
            } else {
                WorkMode = Mode_Stand_By;
            }
            // AimSateOfNorm(1);
            Senser.AgcFindBugTime = 0;
            Senser.AgcLostFlag = 0;
            BeaconInfo.freq = SatellitesInfo.freq - SatellitesInfo.localOsci;
        }
        break;
    default:
        break;
    }
}

// 计算天线指向参考卫星的角度
// <param IsEnable>0:无效,仅做计算,天线不转 ;1:有效,天线  </param IsEnable>
void AimSateOfNorm(unsigned char IsEnable)
{
    float egRad, aRad, cRad, pRad, rRad;
    const float pi = 3.141592653;

    //计算得到参考卫星相对于天线的空间指向角及极化角
    GetAngleGOfNormalSate(&MotoCtr.AZgPerset, &MotoCtr.ELgPerset, &MotoCtr.PolgPerset);

    aRad = Senser.Angle_AZ[10] * pi / 180.0f;
    pRad = Senser.Pitch[10] * pi / 180.0f;
    rRad = Senser.Roll[10] * pi / 180.0f;
    MotoCtr.ROLLPerset = GetC(aRad, pRad, rRad);

    cRad = MotoCtr.ROLLPerset * pi / 180.0f;
    egRad = MotoCtr.ELgPerset * pi / 180.0f;
    MotoCtr.ELPerset = GetE(egRad, aRad, cRad, pRad, rRad);

    if (SatellitesInfo.polerMode == 0) {
        MotoCtr.POLPerset = MotoCtr.PolgPerset;
    } else {
        MotoCtr.POLPerset = MotoCtr.PolgPerset + 90;
    }

    if (MotoCtr.POLPerset < 0) {
        MotoCtr.POLPerset += 180;
    } else if (MotoCtr.POLPerset >= 180) {
        MotoCtr.POLPerset -= 180;
    }

    if (MotoCtr.POLPerset >= 15 && MotoCtr.POLPerset <= 195) {
        MotoCtr.POLPerset -= 180;
    }

    if (MotoCtr.POLPerset < 0) {
        MotoCtr.POLPerset += 360;
    }

    GetElb_HeadCheck(MotoCtr.ELgPerset,
        Senser.Pitch[10], Senser.Roll[10],
        Senser.Angle_AZ[10], &MotoCtr.ELPerset);

    if (SatellitesInfo.polerMode == 0) {
        MotoCtr.ROLLPerset = MotoCtr.PolgPerset;
    } else {
        MotoCtr.ROLLPerset = MotoCtr.PolgPerset + 90;
        if (MotoCtr.ROLLPerset >= 360.0f) {
            MotoCtr.ROLLPerset -= 360.0f;
        }
    }
    MotoCtr.POLPerset = MotoCtr.PolgPerset + 90;
    if (MotoCtr.POLPerset >= 360.0f) {
        MotoCtr.POLPerset -= 360.0f;
    }

    MotoCtr.ESpeed = PositionControlEL(MotoCtr.ELPerset, Senser.Angle_EL);
    MotoCtr.ROLLSpeed = PositionControlROLL(MotoCtr.ROLLPerset, Senser.Angle_ROLL);
    MotoCtr.POLSpeed = PositionControlPOLPoint(MotoCtr.POLPerset, Senser.Angle_POL);
    if (IsEnable != 1) {
        MotoCtr.ESpeed = 0;
        MotoCtr.ROLLSpeed = 0;
        MotoCtr.POLSpeed = 0;
    }
}

/// <summary>
/// 圆扫自跟踪调试
/// </summary>
void CircleTrackDebug()
{
}

/// <summary>
/// 航向校准过程中，计算俯仰角
/// </summary>
/// <param name="ELg">俯仰指向角</param>
/// <param name="Pitch">纵摇</param>
/// <param name="Roll">横滚</param>
/// <param name="AZb">方位轴角</param>
/// <param name="ELb">俯仰轴角</param>
void GetElb_HeadCheck(float ELg, float Pitch, float Roll, float AZb, float* ELb)
{
    //俯仰指向角
    float elgRad = 0;
    //纵摇、横滚
    float pRad = 0, rRad = 0;
    //方位、俯仰轴角
    float azbRad = 0, elbRad = 0;

    //g系到b系角度矩阵
    float a = 0, b = 0, c = 0;

    elgRad = ELg * PI / 180.0f;

    azbRad = AZb * PI / 180.0f;

    pRad = Pitch * PI / 180.0f;
    rRad = Roll * PI / 180.0f;

    a = cos(pRad) * cos(rRad);
    b = sin(pRad) * cos(azbRad) - cos(pRad) * sin(rRad) * sin(azbRad);
    c = sin(elgRad);

    elbRad = asin(c / sqrt(a * a + b * b)) - atan(b / a);

    *ELb = elbRad * 180.0f / PI;
}

//计算得到当前航向
void GetHead_HeadCheck(float AZg, float ELg, float AZb, float Pitch, float Roll, float* Head)
{
    float agRad, egRad;
    float xg, yg, a, p, r, h, o, B, m, n;
    const float pi = 3.141592653;

    agRad = AZg * pi / 180.0f;
    egRad = ELg * pi / 180.0f;

    a = AZb * pi / 180.0f;
    p = Pitch * pi / 180.0f;
    r = Roll * pi / 180.0f;

    o = 30 * pi / 180.0f;

    xg = cos(egRad) * sin(agRad);
    yg = cos(egRad) * cos(agRad);

    m = yg * (sin(o) * sin(r) + sin(a) * cos(o) * cos(r)) - xg * (cos(a) * cos(o) * cos(p) - cos(r) * sin(o) * sin(p) + sin(a) * cos(o) * sin(p) * sin(r));
    n = xg * (sin(o) * sin(r) + sin(a) * cos(o) * cos(r)) + yg * (cos(a) * cos(o) * cos(p) - cos(r) * sin(o) * sin(p) + sin(a) * cos(o) * sin(p) * sin(r));

    B = atan(m / n);
    B = B * 180.0f / pi;

    if (m > 0) {
        if (n > 0) {
            h = -1 * B + 360;
        } else {
            h = -1 * B + 180;
        }
    } else {
        if (n > 0) {
            h = -1 * B;
        } else {
            h = -1 * B + 180;
        }
    }

    *Head = h;
}
