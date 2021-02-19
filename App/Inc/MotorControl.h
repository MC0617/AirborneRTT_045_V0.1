#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "MainDefine.h"
#include "Parameters.h"
#include "math.h"

// 电机控制变量结构体
typedef struct _PID {
    float P; //比例变量
    float I; //积分变量
    float D; //微分变量
    float K; //前馈变量
    float ValueBeforeL; //加速度限制前的控制量
    float Value; //最终控制量
    float Value_Last; //上一次的控制量

    float err; //本次误差
    float err_last; //上次误差
    float err_last_before; //上上次误差

    float integral; //积分值

    unsigned int count; //轨迹规划计数器
    float startPosition; //开始指向时天线的位置

    float LookHead; //最终五点外推值
    float LookHeadBeforeL; //进行限制前的五点外推值

    float PLast;

} PID;

typedef struct _PWMFRE {
    unsigned long int az;
    unsigned long int el;
} PWMFRE;

// 指向启动时平滑处理计数器
typedef struct _TIMER {
    unsigned short int az; //方位指向启动计时器
    unsigned short int el; //俯仰指向启动计时器
    unsigned short int roll; //横滚指向启动计时器
    unsigned short int pol; //极化指向启动计时器

} TIMER;

typedef struct _TIMPERIOD {
    unsigned long int az; //方位PWM定时器周期
    unsigned long int el; //俯仰PWM定时器周期
} TIMPERIOD;

// 电机控制变量结构体
typedef struct _MOTOCTR {
    float AZPerset;
    float ELPerset;
    float ROLLPerset;
    float POLPerset;
    float AZP[301]; //方位预置集合
    float AZP_[301]; //方位预置集合，用于解决0|360切换
    float ELP[301]; //俯仰预置集合
    float ROLLP[301]; //横滚预置集合，用于五点外推
    float POLP[301]; //极化预置集合，用于五点外推

    float AZgPerset;
    float ELgPerset;
    float PolgPerset;

    float AZSpeed; //方位速度预置
    float ESpeed; //俯仰速度预置
    float ROLLSpeed; //极化速度预置
    float POLSpeed; //波导切换装置角速度预置

    float ASpeddLast;
    float ESpeddLast;
    float PSpeddLast;

    float AZCenter;
    float ELCenter;
    float ROLLCenter;
    float PolRxCenter;
    //float PolCenter;

    int PeriodCountTotal;
    int PeriodCountSinAZ;
    int PeriodCountSinEL;
    int PeriodCountSinROLL;
    int PeriodCountSinPRx;

    int AZEnalbe; //方位使能：1=使能开；0=使能关
    int ELEnalbe; //俯仰使能：1=使能开；0=使能关
    int ROLLEnalbe; //极化使能：1=使能开；0=使能关
    int POLEnalbe; //极化使能：1=使能开；0=使能关

    PID PIDAZ;
    PID PIDEL;
    PID PIDROLL;
    PID PIDPOL;

    float ValueP_Pol; //极化比例变量
    float ValueI_Pol; //极化积分变量
    float ValueD_Pol; //极化微分变量
    float Err_Pol; //极化角度差

    //	float AZP_LookHead;	//五点外推结果，先行的预置角
    //	float ELP_LookHead;	//五点外推结果，先行的预置角

    float AZP_StandBy; //待机时方位预置角度

    float angleAdjustAZ; //方位调整角度(空间角)
    float angleAdjustEL; //俯仰调整角度(空间角)

    TIMER Timer; //计时器，用于指向时的启动平滑处理

    int CircleTrackDebugStep; //圆扫自跟踪调试状态

    PWMFRE PWMFre;

    TIMPERIOD TIMPeriod;
} MOTOCTR;
extern MOTOCTR MotoCtr;

// 对参数进行范围限制
// <param name="val">需要进行范围限制参数</param>
// <param name="LMT">限制范围</param>
// <return>当数据处于±LMT范围内返回数据，当超出范围则返回范围边界</return>
float uLimit(float val, float LMT);

// 方位位置环PID，跟踪
float PositionControlAZ(float PstnPerset, float* PstnReal);

// 俯仰位置环PID，跟踪
float PositionControlEL(float PstnPerset, float* PstnReal);

// 横滚位置环PID
float PositionControlROLL(float PstnPerset, float* PstnReal);

// 极化位置环PID
float PositionControlPOL(float PstnPerset, float* PstnReal);

// 方位位置环PID，指向
float PositionControlAZPoint(float PstnPerset, float* PstnReal);

// 俯仰位置环PID，指向
float PositionControlELPoint(float PstnPerset, float* PstnReal);

// 横滚位置环PID，指向
float PositionControlROLLPoint(float PstnPerset, float* PstnReal);

// 极化位置环PID，用于指向
float PositionControlPOLPoint(float PstnPerset, float* PstnReal);

//将角度转换为±180°范围内
float _180convert(float angle);

//将角度转换为±10°范围内
float _10convert(float angle);

//方位电机使能
void MotoEnableAZ(void);

//方位电机去使能
void MotoDisableAZ(void);

//方位电机角速度控制
void SpeedCtrAZ(float speed);

//俯仰电机使能
void MotoEnableEL(void);

//俯仰电机使能
void MotoDisableEL(void);

//俯仰电机角速度控制
void SpeedCtrEL(float speed);

//横滚电机使能
void MotoEnableROLL(void);

//横滚电机去使能
void MotoDisableROLL(void);

//横滚电机角速度控制
void SpeedCtrROLL(float speed);

//极化电机角速度控制
void SpeedCtrPOL(float speed);

//极化电机使能
void MotoEnablePOL(void);

//极化电机去使能
void MotoDisablePOL(void);

// 速度控制
void SpeedCtrl(void);

// 电机使能
void MotoInit(void);

#endif
