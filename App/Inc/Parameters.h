#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "stm32f4xx_hal.h"

#include "stdint.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"

#define _CTRCLK 200

#define Mode_Find_Zero 0 //寻灵
#define Mode_HeadCheck 1 //航向校准
#define Mode_Stand_By 2 //待机
#define Mode_Collection 3 //收藏
#define Mode_AimSatellite 4 //指向
#define Mode_Searching 5 //搜索
#define Mode_Tracking 6 //跟踪
#define Mode_Leading 7 //程序引导
#define Mode_Hand_Control 8 //手动控制
#define Mode_Debug_Speed 9 //速度环调试
#define Mode_Debug_Position 10 //位置环调试
#define Mode_Debug_Speed_Sin 11 //速度环正弦调试
#define Mode_Debug_Position_Sin 12 //位置环正弦调试

#define RecvMode_DVB 0
#define RecvMode_JianBo 1
#define RecvMode_Xinbiao 2

extern char AZFindZeroEndFlag; //方位寻零完成标志位;0:未完成,1:完成
extern char ELFindZeroEndFlag; //俯仰寻零完成标志位;0:未完成,1:完成
extern char ROLLFindZeroEndFlag; //横滚寻零完成标志位;0:未完成,1:完成
extern char POLFindZeroEndFlag; //极化寻零完成标志位;0:未完成,1:完成

extern char HeadCheckFlag; //航向校准完成标志位;0:未完成,1:完成

//工作模式
extern uint8_t WorkMode;

extern int PeriodPilotLamp; //指示灯控制计数器
extern int HandControlEnable; //手动控制使能;1:使能,0:去使能

extern int StandbyTimeCount; //待机减速计时器

// 卫星信息结构体
typedef struct _SATELLITEINFO {
    uint8_t IsStoreData; //是否存储当前卫星信息

    //卫星经度
    float longitude;

    //极化方式
    uint8_t polerMode;

    //接收模式 0:DVB, 1:检波, 2:信标
    uint8_t recvMode;

    //本振
    uint32_t localOsci;

    //接收频率
    uint32_t freq;

    //符号速率
    uint32_t symbolRate;

    //搜索范围
    uint32_t searchRange;

    //滚降系数
    uint8_t roll;
} SATELLITEINFO;
// 工作目标卫星信息
extern SATELLITEINFO SatellitesInfo;

//信标机设置结构体
typedef struct _BEACONINFO {
    uint8_t IsSet; //是否需要设置
    uint8_t mode; //接收模式 0:DVB 1:检波 2:信标
    uint32_t freq; //接收频率
    uint32_t symbolRate; //符号速率
    uint32_t range; //搜索范围
    uint8_t roll; //滚降系数
    uint8_t feed; //馈电 1:13.4, 2:18.2, 3:14.6, 4:19.4
    uint8_t mono; //22k单音 1:ON
    uint16_t LostCount; //丢帧计数
    uint16_t ErrCount; //错帧计数
    uint16_t BugTime; //丢/错帧时间
} BeaconInfo_t;
extern BeaconInfo_t BeaconInfo;

// 工作目标卫星信息
extern SATELLITEINFO SatellitesInfo;

// 寻零参数结构体
typedef struct _FINDZEROPARAMS {
    int PeriodCount;

    int State;

    float StartAZg; //起始AZg，用于航向校准
} FINDZEROPARAMS;
// 方位寻零参数
extern FINDZEROPARAMS AZFindZeroParams;
// 俯仰寻零参数
extern FINDZEROPARAMS ELFindZeroParams;
// 横滚寻零参数
extern FINDZEROPARAMS ROLLFindZeroParams;
// 极化寻零参数
extern FINDZEROPARAMS POLFindZeroParams;
// 航向校准参数
extern FINDZEROPARAMS HeadCheckParams;

// 极化寻零变量结构体
typedef struct _POLFINDZEROPARAMS {
    //周期计数器，用于记录寻零起始角度
    int PeriodCount;
    //寻零起始角度，用于判断转动大于5°后进行速度切换
    float PositionStart;
    //寻零状态机标志，用于状态机流程跳转
    //0:默认状态，初始化
    //1：状态1，出现于寻零起始过程，表现为顺时针低速转动
    //2：状态2，出现于寻零最后过程，表现为逆时针低速转动，发生于极化转动至硬限位后或顺时针过光电开关后，该状态为寻零最后状态
    //3：状态3，出现于寻零中间过程，表现为顺时针高速转动，发生于极化顺时针低速转动角度大于5°后
    //4：状态4，出现于寻零中间过程，表现为顺时针低速转动，发生于极化顺时针转动且转动角度小于5°内触发90°光电开关时，保持顺时针低速转动直至90°光电开关触发消失
    //5：状态5，出现于寻零结束过程，表现为电机停止转动，保持20ms后极化寻零结束
    //else 寻零已结束
    int State;
} POLFINDZEROPARAMS;
extern POLFINDZEROPARAMS PolFindZeroParams;

// 扫描搜索变量结构
typedef struct _SEARCHPARAMS {
    //搜索模式：0-Z扫;1-圆扫
    int SearchMode;

    //周期计数器，用于搜索路径计算
    int PeriodCount;

    //一个Step执行周期数
    int PeriodCountStepTotal;

    //搜索中心角，方位，地理坐标系
    float CenterAZ;

    //搜索中心角，俯仰，地理坐标系
    float CenterEL;

    //搜索范围，方位，Z扫表示总宽度， 圆扫表示直径
    float RangeAZ;

    //搜索范围，俯仰，Z扫表示总宽度， 圆扫表示直径
    float RangeEL;

    //搜索范围，方位，Z扫表示总宽度一半，圆扫表示半径
    float RangeAZHalf;

    //搜索范围，俯仰，Z扫表示总宽度一半，圆扫表示半径
    float RangeELHalf;

    //Z扫时搜索速度, 单位°/2ms
    float SearchSpeed;

    //圆扫周期
    float CircleTime;

    //圆扫步骤标志，用于搜索流程跳转
    //0:默认状态，初始化
    //1：步骤1，天线指向搜索起始指向
    //2：步骤2，周期2S大范围圆扫
    //3：步骤3，天线指向以第一次圆扫得到的AGC最大点为中心第二次圆扫起始点
    //4：步骤4，周期2S小范围圆扫
    //Z扫步骤标志，用于搜索流程跳转
    //0:默认状态，初始化
    //1:步骤1，天线指向搜索中心指向
    //2:步骤2，天线指向搜索起始指向
    //3:步骤3，俯仰不动，方位逆时针转5°
    //4:步骤4，方位不动，俯仰下转2.5°
    //5:步骤5，俯仰不动，方位顺时针转5°
    //6:步骤6，方位不动，俯仰下转2.5°
    //7:步骤7，俯仰不动，方位逆时针转5°
    //8:步骤8，天线指向搜索中心指向
    //9:步骤9，天线指向搜索起始指向
    //10:步骤10，俯仰不动，方位逆时针转2°
    //11:步骤11，方位不动，俯仰下转1°
    //12:步骤12，俯仰不动，方位顺时针转2°
    //13:步骤13，方位不动，俯仰下转1°
    //14:步骤14，俯仰不动，方位逆时针转2°
    int Step;

    long SearchTime; //搜索计数器，当超过时间，需要进行航向校准
} SEARCHPARAMS;
extern SEARCHPARAMS SearchParams;

typedef struct _DEBUGPARAMS {
    //最新设置参数编号
    int SetParamNo;

    //参数集合
    float Params[50];

    //0=不需要备份数据；1=需要备份数据
    int IsStore;
} DEBUGPARAMS;
extern DEBUGPARAMS DebugParams;

// 扫描搜索变量结构
typedef struct _TRACKPARAMS {
    //跟踪方式：0-Z扫;1-圆扫
    //int SearchMode;

    //周期计数器，用于搜索路径计算
    int PeriodCount;

    //圆扫一个周期的Count数
    int PeriodCountStepTotal;

    //搜索中心角，方位，地理坐标系
    float CenterAZ;

    //搜索中心角，俯仰，地理坐标系
    float CenterEL;

    //搜索范围，方位，Z扫表示总宽度， 圆扫表示直径
    float RangeAZ;

    //搜索范围，俯仰，Z扫表示总宽度， 圆扫表示直径
    float RangeEL;

    //搜索范围，方位，Z扫表示总宽度一半，圆扫表示半径
    //float RangeAZHalf;

    //搜索范围，俯仰，Z扫表示总宽度一半，圆扫表示半径
    //float RangeELHalf;

    //Z扫时搜索速度, 单位°/2ms
    //float SearchSpeed;

    //圆扫周期
    float CircleTime;

    //跟踪过程中AGC记录
    float AgcTracking[4];

    //圆扫步骤标志，用于搜索流程跳转
    //0:默认状态，初始化
    //1：步骤1，天线指向搜索起始指向
    //2：步骤2，周期2S大范围圆扫
    //3：步骤3，天线指向以第一次圆扫得到的AGC最大点为中心第二次圆扫起始点
    //4：步骤4，周期2S小范围圆扫
    //Z扫步骤标志，用于搜索流程跳转
    //0:默认状态，初始化
    //1:步骤1，天线指向搜索中心指向
    //2:步骤2，天线指向搜索起始指向
    //3:步骤3，俯仰不动，方位逆时针转5°
    //4:步骤4，方位不动，俯仰下转2.5°
    //5:步骤5，俯仰不动，方位顺时针转5°
    //6:步骤6，方位不动，俯仰下转2.5°
    //7:步骤7，俯仰不动，方位逆时针转5°
    //8:步骤8，天线指向搜索中心指向
    //9:步骤9，天线指向搜索起始指向
    //10:步骤10，俯仰不动，方位逆时针转2°
    //11:步骤11，方位不动，俯仰下转1°
    //12:步骤12，俯仰不动，方位顺时针转2°
    //13:步骤13，方位不动，俯仰下转1°
    //14:步骤14，俯仰不动，方位逆时针转2°
    //int Step;

    //用于计时，当跟踪启动100ms后，五点外推才有效,方位用
    int StartTimeCountAZ;
    //用于计时，当跟踪启动100ms后，五点外推才有效,俯仰用
    int StartTimeCountEL;

    //计时器，作为自跟踪与程序引导切换的参考
    //当AGC连续100ms低于阈值，由自跟踪切换为程序引导
    //当AGC连续100ms高于阈值，由程序引导切换为自跟踪
    int Timer;

    //计时器，作为自跟踪中判断信号是否持续跌落计时
    //当AGC连续1000ms低于信号最大值0.15V，扫描范围扩大为0.1°
    //当AGC不连续1000ms低于信号最大值0.15V，扫描范围变回0.05°
    int Timer2;

    //切换扫描范围标志位：0=0.05°；1=0.1°
    int RangeChangeFlag;

    float _value1, _value2, _value3, _value4; //圆扫过程中4个象限的AGC之和

    uint8_t AmendEffectiv; //惯导修正的有效性，当圆扫过程中出现瞬时遮挡，置该标志位为0，则此次圆扫修正无效

} TRACKPARAMS;
extern TRACKPARAMS TrackParams;

typedef struct _GDAMAND {
    //惯导数据
    float Value;

    //0=不需要修正；1=需要修正
    int IsAmand;

    uint8_t isFirst;
} GDAMAND;
extern GDAMAND GDAmand;

// 变量初始化
void ParamsInit(void);

// 读卫星信息
void ReadSatelliteInfo(void);

// 写卫星信息
void WriteSatelliteInfo(void);

// 写调试参数
void WriteDebugParam(void);

// 从FLASH中读取调试参数
void ReadDebugParams(void);

// 将数据存入Flash中
void FlashDataWrite(void);

#endif
