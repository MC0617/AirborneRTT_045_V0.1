#ifndef MAINDEFINE_H_
#define MAINDEFINE_H_

#include "CoordinateConvert.h"
#include "DataAnalysis.h"
#include "Message.h"
#include "MotorControl.h"
#include "Parameters.h"
#include "Senser.h"
#include "math.h"

void Delayus(uint16_t Time); //延时

// 工作指示灯控制，闪烁，周期1s
void PilotLampControl(void);

// 待机模式
void StandBy(void);

// 方位寻零
void AZFindZero(void);
// 俯仰寻零
void ELFindZero(void);
// 横滚寻零
void ROLLFindZero(void);
// 极化寻零
void POLFindZero(void);

//初始航向校准
void HeadCalibrate(void);

// 存储AGC信号最大值及其对应的空间位置
void StoreAgcMax(void);

// 指向目标卫星
void AimSatellite(void);

// 计算天线指向参考卫星的角度
// <param IsEnable>0:无效,仅做计算,天线不转动;1:有效,天线转动</param IsEnable>
void AimSateOfNorm(unsigned char IsEnable);

// 搜索扫描，搜索到AGC最大值的方位、俯仰角度（地理坐标系下）
// 当信标接收机锁定且AGC最大值大于跟踪阈值切入自跟踪模式；
// 若AGC最大值小于跟踪阈值则切到初始对星模式
// <param name="AZg">方位角，地理坐标系</param>
// <param name="ELg">俯仰角，地理坐标系</param>
void SearchAGCMax(float AZg, float ELg);

// 搜索扫描，搜索到AGC最大值的方位、俯仰角度（地理坐标系下）
// 当信标接收机锁定且AGC最大值大于跟踪阈值切入自跟踪模式；
// 若AGC最大值小于跟踪阈值则切到初始对星模式
// <param name="AZCenter">方位角，地理坐标系</param>
// <param name="ELCenter">俯仰角，地理坐标系</param>
// <param name="azb">方位角，载体坐标系</param>
// <param name="elb">俯仰角，载体坐标系</param>
void SearchCircle(float AZCenter, float ELCenter, float* AZb, float* ELb);

// 程序引导
void Leading(void);

// 速度环调试
void DebugSpeed(void);

// 位置环调试
void DebugPosition(void);

// 速度环正弦调试
void DebugSpeedSin(void);

// 位置环正弦调试
void DebugPositionSin(void);

// 圆扫自跟踪调试
void CircleTrackDebug(void);

// 五点外推算法
// <param name="PastGather">过去数据集合</param>
// <param name="LookHeadTime">先行时间，单位ms</param>
// <param name="FREQ">控制频率</param>
// <param name="Index">最新数据索引</param>
float Extrapolate(float* PastGather, int LookHeadTime, int FREQ, int Index);

// 方位位置环控制
// <param name="PPerset">预置角</param>
void AZControl(float PPerset);

// 俯仰位置环控制

// <param name="PPerset">预置角</param>
void ELControl(float PPerset);

// 计算预置与当前误差

// <param name="PPersetAZ">方位预置角</param>
// <param name="PPersetEL">俯仰预置角</param>
// <param name="ErrAZ">方位误差角</param>
// <param name="ErrEL">俯仰误差角</param>
void GetAngleErr(float PPersetAZ, float PPersetEL, float* ErrAZ, float* ErrEL);

// 计算实际角度
void GetAngleReal(void);

// 圆扫自跟踪
void TrackCircle(void);

// 寻零
void ZeroCheck(void);

#endif /* MAINDEFINE_H_ */
