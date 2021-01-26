#ifndef COORDINATECONVERT_H_
#define COORDINATECONVERT_H_

#include "Parameters.h"

// 计算得到目标卫星的空间指向角
void GetSatelliteAngle(float* AZ, float* EL, float* Pol);

// 计算得到参考卫星对于天线的空间指向角及极化角
void GetAngleGOfNormalSate(float* AZ, float* EL, float* Pol);

/// <summary>
/// 获取极化角度，载体坐标系
/// </summary>
/// <param name="AZb">方位角，载体坐标系</param>
/// <param name="ELb">俯仰角，载体坐标系</param>
/// <param name="Head">载体姿态，航向角</param>
/// <param name="Pitch">载体姿态，纵摇角</param>
/// <param name="Roll">载体姿态，横滚角</param>
/// <param name="Pol">极化角，载体坐标系</param>
void GetPol(float AZb, float ELb, float Head, float Pitch, float Roll, float* Pol);

/// <summary>
/// 坐标系转换，载体坐标系->地理坐标系
/// </summary>
/// <param name="AZb">方位角，载体坐标系</param>
/// <param name="ELb">俯仰角，载体坐标系</param>
/// <param name="Head">载体姿态，航向角</param>
/// <param name="Pitch">载体姿态，纵摇角</param>
/// <param name="Roll">载体姿态，横滚角</param>
/// <param name="AZg">方位角，地理坐标系</param>
/// <param name="ELg">俯仰角，地理坐标系</param>
void NewConvertBTOG(float AZb, float ELb, float Head, float Pitch, float Roll, float* AZg, float* ELg);

// 惯导跟随方位转动，通过空间指向计算天线坐标系方位预置角
void NewConvertGTOB(float AZg, float ELg, float Head, float Pitch, float Roll, float azbNow, float* AZb, float* ELb);

//三轴天线空间坐标系到天线坐标系角度计算
//注：交叉安装轴与方位转盘承120°
//惯导安装于天线底座，不跟随方位转动
void GtoB(float azg, float elg, float head, float pitch, float roll, float* A, float* E, float* C);
//计算得到空间指向
void BtoG(float azb, float elb, float crb, float head, float pitch, float roll, float* azg, float* elg);

// 惯导安装于方位转台，通过当前空间指向及编码器数据计算得到当前惯导航向
void GetRealHead(float AZg, float ELg, float AZb, float ELb, float Pitch, float Roll, float* Head);
//计算得到方位转角，输入参数为弧度制
float GetA(float agRad, float egRad, float headRad, float pitchRad, float rollRad);
//计算得到方位转角，输入参数为角度制
float GetA_(float ag, float eg, float head, float pitch, float roll);
//计算得到交叉转角
float GetC(float azbRad, float pitchRad, float rollRad);
//计算得到俯仰转角
float GetE(float egRad, float abRad, float cRad, float pitchRad, float rollRad);
//计算得到当前航向角
float GetH(float AZg, float ELg, float AZb, float Pitch, float Roll);

#endif /* COORDINATECONVERT_H_ */
