/*
 * Parameters.h
 *
 *  Created on: 2016-6-12
 *      Author: Administrator
 */

#ifndef PARAMETERSDEFINE_H_
#define PARAMETERSDEFINE_H_

#include "Parameters.h"

//Encoder Offset AZ
#define OFFSETAZ DebugParams.Params[0]
//Encoder Offset EL
#define OFFSETEL DebugParams.Params[1]
//Encoder Offset POL(Rx)
#define OFFSETROLL DebugParams.Params[2]
//Encoder Offset POL(Tx)
#define OFFSETPOL DebugParams.Params[3]
//Calibrate the installation error of Mems
#define OFFSETPITCH DebugParams.Params[4]
//Calibrate the installation error of Mems
#define OFFSETMEMSROLL DebugParams.Params[5]

//PID AZ Position Control
#define P_AZ DebugParams.Params[6]
#define I_AZ DebugParams.Params[7]
#define D_AZ DebugParams.Params[8]
#define F_AZ DebugParams.Params[9]
#define LHT_AZ DebugParams.Params[10]

//PID EL Position Control
#define P_EL DebugParams.Params[11]
#define I_EL DebugParams.Params[12]
#define D_EL DebugParams.Params[13]
#define F_EL DebugParams.Params[14]
#define LHT_EL DebugParams.Params[15]

//PID ROLL Position Control
#define P_ROLL DebugParams.Params[16]
#define I_ROLL DebugParams.Params[17]
#define D_ROLL DebugParams.Params[18]
#define K_ROLL DebugParams.Params[19]
#define LHT_ROLL DebugParams.Params[20]

//PID POL Position Control
#define P_POL DebugParams.Params[21]
#define I_POL DebugParams.Params[22]
#define D_POL DebugParams.Params[23]
#define K_POL DebugParams.Params[24]
#define LHT_POL DebugParams.Params[25]

//Circle_Track Params
#define CIRCLERANGEAZ DebugParams.Params[26]
#define CIRCLERPERIODAZ DebugParams.Params[27]
#define CIRCLERSTEPAZ DebugParams.Params[28]

#define SWITCH_8 ((int)(DebugParams.Params[32]))
//圆扫开关
#define CIRCLE_S ((SWITCH_8 >> 0) & 0x01)
//信标机波特率
#define XBBAUD_S ((SWITCH_8 >> 1) & 0x01)
//LNB本振标识
#define LOCOSC_S ((SWITCH_8 >> 2) & 0x01)
//初始航向校准自动启动标志
#define AUTOFS_S ((SWITCH_8 >> 3) & 0x01)
//初始航向校准完成后进入的工作状态
#define AUTOTK_S ((SWITCH_8 >> 4) & 0x01)

#define MEMSIN_S ((SWITCH_8 >> 5) & 0x01)

//Other Params
//Speed Loop Sin Range
#define VSINRANGE DebugParams.Params[33]
//Speed Loop Sin Period
#define VSINPERIOD DebugParams.Params[34]
//Position Loop Sin Range
#define SSINRANGE DebugParams.Params[35]
//Position Loop Sin Period
#define SSINPERIOD DebugParams.Params[36]
//Default msg type
#define DEFAULTMSGTYPE DebugParams.Params[37]

#define CIRCLERANGEEL DebugParams.Params[38]
#define CIRCLERPERIODEL CIRCLERPERIODAZ
#define CIRCLERSTEPEL DebugParams.Params[42]
////////////////////////////////////////////////

////////////////////////////////////////////
//参考卫星经度
//#define NormSateLongitude DebugParams.Params[39]
////参考卫星载波频率
//#define NormStaeFre 0
////参考卫星符码率
//#define NormSateSr 0
//参考卫星信标频率
//#define NormSateXinBiao DebugParams.Params[44]
//////////////////////////////////////////////

//////////////////////////////////////////////
//交叉轴角水平方向误差
#define CROSS_ERR DebugParams.Params[44]

//////////////////////////////////////////////

//默认上电接收机工作模式
#define RecvModeDefault DebugParams.Params[43]

/////////////////////////////////////////////
#define ACCMAXAZ DebugParams.Params[45]

//////////////////////////
#define OFFSETO DebugParams.Params[39]
#define TRACKMSGPERIOD DebugParams.Params[46]
//////////////////////////

#define ELBUCHANGA DebugParams.Params[47]
#define ELBUCHANGB DebugParams.Params[48]

//俯仰补偿参数
//线性补偿标识,0:线性;1:分段
#define LINEARFLAG DebugParams.Params[29]
//分段补偿上阈值
#define THRESHOLDUP DebugParams.Params[30]
//上阈值补偿值
#define VALUEUP DebugParams.Params[31]
//下阈值
#define THRESHOLDDOWN DebugParams.Params[40]
//下阈值补偿值
#define VALUEDOWN DebugParams.Params[41]

#define AZInc 0

#endif /* PARAMETERS_H_ */
