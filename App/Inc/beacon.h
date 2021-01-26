#ifndef __BEACON_H__
#define __BEACON_H__

#include "stdint.h"

#define SATEMODE_DVB 0
#define SATEMODE_DC 1
#define SATEMODE_BC 2

#define STATE_OK 0
#define STATE_NOK 1

#define BC_FREQ 0 //信标参数
#define BC_FEED 1 //馈电
#define BC_MONO 2 //22k单音
#define BC_MODE 3 //接收模式

void BC_Set22K(uint8_t sw);
void BC_SetFeed(uint8_t vo);
void BC_SetFreq(uint32_t freq);
void BC_SetMode(uint8_t mode);
void BC_SetLocalSel(uint8_t localSel);
void BC_SetCapRange(uint8_t range);
void BC_SetRefreshRate(uint8_t rate);
void BC_SetVMapping(uint8_t sw);
void BC_SetVMappingRange(uint32_t min, uint32_t max);

void BC_SetDetection(uint32_t freq, uint32_t sym, uint32_t rng, uint8_t roll); //设置检波参数

void BC_Get22K(void);
void BC_GetCapRange(void);
void BC_GetFeed(void);
void BC_GetFreq(void);
void BC_GetMode(void);
void BC_GetRefreshRate(void);
void BC_GetVMapping(void);
void BC_GetVMappingRange(void);

void BC_GetBCInfo(void);
void BC_SetBCInfo(void);
void BC_SetTrState(uint8_t pos, uint8_t isOk);
uint8_t BC_IsNeedSet(void);

#endif //__BEACON_H__
