#ifndef __THREAD_CTRL_H__
#define __THREAD_CTRL_H__

#include "rtthread.h"

#define EVENT_FLAG_PCRECV (1 << 3)
#define EVENT_FLAG5 (1 << 5)

#define EVENT_FLAG_MEMSRECV (1 << 6)
#define EVENT_FLAG_TIMEOUT5 (1 << 10)

/* 事件控制块 */
extern struct rt_event event_led;
extern struct rt_event event_mems;

void CtrlInit(void);

#endif // !__THREAD_CTRL_H__
