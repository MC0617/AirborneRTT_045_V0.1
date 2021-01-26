#ifndef __THREAD_CTRL_H__
#define __THREAD_CTRL_H__

#define EVENT_FLAG3 (1 << 3)
#define EVENT_FLAG5 (1 << 5)

/* 事件控制块 */
extern struct rt_event event;

void CtrlInit(void);

#endif // !__THREAD_CTRL_H__
