#include "thread_ctrl.h"

#include "main.h"
#include "rtthread.h"
#include "stdint.h"
#include "udp_echoserver.h"

#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 768
#define THREAD_TIMESLICE 5

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t thread_stack[THREAD_STACK_SIZE];
static struct rt_thread tid_ctrl;

//等待时间计数器
uint8_t Control_Period_Flag = 0;

static void thread_ctrl(void* parameter)
{
    rt_uint32_t e;
    while (1) {
        /* 第一次接收事件，事件3或事件5任意一个可以触发线程1，接收完后清除事件标志 */
        if (rt_event_recv(&event, (EVENT_FLAG3 | EVENT_FLAG5), RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e) == RT_EOK) {
        }
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

void CtrlInit()
{
    rt_err_t result;
    result = rt_thread_init(&tid_ctrl, "thread_ctrl", thread_ctrl, RT_NULL, thread_stack, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    /* 启动线程 */
    if (result == RT_EOK) {
        rt_thread_startup(&tid_ctrl);
    }
}
