#include "thread_ctrl.h"

#include "main.h"
#include "rtthread.h"
#include "stdint.h"
#include "udp_echoserver.h"

#define THREAD_PRIORITY 4
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 5

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t thread_stack[THREAD_STACK_SIZE];
static struct rt_thread tid_ctrl;
rt_timer_t timer_mems;

/* 事件控制块 */
struct rt_event event_mems;

static void thread_ctrl(void* parameter)
{
    rt_uint32_t e;
    while (1) {
        /* 第一次接收事件，事件3或事件5任意一个可以触发线程1，接收完后清除事件标志 */
        if (rt_event_recv(&event_led, (EVENT_FLAG_PCRECV | EVENT_FLAG5), RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e) == RT_EOK) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
    }
}

static void timeout_mems(void* parameter)
{
    rt_event_send(&event_mems, EVENT_FLAG_TIMEOUT5);
}

void CtrlInit()
{
    rt_err_t result;
    result = rt_thread_init(&tid_ctrl, "thread_ctrl", thread_ctrl, RT_NULL, thread_stack, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    /* 启动线程 */
    if (result == RT_EOK) {
        rt_thread_startup(&tid_ctrl);
    } else {
        SendMsgUdp((uint8_t*)"thread_ctrl err", sizeof("thread_ctrl err"));
    }

    result = rt_event_init(&event_mems, "event_mems", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK) {
        SendMsgUdp((uint8_t*)"event_mems err", sizeof("event_mems err"));
    }

    timer_mems = rt_timer_create("timer_mems", timeout_mems, RT_NULL, 50, RT_TIMER_FLAG_ONE_SHOT);
    // rt_timer_init(timer_mems, "timer_mems", timeout_mems, RT_NULL, 5, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);

    rt_timer_start(timer_mems);
}
