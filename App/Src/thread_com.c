#include "thread_com.h"

#include "lwip.h"
#include "rtthread.h"
#include "string.h"
#include "udp_echoserver.h"

#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 768
#define THREAD_TIMESLICE 5

#define EVENT_FLAG3 (1 << 3)
#define EVENT_FLAG5 (1 << 5)

/* 事件控制块 */
struct rt_event event;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t thread_stack[THREAD_STACK_SIZE];
static struct rt_thread tid_com;

unsigned char PCRecvCount;
unsigned char PCRecvBuff[PC_RX_BUFSIZE];
unsigned char PCRecvFlag;
unsigned char PCSendBuff[PC_TX_BUFSIZE];

static void thread_com(void* parameter)
{
    while (1) {
        MX_LWIP_Process();

        if (PCRecvFlag) {
            if (PCRecvBuff[0] == 0xEB && PCRecvBuff[1] == 0x90) {
                rt_event_send(&event, EVENT_FLAG3);
            } else if (PCRecvBuff[0] == 'B' && PCRecvBuff[1] == 'A') {
                rt_event_send(&event, EVENT_FLAG5);
            }
            // PCRecvFlag = 0;
            // memset(PCRecvBuff, 0, sizeof(PCRecvBuff));
        }
        rt_thread_mdelay(4);
    }
}

void ComInit()
{
    rt_err_t result;
    udp_echoserver_init();
    result = rt_thread_init(&tid_com, "thread_com", thread_com, RT_NULL, thread_stack, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    /* 启动线程 */
    if (result == RT_EOK) {
        rt_thread_startup(&tid_com);
    }

    /* 初始化事件对象 */
    result = rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK) {
        SendMsgUdp((uint8_t*)"event err", sizeof("event err"));
    }
}
