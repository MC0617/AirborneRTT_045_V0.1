#include "thread_com.h"

#include "lwip.h"
#include "rtthread.h"
#include "udp_echoserver.h"

#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 512
#define THREAD_TIMESLICE 5

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
        rt_thread_mdelay(4);
    }
}

void ComInit()
{
    rt_err_t result;
    result = rt_thread_init(&tid_com, "thread1", thread_com, RT_NULL, thread_stack, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    /* 启动线程 */
    if (result == RT_EOK) {
        rt_thread_startup(&tid_com);
    }
}
