/**
  ******************************************************************************
  * @file    udp_echoserver.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "udp_echoserver.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

struct udp_pcb currUpcb;
ip_addr_t currAddr;
u16_t currPort;

void saveCurrConnect(struct udp_pcb* upcb, const ip_addr_t* addr, u16_t port)
{
    memcpy(&currUpcb, upcb, sizeof(struct udp_pcb));
    memcpy(&currAddr, addr, sizeof(ip_addr_t));
    currPort = port;
}

void initConnect(struct udp_pcb* upcb)
{
    IP4_ADDR(&currAddr, PC_IP_ADDR0, PC_IP_ADDR1, PC_IP_ADDR2, PC_IP_ADDR3);
    memcpy(&currUpcb, upcb, sizeof(struct udp_pcb));
    currPort = UDP_CLIENT_PORT;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void udp_echoserver_receive_callback(void* arg, struct udp_pcb* upcb, struct pbuf* p, const ip_addr_t* addr, u16_t port);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void udp_echoserver_init(void)
{
    struct udp_pcb* upcb;
    err_t err;

    /* Create a new UDP control block  */
    upcb = udp_new();

    if (upcb) {
        /* Bind the upcb to the UDP_PORT port */
        /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
        err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

        if (err == ERR_OK) {
            /* Set a receive callback for the upcb */
            udp_recv(upcb, &udp_echoserver_receive_callback, NULL);
            initConnect(upcb);

        } else {
            udp_remove(upcb);
            printf("can not bind pcb");
        }
    } else {
        printf("can not create pcb");
    }
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */

void udp_echoserver_receive_callback(void* arg, struct udp_pcb* upcb, struct pbuf* p, const ip_addr_t* addr, u16_t port)
{
    uint32_t len;
    uint8_t buf[PC_RX_BUFSIZE] = { 0 };
    struct pbuf* pudp_buf = p;

    //接收处理
    if (p->len > PC_RX_BUFSIZE)
        len = PC_RX_BUFSIZE;
    else
        len = p->len;

    memcpy(buf, p->payload, len);

    if (buf[0] == 0xEB && buf[1] == 0x90) {
        len = 24;
        saveCurrConnect(upcb, addr, port);
    } else if (buf[0] == 0xAA && buf[1] == 0x55) {
        len = buf[3] + 6;
        saveCurrConnect(upcb, addr, port);
    } else {
        memset(buf, 0, sizeof(buf));
        pbuf_free(p);
        return;
    }

    PCRecvFlag = 1;
    memcpy(PCRecvBuff, buf, len);
    pbuf_free(p);
    return ;

    /* Connect to the remote client */
    udp_connect(upcb, addr, UDP_CLIENT_PORT);

    /* Tell the client that we have accepted it */
    udp_send(upcb, p);

    /* free the UDP connection, so we can accept new clients */
    udp_disconnect(upcb);

    /* Free the p buffer */
    pbuf_free(p);
}

void SendMsgUdp(uint8_t* buff, uint32_t len)
{
    struct pbuf* pudp_buf;

    pudp_buf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (pudp_buf) {
        // pbuf_take(pudp_buf, buff, len);
        memcpy(pudp_buf->payload, buff, len);
        udp_sendto(&currUpcb, pudp_buf, &currAddr, currPort);
        pbuf_free(pudp_buf);
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
