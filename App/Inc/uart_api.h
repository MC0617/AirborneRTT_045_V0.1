/*
  ******************************************************************************
  * @file    Project/Template/usart.h
  * @author  liuchuang
  * @version V1.0.0
  * @date    18/08/2011
  * @brief   usart program body
  ******************************************************************************
*/
#include "stm32f4xx.h"

#ifndef __UART_API_H
#define __UART_API_H

#define USART1_DR_Base 0x40011004 //串口1的映射地址
#define USART2_DR_Base 0x40004404 //串口2的映射地址
#define USART3_DR_Base 0x40004804 //串口3的映射地址
#define UART4_DR_Base 0x40004c04 //串口4的映射地址
#define UART5_DR_Base 0x40005004 //串口5的映射地址

#define UART1_TX_BUF_SIZE 256
#define UART1_RX_BUF_SIZE 256
#define UART2_TX_BUF_SIZE 256
#define UART2_RX_BUF_SIZE 256
#define UART3_TX_BUF_SIZE 256
#define UART3_RX_BUF_SIZE 256
#define UART4_TX_BUF_SIZE 256
#define UART4_RX_BUF_SIZE 256
#define UART5_TX_BUF_SIZE 256
#define UART5_RX_BUF_SIZE 256

#define UART1_DMA_TXBUFSIZE 256 //串口1利用DMA待发送数据缓冲大小
#define UART2_DMA_TXBUFSIZE 256 //串口2利用DMA待发送数据缓冲大小
#define UART3_DMA_TXBUFSIZE 256 //串口3利用DMA待发送数据缓冲大小
#define UART4_DMA_TXBUFSIZE 256 //串口4利用DMA待发送数据缓冲大小
#define UART5_DMA_TXBUFSIZE 256 //串口5利用DMA待发送数据缓冲大小

#define UART1_DMA_RXBUFSIZE 256 //串口1DMA接收Buffer大小
#define UART2_DMA_RXBUFSIZE 256 //串口2DMA接收Buffer大小
#define UART3_DMA_RXBUFSIZE 256 //串口3DMA接收Buffer大小
#define UART4_DMA_RXBUFSIZE 256 //串口4DMA接收Buffer大小
#define UART5_DMA_RXBUFSIZE 256 //串口5DMA接收Buffer大小

#define COMMAND_BUF_SIZE 256

//#define Indexto232                1	//对用户232接口的串口编号
//#define IndextoDsp                2			//对内部DSP接口的串口编号
//#define Indexto422                4  	//对用户422接口的串口编号

#pragma pack(1)

typedef struct
{
    short uGpsWeek;
    double dGpsSecond;
    double fHeading;
    double fPitch;
    double fTrack;
    double dLatitude;
    double dLongitude;
    double fAltitude;
    double fVe;
    double fVn;
    double fVu;
    double fVg;
    double fAe;
    double fAn;
    double fAu;
    double fBaseline;
    uint8_t uNsv1;
    uint8_t uNsv2;
} GPHPD_Date;

typedef struct
{
    short uGpsWeek;
    double dGpsSecond;
    double fOffSet;
    double fUtcoffset;
    double fYear;
    double fMonth;
    double fDate;
    double fHour;
    double fMinite;
    double fSecond;
    double fMsecond;
} GPTIMEA_Date;

typedef struct
{
    double dGpsSecond;
    //uint16_t RxStatus;;
    double dLatitude;
    double dLongitude;
    double fAltitude;
    uint8_t uNsv;
} GPBESTPOSA_Date;

typedef struct
{
    double dGpsSecond;
    double fTrack;
    double fVe;
    double fVn;
    double fVu;
    double fVg;
} GPBESTVELA_Date;

typedef struct
{
    double fHead;
    double fBaseline;
    double fPitch;
    uint8_t uNsv;
    double second;
} GPHEADINGA_Date;

#pragma pack()
/* 全局参数 */
typedef struct
{
    USART_TypeDef* uart; /*  */
    uint8_t* pTxBuf; /* 发送缓冲区 */
    uint8_t* pRxBuf; /* 接收缓冲区 */
    uint16_t usTxBufSize; /* 发送缓冲区大小 */
    uint16_t usRxBufSize; /* 接收缓冲区大小 */
    uint16_t usTxWrite; /* 发送缓冲区写指针 */
    uint16_t usTxRead; /* 发送缓冲区读指针 */
    uint16_t usRxWrite; /* 接收缓冲区写指针 */
    uint16_t usRxRead; /* 接收缓冲区读指针 */
    void (*SendOver)(void); /* 发送完毕的回调函数指针 */
    void (*ReciveNew)(void); /* 串口收到数据的回调函数指针 */
} UART_T;

/* 定义端口号 		  */
typedef enum {
    COM1 = 0, // COM1口, RS232
    COM2 = 1, // COM2口, RS232
    COM3 = 2, // COM3口, RS485
    COM4 = 3,
    COM5 = 4,
    COM6 = 5
} PORT_NUM;

typedef enum {
    START_CMD = 0, // COM1口, RS232
    SLAVE_ADDR_CMD = 1,
    MASTER_ADDR_CMD = 2,
    FIRST_LETTER_CMD = 3,
    SECOND_LETTER_CMD = 4,
    LAST_LETTER_CMD = 5,
    END_CMD = 6
} CMDSTATE;

/* ==================== extern variable ==================== */

#define PC_RX_BUFSIZE 26
#define PC_TX_BUFSIZE 400

extern unsigned char PCRecvCount;
extern unsigned char PCRecvBuff[PC_RX_BUFSIZE];
extern unsigned char PCRecvFlag;
extern unsigned char PCSendBuff[PC_TX_BUFSIZE];

extern unsigned char DVBRecvBuffLenth;
extern unsigned char DVBRecvBuff[];
extern unsigned char DVBRecvFlag;
extern unsigned char DVBSendBuff[];

extern uint8_t BUC_RecvLength;
extern uint8_t BUC_RecvCount;
extern uint8_t BUC_RecvBuff[50];
extern uint8_t BUC_RecvFlag;

extern volatile int TX_Completed;
extern volatile int RX_Completed;
extern uint8_t USART1_GetBuffer[15];
extern uint8_t USART1_SendBuffer[102];

extern uint8_t USART2_GetBuffer[15];
extern uint16_t USART2_get_data_length;
extern uint16_t USART2_send_data_length;
extern uint8_t MemsSendBuff[30];

extern volatile int IN_TX_Completed;
extern volatile int IN_RX_Completed;
extern uint8_t IN_SendBuffer[24];
extern uint32_t MemsSendBufferDMA[10];

extern uint16_t IN_get_data_length;
extern uint16_t IN_send_data_length;

extern volatile int BC_TX_Completed;
extern volatile int BC_RX_Completed;
extern uint8_t BC_GetBuffer[24];

extern uint16_t BC_get_data_length;
extern uint16_t BC_send_data_length;

//extern double Sbaseline;
//extern double Heading_fixerr;
//extern double Pitch_fixerr;
//extern double Mode;
extern uint8_t CMD_HandlerBuf[256];

extern uint8_t UART1_RxDMAbuf[UART1_DMA_RXBUFSIZE]; //USART1 DMA接收FIFO
extern uint8_t UART2_RxDMAbuf[UART2_DMA_RXBUFSIZE]; //USART2 DMA接收FIFO
extern uint8_t UART3_RxDMAbuf[UART3_DMA_RXBUFSIZE]; //USART3 DMA接收FIFO
extern uint8_t UART4_RxDMAbuf[UART4_DMA_RXBUFSIZE]; //USART4 DMA接收FIFO
extern uint8_t UART5_RxDMAbuf[UART5_DMA_RXBUFSIZE]; //USART5 DMA接收FIFO

extern uint8_t UART1_TxDMAbuf[UART1_DMA_TXBUFSIZE]; //USART1 DMA发送数组
extern uint8_t UART2_TxDMAbuf[UART2_DMA_TXBUFSIZE]; //USART2 DMA发送数组
extern uint8_t UART3_TxDMAbuf[UART3_DMA_TXBUFSIZE]; //USART3 DMA发送数组
extern uint8_t UART4_TxDMAbuf[UART4_DMA_TXBUFSIZE]; //USART4 DMA发送数组
extern uint8_t UART5_TxDMAbuf[UART5_DMA_TXBUFSIZE];

extern UART_T g_tUart1;
extern UART_T g_tUart2;
extern UART_T g_tUart3;

extern uint8_t CMD_State;
extern uint16_t p_CMDStart;

extern uint32_t USART1DMASendBuff[];
extern const uint32_t USART1DMARecvLenth;
extern uint32_t USART1DMARecvBuff[];

extern uint32_t USART2DMASendBuff[100];
extern const uint32_t USART2DMARecvLenth;
extern uint32_t USART2DMARecvBuff[];

extern uint32_t USART3DMASendBuff[];
extern const uint32_t USART3DMARecvLenth;
extern uint32_t USART3DMARecvBuff[];

extern uint32_t USART6DMASendBuff[100];
extern uint32_t USART7DMASendBuff[6];

extern uint32_t UART4DMASendBuff[24];

extern const uint32_t USART6DMARecvLenth;
extern uint32_t USART6DMARecvBuff[];
extern uint32_t MemsData[6];

void USART1_Test(void);
void USART1_DMA_Test(void);
void USART2_DMA_Test(void);
extern uint8_t Judge_Command(const int16_t FIFOIndex, const int16_t DataBufLen);

void SendDMAData(int16_t Comport); //数据从 DMA buffer发送出去

extern int16_t UART_Read(const int16_t ComPort); //从串口DMA读取数据到接受FIFO
void UartVarInit(void); //串口FIFO初始化函数
void SendCom(int16_t Port, uint8_t* ByteBuf, int16_t BufLen); //串口数组发送函数

void RS485_TX_Mode(void);
void RS485_RX_Mode(void);

// DMA中断接收数据处理完成后，需要调用该函数重新使能DMA接收

void USART1_DMA_Get_Enable(void);

// 通过DMA方式发送数据
// <param name="Send_Count">需要发送的字节数</param>
void USART1_DMA_Send_Enable(uint16_t Send_Count);

// DMA中断接收数据处理完成后，需要调用该函数重新使能DMA接收
void USART2_DMA_Get_Enable(void);

// 通过DMA方式发送数据
// <param name="Send_Count">需要发送的字节数</param>
void USART2_DMA_Send_Enable(uint16_t Send_Count);

//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//ndtr:数据传输量
void DmaSendDataProc(DMA_Stream_TypeDef* DMA_Streamx, uint16_t ndtr);

//DMA方式串口发送数据
//stm: DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//dst: 串口DMA内存基地址（串口数组）
//src: 需要传输数据
//len: 数据传输量
void DmaSendData(DMA_Stream_TypeDef* stm, uint32_t* dst, uint8_t* src, uint16_t len);

enum A_Group_def {
    A_Group_1 = 1,
    A_Group_2,
    A_Group_3,
    A_Group_None
};
struct upper_test_def {
    uint8_t A_Test_1[8];
    uint8_t A_Test_2[8];
    uint8_t A_Test_3[8];
    uint8_t A_Send[8];
    uint8_t C_Test[8];
    uint8_t D_Test[8];
    uint8_t D_End[8];
    uint8_t D_End_Flag;
    uint8_t E_Test_1[8];
    uint8_t E_Test_2[8];
    uint8_t upper_USART_Test[8];
    uint8_t pwm_Test[8];
};

#endif
