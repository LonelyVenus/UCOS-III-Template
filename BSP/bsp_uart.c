/***********************************************************************************************
************************************************************************************************
**
**文件名: bsp_uart.c
**作  者: mys
**日  期: 2017.09.08
**功  能: 提供串口读写功能
**
**版  本: V1.0
************************************************************************************************
***********************************************************************************************/
#include "bsp_uart.h"

/*
*********************************************************************************************************
*                                            LOCAL FUNCTION
*********************************************************************************************************
*/
#if COM1_ENABLE
void Uart1IRQ();
#endif
#if COM2_ENABLE
void Uart2IRQ();
#endif
#if COM3_ENABLE
void Uart3IRQ();
#endif
#if COM4_ENABLE
void Uart4IRQ();
#endif
#if COM5_ENABLE
void Uart5IRQ();
#endif


typedef struct{
  USART_TypeDef * const sReg;            //串口对应的寄存器
  OS_TCB  *p_tcb;                        //OS任务控制块
  uint8_t * preciveBuff;
  uint8_t * psendBuff;
  uint16_t reciveBuffSize;
  uint16_t sendBuffSize;
  CFifoWriteCallBack callBack;
}sUart_Base_Info;                        //串口基本参数信息

typedef struct
{
  sUart_Paramer par;                    //参数
  scFifo reciveFifo;                    //接收缓冲区
  scFifo sendFifo;                      //发送缓冲区
  uint8_t  opensFlag;                   //串口打开标志：1表示打开，0表示关闭
  volatile  bool  isRecive;             //正在接受数据标志
  volatile  bool  isSending;            //正在发送标志
}sUartMem;                              //串口参数

//串口配置
#if COM1_ENABLE
uint8_t com1ReciveBuff[com1ReciveBuffSize];    //接收缓冲区大小定义
uint8_t com1SendBuff[com1SendBuffSize];        //发送缓冲区大小定义
void UartStartSendData1(void);                 //实际发送数据
OS_TCB  Uart1Tcb;
#endif

//涓插彛2閰嶇疆
#if COM2_ENABLE
uint8_t com2ReciveBuff[com2ReciveBuffSize];
uint8_t com2SendBuff[com2SendBuffSize];
void UartStartSendData2(void);
OS_TCB  Uart2Tcb;
#endif

//涓插彛3閰嶇疆
#if COM3_ENABLE
uint8_t com3ReciveBuff[com3ReciveBuffSize];
uint8_t com3SendBuff[com3SendBuffSize];
void UartStartSendData3(void);
OS_TCB  Uart3Tcb;
#endif

//涓插彛4閰嶇疆
#if COM4_ENABLE
uint8_t com4ReciveBuff[com4ReciveBuffSize];
uint8_t com4SendBuff[com4SendBuffSize];
void UartStartSendData4(void);
OS_TCB  Uart4Tcb;
#endif

//涓插彛5閰嶇疆
#if COM5_ENABLE
uint8_t com5ReciveBuff[com5ReciveBuffSize];
uint8_t com5SendBuff[com5SendBuffSize];
void UartStartSendData5(void);
OS_TCB  Uart5Tcb;
#endif

const sUart_Base_Info sUart_base_info[COMMAX] = {
#if COM1_ENABLE
                                 {USART1,&Uart1Tcb,com1ReciveBuff,com1SendBuff,
                                  com1ReciveBuffSize,com1SendBuffSize,UartStartSendData1},
#endif
#if COM2_ENABLE
                                 {USART2,&Uart2Tcb,com2ReciveBuff,com2SendBuff,
                                  com2ReciveBuffSize,com2SendBuffSize,UartStartSendData2},
#endif
#if COM3_ENABLE
                                 {USART3,&Uart3Tcb,com3ReciveBuff,com3SendBuff,
                                  com3ReciveBuffSize,com3SendBuffSize,UartStartSendData3},
#endif
#if COM4_ENABLE
                                 {UART4,&Uart4Tcb,com4ReciveBuff,com4SendBuff,
                                  com4ReciveBuffSize,com4SendBuffSize,UartStartSendData4},
#endif
#if COM5_ENABLE
                                 {UART5,&Uart5Tcb,com5ReciveBuff,com5SendBuff,
                                  com5ReciveBuffSize,com5SendBuffSize,UartStartSendData5}
#endif
};  //串口常量定义

sUartMem uartMem[COMMAX];        //串口配置信息定义

/*******************************************************************************
*功  能:  串口管理初始化
*参  数:  void
*返回值: void
*备  注:
*******************************************************************************/
void UartInit(void)
{
  uint32_t i;
  for(i = 0;i < COMMAX;i++)
  {
    uartMem[i].opensFlag = 0;
    uartMem[i].isRecive = FALSE;
  }
}

/*******************************************************************************
*功能:配置COM管脚
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:
*备注:内部函数
*******************************************************************************/
void UartGpioRccIrqConfig(eComPort eCom_no)
{
  if(eCom_no > COMMAX)
    return;
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  switch (eCom_no)
  {
#if COM1_ENABLE
    case COM1:
      RCC->APB2ENR |= RCC_APB2Periph_USART1;//uart1 clock
      sUart_base_info[eCom_no].sReg->SR   = 0x00000000;
#if COM1_MAP_DISABLE       //如果不复用串口1管脚
      //配置PA9,PA10
      RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
      GPIOA->CRH = (GPIOA->CRH&0xFFFFF00F)|0x4B0;      
#else
      //复用管脚PB6,PB7作为串口1
      RCC->APB2ENR |= RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO;
      AFIO->MAPR = (AFIO->MAPR&0xFFFFFFFB)|(0x1<<2);
      GPIOB->CRL = (GPIOB->CRL&0x00FFFFFF)| 0x4B000000;
#endif
      NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
      NVIC_Init(&NVIC_InitStructure); 
      BSP_IntVectSet(BSP_INT_ID_USART1,Uart1IRQ);
      BSP_IntEn(BSP_INT_ID_USART1);
      break;
#endif
#if COM2_ENABLE
    case COM2:
      RCC->APB1ENR   |= RCC_APB1Periph_USART2;//uart2 clock
      sUart_base_info[eCom_no].sReg->SR   = 0x00000000;
      RCC->APB2ENR   |= RCC_APB2Periph_GPIOA;  //打开GPIOA的时钟
      GPIOA->CRL = (GPIOA->CRL&0xFFFF00FF)|0x4B00;
      NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
      NVIC_Init(&NVIC_InitStructure);
      BSP_IntVectSet(BSP_INT_ID_USART2,Uart2IRQ);
      BSP_IntEn(BSP_INT_ID_USART2);
      break;
#endif
#if COM3_ENABLE
    case COM3:
      RCC->APB1ENR   |= RCC_APB1Periph_USART3;//uart3 clock
      sUart_base_info[eCom_no].sReg->SR   = 0x00000000;
      RCC->APB2ENR   |= RCC_APB2Periph_GPIOB;
      GPIOB->CRH = (GPIOB->CRH&0xFFFF00FF)|0x4B00;      //PB10,PB11
      NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
      NVIC_Init(&NVIC_InitStructure);
      BSP_IntVectSet(BSP_INT_ID_USART3,Uart3IRQ);
      BSP_IntEn(BSP_INT_ID_USART3);
      break;
#endif
#if COM4_ENABLE
    case COM4:
      RCC->APB1ENR   |= RCC_APB1Periph_UART4;
      sUart_base_info[eCom_no].sReg->SR   = 0x00000000;
      RCC->APB2ENR   |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;
      GPIOC->CRH = (GPIOC->CRH&0xFFFF00FF)|(0x4B<<8);      //UART4 PC10 PC11
      NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQChannel;
      NVIC_Init(&NVIC_InitStructure);
      BSP_IntVectSet(BSP_INT_ID_USART4,Uart4IRQ);
      BSP_IntEn(BSP_INT_ID_USART4);
      break;
#endif
#if COM5_ENABLE
    case COM5:
      RCC->APB1ENR   |= RCC_APB1Periph_UART5;
      sUart_base_info[eCom_no].sReg->SR   = 0x00000000;
      RCC->APB2ENR   |= 0x31;
      GPIOC->CRH = GPIOC->CRH&0xFFF0FFFF|(0x0B<<16);      //UART5   PC12 TX
      GPIOD->CRL = GPIOD->CRL&0xFFFFF0FF|(0x4<<8);        //UART5   PD2  RX
      NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
      NVIC_Init(&NVIC_InitStructure);
      BSP_IntVectSet(BSP_INT_ID_USART5,Uart5IRQ);
      BSP_IntEn(BSP_INT_ID_USART5);
      break;
#endif
    default:
    	break;
  }
}

/*******************************************************************************
*功能:打开串口
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*     pd           串口参数
*返回值:void
*备注:
*******************************************************************************/
void UartOpen(eComPort eCom_no,const sUart_Paramer * pd)
{
  uint32_t integerdivider, tmpreg,fractionaldivider,apbclock;
  if(eCom_no >= COMMAX)
    return;
  uartMem[eCom_no].par = *pd;
  /*初始化sUart_info结构体*/
  CFifoCreate(&uartMem[eCom_no].reciveFifo,sUart_base_info[eCom_no].preciveBuff,sUart_base_info[eCom_no].reciveBuffSize,NULL);
  CFifoCreate(&uartMem[eCom_no].sendFifo,sUart_base_info[eCom_no].psendBuff,sUart_base_info[eCom_no].sendBuffSize,sUart_base_info[eCom_no].callBack);
  
  /* USART configured as follow:
          - BaudRate = pd->baud
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Hardware flow control disabled (RTS and CTS signals)
          - Receive and transmit enabled
    */
  UartGpioRccIrqConfig(eCom_no);
  RCC_ClocksTypeDef rcc_clk;
  RCC_GetClocksFreq(&rcc_clk);
  if(eCom_no  == 0 && sUart_base_info[eCom_no].sReg == USART1)
    apbclock  = rcc_clk.PCLK2_Frequency;   //BaudRate Register
  else 
    apbclock  = rcc_clk.PCLK1_Frequency;
  //计算波特率设置参数
  integerdivider = ((0x19 * apbclock) / (0x04 * pd->baud));
  tmpreg = (integerdivider / 0x64) << 0x04;
  fractionaldivider = integerdivider - (0x64 * (tmpreg >> 0x04));
  tmpreg |= ((((fractionaldivider * 0x10) + 0x32) / 0x64)) & ((uint8_t)0x0F);

  sUart_base_info[eCom_no].sReg->BRR = (uint16_t)tmpreg;
  if(eCom_no == COM2) 
    sUart_base_info[eCom_no].sReg->CR2   = 0x00002740;
  else
    sUart_base_info[eCom_no].sReg->CR2   = 0x00000740;                    //控制寄存器
  sUart_base_info[eCom_no].sReg->CR3   = 0x00000000;
  uartMem[eCom_no].opensFlag  = 1;
  //使能RE      0x04    接收使能
  //使能TE      0x08    发送使能
  //使能IDLEIE  0x10    IDLE中断使能
  //使能RXNEIE  0x20    接收缓冲区非空中断使能
  //使能UE      0x2000  UART使能
  sUart_base_info[eCom_no].sReg->CR1   = 0x0000203C;         
  //使能收发，开RXTx中断//使能发送中断
  uint16_t data = sUart_base_info[eCom_no].sReg->DR;
  sUart_base_info[eCom_no].sReg->SR &= ~0x40;
}

/*******************************************************************************
*功  能:  获取串口接收状态
*参  数:  eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:  串口是否正在接收数据
*备  注:
*******************************************************************************/
bool UartIsRecive(eComPort eCom_no)
{
  if(eCom_no >= COMMAX)
    return FALSE;
  return uartMem[eCom_no].isRecive;
}

/*******************************************************************************
*功能:关闭串口
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:  关闭成功返回1,否则返回0
*备注:
*******************************************************************************/
void  UartClose(eComPort eCom_no)
{
  if(eCom_no >= COMMAX)
    return;
  sUart_base_info[eCom_no].sReg->CR1   = 0x00000000;
  sUart_base_info[eCom_no].sReg->CR2   = 0x00000000;
  sUart_base_info[eCom_no].sReg->CR3   = 0x00000000;

  sUart_base_info[eCom_no].sReg->SR   = 0x00000000;
  uartMem[eCom_no].opensFlag  = 0;
  switch (eCom_no)
  {
#if COM1_ENABLE
    case COM1:
        RCC->APB2ENR   &= ~(1<<14);//关闭uart1 clock
        break;
#endif
#if COM2_ENABLE
    case COM2:
        RCC->APB1ENR   &= ~(1<<17);
        break;
#endif
#if COM3_ENABLE
    case COM3:
        RCC->APB1ENR   &= ~(1<<18);
        break;
#endif
#if COM4_ENABLE
    case COM4:
        RCC->APB1ENR   &= ~(1<<19);
        break;
#endif
#if COM5_ENABLE
    case COM5:
        RCC->APB1ENR   &= ~(1<<20);
        break;
#endif
    default:
    	break;
  }
}

/*******************************************************************************
*功能:清空串口接收缓冲区里的数据
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:
*备注:
*******************************************************************************/
void UartClearBuff(eComPort eCom_no)
{
  if(eCom_no >= COMMAX)
    return;
  CFifoEmpty(&uartMem[eCom_no].reciveFifo);
}

/*******************************************************************************
*功  能:  从接收缓冲区中读取出指定长度的数据
*参  数:  eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*参  数:  pch         读出数据存储缓冲区
*参  数:  len         待读出数据长度
*返回值:  写入成功返回true,否则返回false
*备  注:  实际读到的数据长度
*******************************************************************************/
uint16_t UartRead(eComPort eCom_no,uint8_t *pch,uint16_t len)
{
  if(eCom_no >= COMMAX)
    return 0;
  if(uartMem[eCom_no].opensFlag  == 0)
    return 0;
  return  CFifoReads(&uartMem[eCom_no].reciveFifo,pch,len);
}

/*******************************************************************************
*功能:向串口缓冲区写入数据
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*     pch        待写入的数据缓存区
*     len         待写入的数据长度
*返回值:
*备注:提供发送缓冲区,程式要等待串口发送完成后才能返回
*******************************************************************************/
uint16_t  UartWrite(eComPort eCom_no,const uint8_t *pch,uint16_t len)
{
  if(eCom_no >= COMMAX)
    return 0;
  if(uartMem[eCom_no].opensFlag  == 0)
    return 0;
  return CFifoWrites(&uartMem[eCom_no].sendFifo,pch,len);
}

/*******************************************************************************
*功能:等待数据发送完成
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:
*备注:提供发送缓冲区,程式要等待串口发送完成后才能返回
*******************************************************************************/
void  UartWaitSend(eComPort eCom_no)
{
  if(eCom_no >= COMMAX)
    return ;
  if(uartMem[eCom_no].opensFlag  == 0)
    return ;
  while(uartMem[eCom_no].isSending)
      ;
}


/*******************************************************************************
*功能:向将缓冲区中的数据写入串口发送寄存器
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:
*备注:
*******************************************************************************/
void UartStartSendData(eComPort eCom_no)
{
	//使能TXEIE 发送缓冲区空中断使能
  uartMem[eCom_no].isSending = TRUE;
  sUart_base_info[eCom_no].sReg->CR1   |= 0x00000080;
}
#if COM1_ENABLE
void UartStartSendData1(void)
{
  UartStartSendData(COM1);
}
#endif

#if COM2_ENABLE
void UartStartSendData2(void)
{
  UartStartSendData(COM2);
}
#endif
#if COM3_ENABLE
void UartStartSendData3(void)
{
  UartStartSendData(COM3);
}
#endif
#if COM4_ENABLE
void UartStartSendData4(void)
{
  UartStartSendData(COM4);
}
#endif
#if COM5_ENABLE
void UartStartSendData5(void)
{
  UartStartSendData(COM5);
}
#endif

/*******************************************************************************
*功能:中断处理函数
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:
*备注:
*******************************************************************************/
void UartIRQ(eComPort eCom_no)
{
  uint8_t err;
  uint16_t state  = 0;
  OS_ERR Uart_Err;
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();  
  if((eCom_no < COMMAX))
  {
    state = sUart_base_info[eCom_no].sReg->SR;
    if(state & 0x1f8)
    {
      if(state&0x20)    //接受中断标志
      {
        CFifoWriteNoWait(&uartMem[eCom_no].reciveFifo,(sUart_base_info[eCom_no].sReg->DR));
        uartMem[eCom_no].isRecive = TRUE;
        //PostEvent(sUart_base_info[eCom_no].event);
        OSTaskSemPost ((OS_TCB *)sUart_base_info[eCom_no].p_tcb,
                       (OS_OPT)OS_OPT_POST_NONE,
                       (OS_ERR *)&Uart_Err);
      }
      else if(state&0x08)
        err = sUart_base_info[eCom_no].sReg->DR;     //丢弃接收到的数据不用.
      else if(state&0x10)                            
      {//接收到空闲字符
        uartMem[eCom_no].isRecive = FALSE;
        err = sUart_base_info[eCom_no].sReg->DR;
        //PostEvent(sUart_base_info[eCom_no].event);
        OSTaskSemPost ((OS_TCB *)sUart_base_info[eCom_no].p_tcb,
                       (OS_OPT)OS_OPT_POST_NONE,
                       (OS_ERR *)&Uart_Err);
      }
      else if(state & 0x100)
      {
        uartMem[eCom_no].isRecive = FALSE;
        sUart_base_info[eCom_no].sReg->SR = 0;
        //PostEvent(sUart_base_info[eCom_no].event);
        OSTaskSemPost ((OS_TCB *)sUart_base_info[eCom_no].p_tcb,
                       (OS_OPT)OS_OPT_POST_NONE,
                       (OS_ERR *)&Uart_Err);
      }
      if(state&0x80 && (sUart_base_info[eCom_no].sReg->CR1 & 0x00000080) != 0 )   
      {//发送数据中断
        if(CFifoRead(&uartMem[eCom_no].sendFifo,&err) == 1)
        {
          sUart_base_info[eCom_no].sReg->DR =  err;
        }
        else
        {
          
          sUart_base_info[eCom_no].sReg->SR &= ~0x80;
          sUart_base_info[eCom_no].sReg->CR1   &= ~0x00000080;  //禁止TXEIE 发送缓冲区空中断禁止
          sUart_base_info[eCom_no].sReg->CR1 |= 0x00000040;     //使能发送完成中断
        }
      }
      else if(state & 0x40 && (sUart_base_info[eCom_no].sReg->CR1 & 0x00000040) != 0)
      {//发送完成中断
        sUart_base_info[eCom_no].sReg->SR &= ~0x40;
        sUart_base_info[eCom_no].sReg->CR1 &= ~0x00000040;      //禁止TCIE发送完成中断
        uartMem[eCom_no].isSending = FALSE;
        if(uartMem[eCom_no].par.pcallBack != NULL)
          uartMem[eCom_no].par.pcallBack();
      }
    }
  } 
  CPU_CRITICAL_EXIT();
}

#if COM1_ENABLE
void Uart1IRQ()
{
  UartIRQ(COM1);
}
#endif
#if COM2_ENABLE
void Uart2IRQ()
{
  UartIRQ(COM2);
}
#endif
#if COM3_ENABLE
void Uart3IRQ()
{
  UartIRQ(COM3);
}
#endif
#if COM4_ENABLE
void Uart4IRQ()
{
  UartIRQ(COM4);
}
#endif
#if COM5_ENABLE
void Uart5IRQ()
{
  UartIRQ(COM5);
}
#endif




