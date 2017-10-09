/***********************************************************************************************
************************************************************************************************
**
**文件名: bsp_uart.h
**作  者: mys
**日  期: 2017.09.08
**功  能:
**
**版  本: V1.0
************************************************************************************************
***********************************************************************************************/

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <cpu.h>
#include <os.h>
#include "bsp.h"
#include "stdio.h"
#include "lib_fifo.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_nvic.h"

//串口回调函数定义
typedef void (*UartSendFinishCallBack)(void);

 typedef struct{
  UartSendFinishCallBack pcallBack;   //发送完成回调函数
  uint32_t baud;          //波特率
}sUart_Paramer;         //串口参数设置结构体

/*
*********************************************************************************************************
*                                            USART ENABLE
*********************************************************************************************************
*/
#define COM1_ENABLE     1
#define COM2_ENABLE     1
#define COM3_ENABLE     0
#define COM4_ENABLE     1
#define COM5_ENABLE     0
#define COM1_MAP_DISABLE         0
/*
*********************************************************************************************************
*                                            USART BUFF SIZE DEF
*********************************************************************************************************
*/
#if COM1_ENABLE
#define com1ReciveBuffSize      2048    //接收数据缓存区大小
#define com1SendBuffSize        1054    //发送数据缓存区大小
#endif

#if COM2_ENABLE
#define com2ReciveBuffSize      2048
#define com2SendBuffSize        1054
#endif

#if COM3_ENABLE
#define com3ReciveBuffSize      2048
#define com3SendBuffSize        1054
#endif

#if COM4_ENABLE
#define com4ReciveBuffSize      2048
#define com4SendBuffSize        1054
#endif

#if COM5_ENABLE
#define com5ReciveBuffSize      2048
#define com5SendBuffSize        1054
#endif

typedef enum
{
#if COM1_ENABLE
  COM1 = 0,
#endif
#if COM2_ENABLE
  COM2,
#endif
#if COM3_ENABLE
  COM3,
#endif
#if COM4_ENABLE
  COM4,
#endif
#if COM5_ENABLE
  COM5,
#endif
  COMMAX
}eComPort;
/*******************************************************************************
*功能:打开串口
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*     pd           串口参数
*返回值:
*备注:
*******************************************************************************/
void UartOpen(eComPort eCom_no,const sUart_Paramer * pd);

/*******************************************************************************
*功  能:  获取串口接收状态
*参  数:  eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:  串口是否正在接收数据
*备  注:
*******************************************************************************/
bool UartIsRecive(eComPort eCom_no);

/*******************************************************************************
*功能:关闭串口
*参数:com_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:  关闭成功返回1,否则返回0
*备注:
*******************************************************************************/
void  UartClose(eComPort com_no);

/*******************************************************************************
*功能:清空串口接收缓冲区里的数据
*参数:com_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:
*备注:
*******************************************************************************/
void UartClearBuff(eComPort com_no);

/*******************************************************************************
*功能:从接收缓冲区中读取出指定长度的数据
*参数:com_no      串口号 COM1,COM2,COM3,COM4,COM5
*     pch         读出数据存储缓冲区
*     len         待读出数据长度
*返回值:写入成功返回true,否则返回false
*备注:实际读到的数据长度
*******************************************************************************/
uint16_t UartRead(eComPort com_no,uint8_t *pch,uint16_t len);

/*******************************************************************************
*功能:向串口写入数据
*参数:com_no      串口号 COM1,COM2,COM3,COM4,COM5
*     pch        待写入的数据缓存区
*     len         待写入的数据长度
*返回值:
*备注:不提供发送缓冲区,程式要等待串口发送完成后才能返回
*******************************************************************************/
uint16_t  UartWrite(eComPort com_no,const uint8_t *pch,uint16_t len);

/*******************************************************************************
*功能:向串口缓冲区写入数据
*参数:eCom_no      串口号 COM1,COM2,COM3,COM4,COM5
*     pch        待写入的数据缓存区
*     len         待写入的数据长度
*返回值:
*备注:提供发送缓冲区,程式要等待串口发送完成后才能返回
*******************************************************************************/
void  UartWaitSend(eComPort eCom_no);

/*******************************************************************************
*功能:中断处理函数
*参数:com_no      串口号 COM1,COM2,COM3,COM4,COM5
*返回值:
*备注:
*******************************************************************************/
void UartIRQ(eComPort com_no);

#endif /* _UART_H */
