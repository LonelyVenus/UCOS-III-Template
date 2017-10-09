/***********************************************************************************************
************************************************************************************************
**
**文件名: lib_fifo.c
**作  者: mys
**日  期: 2017.09.08
**功  能: 循环先入先出队列(带数据读写)
**备  注: 数据池使用任意类型的数组
************************************************************************************************
***********************************************************************************************/

#include "lib_fifo.h"

/*******************************************************************************
*功  能:  使用指定的参数创建队列
*参  数:  psFifo -- 空的队列对象
*参  数:  pbuff  -- 队列使用的数据池
*参  数:  buff_size -- 队列的大小
*返回值:
*备  注:
*******************************************************************************/
void CFifoCreate(scFifo * psFifo,CPU_INT08U * pbuff,CPU_INT16U buff_size,CFifoWriteCallBack callBack)
{
  if(psFifo == NULL)
    return;
  psFifo->readItem = 0;           //读数据位置
  psFifo->writeItem = 0;          //写入位置
  psFifo->dataNum = 0;            //当前数据数量
  psFifo->maxNum = buff_size;     //最大数据数量
  psFifo->pbuff = pbuff;          //数据池
  psFifo->callBack = callBack;
}

/*******************************************************************************
*功  能:  数据数量减1
*参  数:  psFifo -- 队列对象
*返回值:
*备  注:
*******************************************************************************/
inline void DataDec(scFifo * psFifo)
{
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  psFifo->dataNum --;
  psFifo->readItem ++;
  if(psFifo->readItem >= psFifo->maxNum)
    psFifo->readItem = 0;
  CPU_CRITICAL_EXIT();
}

/*******************************************************************************
*功  能:  数据数量加1
*参  数:  psFifo -- 队列对象
*返回值:
*备  注:
*******************************************************************************/
inline void DataInc(scFifo * psFifo)
{
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  psFifo->dataNum ++;
  psFifo->writeItem ++;
  if(psFifo->writeItem >= psFifo->maxNum)
    psFifo->writeItem = 0;
  CPU_CRITICAL_EXIT();
}

/*******************************************************************************
*功  能:  从指定队列中读出一个数据
*参  数:  psFifo -- 队列对象
*参  数:  pch  -- 读出的数据
*返回值:  读出数据的长度
*备  注:  当队列对象不存在或数据存储位置不存在或队列为空时返回0
*******************************************************************************/
CPU_INT16U CFifoRead(scFifo * psFifo,CPU_INT08U * pch)
{
  if(psFifo == NULL || pch == NULL || psFifo->dataNum == 0)
    return 0;
  *pch = psFifo->pbuff[psFifo->readItem];
  DataDec(psFifo);
  return 1;
}

/*******************************************************************************
*功  能:  从指定队列中读出一个数据
*参  数:  psFifo -- 队列对象
*参  数:  pch  -- 读出的数据
*参  数:  len  -- 读出数据的长度
*返回值:  读出数据的长度
*备  注:  当队列对象不存在或数据存储位置不存在或队列为空时返回0
*******************************************************************************/
CPU_INT16U CFifoReads(scFifo * psFifo,CPU_INT08U * pch, CPU_INT16U len)
{
  CPU_INT16U cur;
  if(psFifo == NULL || pch == NULL)
    return 0;
  for(cur = 0;cur < len && psFifo->dataNum != 0;cur++)
  {
    *(pch++) = psFifo->pbuff[psFifo->readItem];
    DataDec(psFifo);
  }
  return cur;
}

/*******************************************************************************
*功  能:  将数据写入指定队列
*参  数:  psFifo -- 队列对象
*参  数:  ch  -- 写入的数据
*返回值:  写入数据数量
*备  注:  当队列对象不存在或写入的数据不存在或队列为满时返回0
*******************************************************************************/
CPU_INT16U CFifoWriteNoWait(scFifo * psFifo,CPU_INT08U ch)
{
  if(psFifo == NULL || psFifo->dataNum >= psFifo->maxNum)
    return 0;
  psFifo->pbuff[psFifo->writeItem] = ch;
  DataInc(psFifo);
  if(psFifo->callBack != NULL)
    psFifo->callBack();
    
  return 1;
}

/*******************************************************************************
*功  能:  无等待的数据写入
*参  数:  psFifo -- 队列对象
*参  数:  pch  -- 写入的数据
*参  数:  len  -- 写入数据的数量
*返回值:  写入数据数量
*备  注:  当队列对象不存在或写入的数据不存在或队列为满时返回0
*******************************************************************************/
CPU_INT16U CFifoWritesNoWait(scFifo * psFifo,CPU_INT08U* pch,CPU_INT16U len)
{
  CPU_INT16U cur;
  if(psFifo == NULL || pch == NULL)
    return 0;
  for(cur = 0; cur < len && psFifo->dataNum < psFifo->maxNum;cur++)
  {
    psFifo->pbuff[psFifo->writeItem] = *(pch++);
    DataInc(psFifo);
  }
  if(psFifo->callBack != NULL)
    psFifo->callBack();
  return cur;
}

/*******************************************************************************
*功  能:  将数据写入指定队列
*参  数:  psFifo -- 队列对象
*参  数:  ch  -- 写入的数据
*返回值:  是否写入成功
*备  注:  当队列对象不存在或写入的数据不存在返回false,队列满时等待
*******************************************************************************/
CPU_INT16U CFifoWrite(scFifo * psFifo,CPU_INT08U ch)
{
  if(psFifo == NULL)
    return 0;
  while(psFifo->dataNum >= psFifo->maxNum)
    ;
  psFifo->pbuff[psFifo->writeItem] = ch;
  DataInc(psFifo);
  if(psFifo->callBack != NULL)
    psFifo->callBack();
  return 1;
}

/*******************************************************************************
*功  能:  将数据写入指定队列
*参  数:  psFifo -- 队列对象
*参  数:  pch  -- 写入的数据
*参  数:  len   -- 写入数据的长度
*返回值:  是否写入成功
*备  注:  当队列对象不存在或写入的数据不存在返回false,队列满时等待
*******************************************************************************/
CPU_INT16U CFifoWrites(scFifo * psFifo,const CPU_INT08U* pch,CPU_INT16U len)
{
  CPU_INT16U cur = 0;
  if(psFifo == NULL || pch == NULL)
    return 0;
  for(cur = 0;cur < len;cur++)
  {
    while(psFifo->dataNum >= psFifo->maxNum)
      ;
    psFifo->pbuff[psFifo->writeItem] = *(pch++);
    DataInc(psFifo);
    if(cur == 0 || psFifo->callBack != NULL)
      psFifo->callBack();
  }
  return len;
}

/*******************************************************************************
*功  能:  判断队列是否为空
*参  数:  psFifo -- 队列对象
*返回值:  是否为空
*备  注:  当对象存在且为空时返回true否则返回false
*******************************************************************************/
bool CFifoIsEmpty(scFifo * psFifo)
{
  if(psFifo != NULL && psFifo->dataNum == 0)
    return TRUE;
  return FALSE;
}

/*******************************************************************************
*功  能:  清空队列
*参  数:  psFifo -- 队列对象
*返回值:
*备  注:
*******************************************************************************/
void CFifoEmpty(scFifo * psFifo)
{
  if(psFifo == NULL)
    return;
  psFifo->readItem = 0;           //读数据位置
  psFifo->writeItem = 0;          //写入位置
  psFifo->dataNum = 0;            //当前数据数量
  memset(psFifo->pbuff,0,psFifo->maxNum);
}

/*******************************************************************************
*功  能:  判断队列是否已满
*参  数:  psFifo -- 队列对象
*返回值:  是否已满
*备  注:  当对象存在且已满时返回true否则返回false
*******************************************************************************/
bool CFifoIsFill(scFifo * psFifo)
{
  if(psFifo != NULL && psFifo->dataNum == psFifo->maxNum)
    return TRUE;
  return FALSE;
}
