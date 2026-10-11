/**************** 通用循环队列 *****************
 * 文件：myQueue.c
 * 功能：实现 myQueue.h 中声明的“通用循环队列(环形缓冲 / FIFO 先进先出队列)”。
 * 术语：队列=先入先出；环形队列=定长数组下标回绕复用；队头(front)出、队尾(rear)进。
 * 说明：队列默认只保存元素指针(void*)；若附加了数据保存区，入队会自动复制数据本体。
 *       常见用途：串口接收缓冲、中断与任务之间传递数据等。
 **********************************************/

#include "myQueue.h"
#include <string.h>
#include <stdlib.h>

// 初始化队列
// 参数：queue 目标队列；maxSize 队列容量(可存放的元素个数)
// 返回：无
// 说明：申请 maxSize 个 void* 指针的空间作为环形缓冲区
void Queue_Init(Queue *queue,uint16_t maxSize)
{
    queue->data=malloc(maxSize*sizeof(void*)); // 申请 maxSize 个指针(void*)大小的内存作为环形缓冲区
    queue->maxSize=maxSize; // 记录容量，供回绕取模与判满/判空使用
}

// 附加数据保存区：入队时把数据复制到该保存区对应位置，队列保存的是这份副本的指针
// 注意：保存区字节数不得小于 队列容量 × 元素大小，否则入队复制时会越界
void Queue_AttachBuffer(Queue *queue,void *buffer,uint8_t elemSize)
{
    queue->buffer=buffer; // 记录保存区首地址；此后视为“已附加保存区”
    queue->bufElemSize=elemSize; // 记录每个元素的字节数，供入队复制时计算偏移
}

// 销毁队列：释放指针数组内存，并把整个结构体清零
void Queue_Destroy(Queue *queue)
{
    free(queue->data); // 释放 Queue_Init 申请的指针数组内存
    memset(queue,0,sizeof(Queue)); // 结构体整体清零(front=rear=0、buffer=NULL)，回到空队列状态
}

// 获取当前队列中的元素个数
uint16_t Queue_Size(Queue *queue)
{
    return (queue->rear+queue->maxSize-queue->front)%queue->maxSize; // (rear-front+maxSize)%maxSize：加 maxSize 防止出现负数，取模即得元素个数
}

// 判断队列是否已满
uint8_t Queue_IsFull(Queue *queue)
{
    return (queue->front==(queue->rear+1)%queue->maxSize)?1:0; // 环形队列预留一个空位以区分“满/空”：front 等于 rear 的下一位即判满
}

// 判断队列是否为空
uint8_t Queue_IsEmpty(Queue *queue)
{
    return (queue->front==queue->rear)?1:0; // front 与 rear 重合表示没有任何元素，即空
}

// 获取队列中指定位置的元素(队头位置为 0，向后依次递增)
void* Queue_GetElement(Queue *queue,uint16_t position)
{
    if(position>=Queue_Size(queue)) return NULL; // 位置超出当前元素个数：越界，返回 NULL

    return queue->data[(queue->front+position)%queue->maxSize]; // 从队头起算第 position 个，下标回绕取模
}

// 查看队头元素但不将其取出(peek 语义)
void* Queue_Top(Queue *queue)
{
    if(Queue_IsEmpty(queue)) return NULL; // 空队列没有队头元素

    return queue->data[queue->front]; // 只读取队头指针，不移动 front(不出队)
}

// 出队：取出并移除队头元素
void* Queue_Dequeue(Queue *queue)
{
    if(Queue_IsEmpty(queue)) return NULL; // 空队列无可出队元素

    void* res=queue->data[queue->front]; // 先取出队头元素指针
    queue->front=(queue->front+1)%queue->maxSize; // 队头后移一位，到末尾则回绕到 0
    return res;
}

// 入队：把元素放入队尾(已满则忽略本次入队)
void Queue_Enqueue(Queue *queue,void* data)
{
    if(Queue_IsFull(queue)) return; // 队列已满：丢弃本次入队，保持原队列不变

    // 未附加保存区(buffer==NULL)时直接保存指针本身；附加了保存区则先复制数据、再保存副本指针
    if(queue->buffer == NULL) // 分支一：未附加保存区，直接保存传入的指针
	{
    // 把元素指针写入队尾格子 data[rear]
	    queue->data[queue->rear]=data;
	}
    else // 分支二：已附加保存区：先把数据复制到保存区，再让队列保存副本的地址
	{
    // memcpy：把 data 指向的 bufElemSize 字节复制到保存区中第 rear 个格子
    //   目标地址 = (uint8_t*)buffer + rear × bufElemSize；拷贝长度 = bufElemSize
	    memcpy((uint8_t*)(queue->buffer) + (queue->rear)*(queue->bufElemSize),
					    data,
					    queue->bufElemSize);
    // 让队尾格子保存这份副本的地址(即保存区中该格子的位置)
	    queue->data[queue->rear]=(uint8_t*)(queue->buffer) + (queue->rear)*(queue->bufElemSize);
	}

    queue->rear=(queue->rear+1)%queue->maxSize; // 队尾后移一位，到末尾则回绕到 0
}
