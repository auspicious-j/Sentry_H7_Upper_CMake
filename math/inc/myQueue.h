/************************************************************
 * 文件   ：myQueue.h
 * 功能   ：通用循环队列(环形缓冲 / FIFO 先进先出队列)的接口声明。
 * 术语   ：
 *   - 队列(Queue)   ：遵循“先入先出(FIFO)”的线性结构，元素从队尾(rear)进、
 *                     从队头(front)出，即最早入队的元素最先出队。
 *   - 环形/循环队列 ：用一段定长数组作缓冲，下标走到末尾后自动回绕到 0，
 *                     像首尾相接的环，从而反复利用同一块内存，出队时无需搬移其它元素。
 *   - 队头(front)   ：下一个出队元素的下标；队尾(rear)：下一个入队元素的下标。
 * 说明   ：本队列默认只保存元素指针(void*)；若调用 Queue_AttachBuffer 附加
 *          数据保存区，入队时会自动把数据复制进保存区并保存其副本指针。
 * 典型用途：串口接收缓冲、中断与任务之间传递数据等。
 ************************************************************/
#ifndef _QUEUE_H_
#define _QUEUE_H_

#include "stdint.h"

#define EMPTY_QUEUE {NULL,0,0,0,NULL,0} // 空队列初值：按结构体成员顺序 {data,maxSize,front,rear,buffer,bufElemSize} 给出；可写 Queue q=EMPTY_QUEUE; 得到空队列

/************* 数据结构 **************/
// 队列结构体：描述一个环形队列的全部状态(指针存储区 + 可选的数据保存区)
typedef struct _Queue
{
    // ---- 指针存储区(队列本体) ----
    void **data; // 元素指针数组：每个格子存放一个 void* 指针；只保存指针本身，若要连数据本体一起保存请附加数据保存区
    // 队列容量：最多可存放的元素个数(单位：个)，取值需 >0
    uint16_t maxSize;
    // front=队头下标(出队位置)，rear=队尾下标(下一个入队位置)；
    // 取值范围均为 0 ~ maxSize-1，走到末尾后回绕到 0
    uint16_t front,rear;
    // ---- 数据保存区(可选用)：用于保存数据本体而非仅指针 ----
    // 附加后，入队会把数据复制到保存区中对应的位置，队列里保存的是这些副本的指针
    void *buffer;
    uint8_t bufElemSize; // 每个元素占用的字节数(单位：字节)；仅当 buffer 非空(已附加保存区)时有效
}Queue;

/************** 接口函数 ***************/
// 初始化队列：为指针数组分配内存，并把容量设为 maxSize
// 参数：queue   要初始化的队列；maxSize 队列容量(可存放的元素个数)
// 返回：无
// 说明：内部用 malloc 申请 maxSize × sizeof(void*) 字节；失败时 data 为 NULL，本函数不做判断
void Queue_Init(Queue *queue,uint16_t maxSize);
// 附加数据保存区：入队时把数据复制到该保存区对应位置，队列改为保存副本的指针
// 参数：queue 目标队列；buffer 保存区首地址；elemSize 每个元素占用的字节数
// 返回：无
// 注意：保存区字节数不得小于 队列容量 × 元素大小，否则入队复制时会越界
void Queue_AttachBuffer(Queue *queue,void *buffer,uint8_t elemSize);
// 销毁队列：释放指针数组内存，并把整个结构体清零
// 参数：queue 要销毁的队列
// 返回：无
void Queue_Destroy(Queue *queue);
// 获取当前队列中的元素个数
// 参数：queue 目标队列
// 返回：元素个数(0 表示空)
uint16_t Queue_Size(Queue *queue);
// 判断队列是否已满
// 参数：queue 目标队列
// 返回：1=已满；0=未满
uint8_t Queue_IsFull(Queue *queue);
// 判断队列是否为空
// 参数：queue 目标队列
// 返回：1=空；0=非空
uint8_t Queue_IsEmpty(Queue *queue);
// 获取队列中指定位置的元素(队头位置为 0，向后依次递增)
// 参数：queue 目标队列；position 相对队头的位置(从 0 起)
// 返回：该位置元素的指针；越界则返回 NULL
void* Queue_GetElement(Queue *queue,uint16_t position);
// 查看队头元素但不将其取出(peek 语义)
// 参数：queue 目标队列
// 返回：队头元素指针；空队列返回 NULL
void* Queue_Top(Queue *queue);
// 出队：取出并移除队头元素
// 参数：queue 目标队列
// 返回：被取出元素的指针；空队列返回 NULL
void* Queue_Dequeue(Queue *queue);
// 入队：把一个元素放入队尾
// 参数：queue 目标队列；data 要入队的元素指针
// 返回：无；若队列已满则忽略本次入队，队列保持不变
void Queue_Enqueue(Queue *queue,void* data);

#endif
