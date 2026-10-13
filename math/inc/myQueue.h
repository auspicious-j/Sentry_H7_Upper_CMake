#ifndef _QUEUE_H_
#define _QUEUE_H_

#include "stdint.h"

#define EMPTY_QUEUE {NULL,0,0,0,NULL,0}

/*************���ݽṹ**************/
//���нṹ��
typedef struct _Queue
{
	//��������
    void **data;//ֻ����ָ�룬��Ҫͬʱ����ָ��������븽�ӱ�����
    uint16_t maxSize;
    uint16_t front,rear;
	//���ݱ�����(��ѡ��)
    void *buffer;
    uint8_t bufElemSize;//ÿ��Ԫ�صĴ�С
}Queue;

/**************�ӿں���***************/
void Queue_Init(Queue *queue,uint16_t maxSize);
void Queue_AttachBuffer(Queue *queue,void *buffer,uint8_t elemSize);
void Queue_Destroy(Queue *queue);
uint16_t Queue_Size(Queue *queue);
uint8_t Queue_IsFull(Queue *queue);
uint8_t Queue_IsEmpty(Queue *queue);
void* Queue_GetElement(Queue *queue,uint16_t position);
void* Queue_Top(Queue *queue);
void* Queue_Dequeue(Queue *queue);
void Queue_Enqueue(Queue *queue,void* data);

#endif
