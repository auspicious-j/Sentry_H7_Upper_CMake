#ifndef _SLOPE_H_
#define _SLOPE_H_

#include "main.h"

//б�½ṹ��
typedef struct{
    float target; //Ŀ��ֵ
    float step; //����ֵ
    float value; //��ǰֵ
    float deadzone; //����������ֵС�ڸ�ֵ�򲻽�������
}Slope;

void Slope_Init(Slope *slope,float step,float deadzone);
void Slope_SetTarget(Slope *slope,float target);
void Slope_SetStep(Slope *slope,float step);
float Slope_NextVal(Slope *slope);
float Slope_GetVal(Slope *slope);

#endif
