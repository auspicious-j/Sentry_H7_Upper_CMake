#ifndef _SLOPE_H_
#define _SLOPE_H_

#include "main.h"

//斜坡结构体
typedef struct{
    float target; //目标值
    float step; //步长值
    float value; //当前值
    float deadzone; //死区，误差小于该值则不进行调节
}Slope;

void Slope_Init(Slope *slope,float step,float deadzone);
void Slope_SetTarget(Slope *slope,float target);
void Slope_SetStep(Slope *slope,float step);
float Slope_NextVal(Slope *slope);
float Slope_GetVal(Slope *slope);

#endif
