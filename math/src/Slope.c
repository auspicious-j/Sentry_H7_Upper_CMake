#include "Slope.h"

#include <math.h>

//初始化斜坡参数
void Slope_Init(Slope *slope,float step,float deadzone)
{
    slope->target=0;
    slope->step=step;
    slope->value=0;
    slope->deadzone=deadzone;
}

//设定斜坡目标
void Slope_SetTarget(Slope *slope,float target)
{
    slope->target=target;
}

//设定斜坡参数
void Slope_SetStep(Slope *slope,float step)
{
    slope->step=step;
}

//计算一次斜坡值，更新slope->value，返回该值
float Slope_NextVal(Slope *slope)
{
    float error=slope->value-slope->target;//当前值与目标值的差值
	
    if(fabsf(error)<slope->deadzone)//误差在死区内，当前值不发生改变
	    return slope->value;
	
    if(fabsf(error)<slope->step)//误差不足步长，当前值直接设为目标值
	    slope->value=slope->target;
    else if(error<0)
	    slope->value+=slope->step;
    else if(error>0)
	    slope->value-=slope->step;
    return slope->value;
}

//获取斜坡当前值
float Slope_GetVal(Slope *slope)
{
    return slope->value;
}
