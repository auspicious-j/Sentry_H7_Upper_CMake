/*
 * Slope.c —— 斜率（斜坡）限制模块实现
 * 作用：让数值以受限的速率逼近目标值，用于平滑给定信号、限制变化率，
 *       避免给定值瞬间跳变对执行机构造成冲击。
 */
#include "Slope.h"

#include <math.h>

/* 初始化斜坡参数
 * 说明：步长与死区由调用者指定；target/value 先清零，
 *       之后需通过 Slope_SetTarget 设定目标、Slope_NextVal 逐步逼近。
 * 参数 slope    : 待初始化的斜坡对象
 * 参数 step     : 步长（单次最大变化量）
 * 参数 deadzone : 死区阈值
 * 返回 无 */
void Slope_Init(Slope *slope,float step,float deadzone)
{
    slope->target=0;          /* 目标值清零（初始无目标） */
    slope->step=step;         /* 写入步长 */
    slope->value=0;           /* 当前值清零（从 0 开始） */
    slope->deadzone=deadzone; /* 写入死区 */
}

/* 设定斜坡目标
 * 说明：仅更新目标值，当前值不会立刻变化，
 *       需后续调用 Slope_NextVal 逐步逼近。
 * 参数 slope  : 斜坡对象
 * 参数 target : 新的目标值
 * 返回 无 */
void Slope_SetTarget(Slope *slope,float target)
{
    slope->target=target; /* 记录新的目标值 */
}

/* 设定斜坡步长
 * 说明：可在运行中动态修改，改变当前值逼近目标的速度。
 * 参数 slope : 斜坡对象
 * 参数 step  : 新的步长
 * 返回 无 */
void Slope_SetStep(Slope *slope,float step)
{
    slope->step=step; /* 记录新的步长 */
}

/* 计算下一个斜坡值，更新 slope->value 并返回该值
 * 算法：先求误差 error = 当前值 - 目标值；
 *       1) 若 |error| 小于死区，认为已到位，直接返回当前值，不做变化；
 *       2) 否则若 |error| 小于一个步长，直接对齐到目标值（避免越过后来回摆动）；
 *       3) 否则按误差正负，向目标方向移动一个步长。
 * 参数 slope : 斜坡对象
 * 返回 更新后的当前值 */
float Slope_NextVal(Slope *slope)
{
    float error=slope->value-slope->target; /* 误差：当前值与目标值之差（可正可负） */

    if(fabsf(error)<slope->deadzone)/* 误差落在死区内：视为已到位，当前值保持不变 */
	    return slope->value;

    if(fabsf(error)<slope->step)/* 误差不足一个步长：直接对齐到目标值，防止过冲 */
	    slope->value=slope->target;
    else if(error<0)
	    slope->value+=slope->step;
    else if(error>0)
	    slope->value-=slope->step;
    return slope->value;
}

/* 获取斜坡当前值
 * 参数 slope : 斜坡对象
 * 返回 当前值（读取但不推进斜坡） */
float Slope_GetVal(Slope *slope)
{
    return slope->value;
}
