/*
 * Slope.h —— 斜率（斜坡）限制模块
 * 功能：把一个突变的“目标值”转换为按固定步长逐步逼近的“当前值”，
 *       从而限制信号（如速度、电流给定）的变化率，避免阶跃冲击。
 * 术语：
 *   - 斜率/斜坡（Slope）：数值每帧按最大步长变化，而不是瞬间跳变。
 *   - 步长（step）      ：单次调用允许增加/减少的最大幅度。
 *   - 死区（deadzone）  ：当前值与目标值之差小于该阈值时视为“已到位”，不再增减。
 */
#ifndef _SLOPE_H_
#define _SLOPE_H_

#include "main.h"

/* 斜坡结构体：描述一个受变化率限制的标量 */
typedef struct{
    float target;   /* 目标值：期望最终达到的数值（单位随被控量而定） */
    float step;     /* 步长：单次调用允许的最大变化量（>0，默认值由 Slope_Init 传入） */
    float value;    /* 当前值：斜坡当前输出，随每次调用逐步逼近 target（初始为 0） */
    float deadzone; /* 死区：|value-target| 小于该值时停止增减（>=0，防止在目标附近抖动） */
}Slope;

/* 初始化：设定步长与死区，并把 target/value 清零
 * 参数 slope    : 待初始化的斜坡对象
 * 参数 step     : 步长（单次最大变化量）
 * 参数 deadzone : 死区阈值
 * 返回 无 */
void Slope_Init(Slope *slope,float step,float deadzone);
/* 设定目标值：只更新目标，当前值需靠 Slope_NextVal 逐步跟上
 * 参数 slope  : 斜坡对象
 * 参数 target : 新的目标值
 * 返回 无 */
void Slope_SetTarget(Slope *slope,float target);
/* 修改步长：运行中动态调整变化率
 * 参数 slope : 斜坡对象
 * 参数 step  : 新的步长
 * 返回 无 */
void Slope_SetStep(Slope *slope,float step);
/* 推进一步：按步长向目标逼近，更新并返回当前值
 * 参数 slope : 斜坡对象
 * 返回 更新后的当前值 */
float Slope_NextVal(Slope *slope);
/* 读取当前值（不推进）
 * 参数 slope : 斜坡对象
 * 返回 当前值 */
float Slope_GetVal(Slope *slope);

#endif
