#ifndef _USER_PID_H_ // 头文件包含保护：首次包含时定义 _USER_PID_H_，再次包含将被 #ifndef 跳过
#define _USER_PID_H_ // 包含保护宏定义(与文件末尾 #endif 配对)

#include "stdint.h" // 标准整型定义；本文件未直接使用，主要为兼容既有包含关系保留
/* ============================================================================
 * PID.h —— PID 控制器数学库(位置式 / 微分先行 / 串级 / 并级 PD) 数据结构与接口
 * ----------------------------------------------------------------------------
 * 本文件声明供底盘、云台、拨弹等控制环复用的 PID 相关结构体与函数原型。
 *
 * 术语速览：
 *   - 偏差 error：设定值(reference)与反馈值(feedback)之差，单位随被控量而定。
 *   - 位置式 PID：输出 = 比例项 + 积分项 + 微分项(此处微分采用增量式 error - lastError)，
 *     对应 PID / PID_SingleCalc。
 *   - 微分先行 PID：微分只作用于反馈(测量)量，避免设定值突变引起微分冲击，
 *     对应 DEPID / PIDRegulation。
 *   - 串级 PID：外环(角度)输出作为内环(速度)的设定值，两层串联，
 *     对应 CascadePID / PID_CascadeCalc / DEPID_CascadeCalc。
 *   - 并级 PD：位置刚度与速度阻尼并联求和(外加前馈力矩)，用于力矩直驱，
 *     对应 PD_Controller / PD_ParallelCalc。
 * ========================================================================== */

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x))) // 区间限幅宏：把左值 x 就地钳位到 [min,max]（x<=min 取 min；x>=max 取 max；否则不变）。注意 x 会被多次求值且被修改，勿传入带副作用的表达式

typedef struct _PID // 位置式(单级) PID 结构体：比例/积分/微分三项作用于同一个偏差 error
{
    float kp,ki,kd; // kp 比例、ki 积分、kd 微分 三个系数(单位随被控量而定，由 PID_Init 设置)
    float error,lastError; // error 本拍偏差(=设定值-反馈值)；lastError 上一拍偏差(用于增量式微分)
    float integral,maxIntegral; // integral 积分累加值；maxIntegral 积分限幅(正负对称上限，防积分饱和)
    float output,maxOutput; // output 本拍控制输出；maxOutput 输出限幅(正负对称上限，钳位最终输出)
    float deadzone; // deadzone 输入死区：|偏差|小于该值时按 0 处理，抑制小幅抖动(默认 0)
}PID; // PID 结构体类型别名

/* 微分先行(DEPID) PID 结构体：微分项只对测量值微分，避免设定值跳变引起微分冲击 */
/* 该结构常与串级配合，外环(角度)使用微分先行可提升云台稳定性 */
typedef struct _DEPID // 微分先行 PID 结构体定义
{
  float kp; // 比例系数 kp
  float ki; // 积分系数 ki
  float kd; // 微分系数 kd
  float lasterror; // lasterror 前一拍偏差(保留，供扩展/调试用)
    float error; // error 当前偏差
  float output; // output 本拍输出值
  float integral; // integral 积分累加值
  float derivative; // derivative 微分项(滤波后的微分，仅调试用)
  float lastPv; // lastPv 前一拍的测量值(参与微分滤波的历史状态)
  float gama; // gama 微分滤波系数(0~1)：越大越平滑、相位滞后越大；滤波式 d = gama×上次滤波值 + (1-gama)×测量值
    float maxOutput; // maxOutput 输出限幅(正负对称上限)
    float maxIntegral; // maxIntegral 积分限幅(正负对称上限)
}DEPID; // DEPID 结构体类型别名

typedef struct _CascadePID // 串级 PID 结构体：外环(角度)输出作为内环(速度)的设定值
{
    PID inner;  // inner 内环 PID(通常为速度环)
    PID outer;  // outer 外环 PID(通常为角度环)
    DEPID deOuter;  // deOuter 外环的微分先行版本(与 outer 二选一使用)
    float output;  // output 串级最终输出，等于 inner.output
}CascadePID; // CascadePID 结构体类型别名

typedef struct // 并级 PD 结构体(位置刚度+速度阻尼+前馈力矩)，常用于力矩直驱控制
{
    float kp; // 位置刚度 kp(单位 Nm/rad 或与位置量纲匹配)
    float kd; // 速度阻尼 kd(单位 Nm/(rad/s) 或与速度量纲匹配)

    float deadzone; // 死区：位置/速度误差绝对值小于该值时按 0 处理

    float torque_ff; // 前馈力矩 torque_ff(单位 Nm)：直接叠加的期望力矩，用于加速/重力补偿

    float maxTorque; // 最大力矩限制 maxTorque(单位 Nm，正负对称上限)
    float outputTorque; // 输出力矩 outputTorque(单位 Nm，T_ref，供电机力矩指令使用)
} PD_Controller; // PD_Controller 结构体类型别名


void PID_Init(PID *pid,float p,float i,float d,float maxSum,float maxOut); // 初始化位置式 PID：p/i/d 系数、maxI 积分限幅(±)、maxOut 输出限幅(±)
void PID_SingleCalc(PID *pid,float reference,float feedback); // 位置式(单级) PID 单次计算：输入设定值 reference 与反馈值 feedback
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb); // 串级 PID 计算：angleRef/angleFdb 为外环(角度)，speedFdb 为内环(速度)反馈
void PID_Clear(PID *pid); // 清空位置式 PID 的历史状态(error/lastError/integral/output)
void DEPID_Clear(DEPID *pid); // 清空微分先行 PID 的历史状态(含 lastPv)
void PID_SetMaxOutput(PID *pid,float maxOut); // 重新设置位置式 PID 的输出限幅
void PID_SetDeadzone(PID *pid,float deadzone); // 设置位置式 PID 的死区
void DEPID_Init(DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama); // 初始化微分先行 PID：额外传入 gama 微分滤波系数
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation); // 微分先行 PID 单次计算：differentiation 为传入的微分源(通常为 -speedFdb)
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb); // 串级微分先行 PID 计算：外环用 DEPID、内环用普通 PID
void PD_Init(PD_Controller *pd,float kp,float kd,float maxTorque); // 初始化并级 PD：kp 位置刚度、kd 速度阻尼、maxTorque 力矩限幅(±)
void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas); // 并级 PD 计算：输入期望/测量位置与速度，输出力矩到 outputTorque

#endif // 结束包含保护
