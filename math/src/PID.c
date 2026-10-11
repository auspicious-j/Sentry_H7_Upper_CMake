/* ============================================================================
 * PID.c —— PID 控制器实现(位置式 / 微分先行 / 串级 / 并级 PD)
 * ----------------------------------------------------------------------------
 * 实现 PID.h 中声明的控制器算法，供底盘、云台、拨弹等控制环调用。
 * 约定：误差 error = reference - feedback(设定值减反馈值)。
 * 抗积分饱和：每次累加积分后立即做积分限幅，避免积分项无限增大导致超调。
 * 输出限幅：最终输出钳位到 ±maxOutput，保护执行机构。
 * 说明：#include <math.h> 提供 fabsf() 浮点绝对值。
 * ========================================================================== */

#include "PID.h" // 本模块对外接口声明

#include <math.h> // fabsf() 等浮点数学函数

// 初始化位置式(单级) PID 参数
// p/i/d：比例、积分、微分系数；maxI：积分限幅(±)；maxOut：输出限幅(±)；deadzone 初始为 0
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut) // 初始化各系数与限幅(死区清 0，需后续用 PID_SetDeadzone 设置)
{
    pid->kp=p; // 比例系数
    pid->ki=i; // 积分系数
    pid->kd=d; // 微分系数
    pid->maxIntegral=maxI; // 积分限幅(±maxIntegral)
    pid->maxOutput=maxOut; // 输出限幅(±maxOutput)
    pid->deadzone=0; // 死区默认 0
}

// 初始化微分先行(DEPID) PID 参数
// 在普通 PID 基础上增加 gama：微分滤波系数(0~1，越大越平滑)
void DEPID_Init(DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama) // 初始化各系数、限幅与微分滤波系数
{
    pid->kp=p; // 比例系数
    pid->ki=i; // 积分系数
    pid->kd=d; // 微分系数
    pid->maxIntegral=maxI; // 积分限幅(±maxIntegral)
    pid->maxOutput=maxOut; // 输出限幅(±maxOutput)
    pid->gama=gama; // 微分滤波系数
}

void PD_Init(PD_Controller *pd,float kp,float kd,float maxTorque) // 初始化并级 PD：kp 位置刚度、kd 速度阻尼、maxTorque 力矩限幅(±)
{
    pd->kp = kp; // 位置刚度
    pd->kd = kd; // 速度阻尼

    pd->deadzone = 0.0f; // 死区清零
    pd->torque_ff = 0.0f; // 前馈力矩清零

    pd->maxTorque = maxTorque; // 力矩限幅(±maxTorque)

    pd->outputTorque = 0.0f; // 输出力矩初始为 0
}


// 微分先行 PID 单次计算
// reference 设定值；feedback 反馈值；differentiation 传入的微分源(通常为 -speedFdb)
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation) // 计算流程：更新偏差→微分滤波→微分项→比例项→积分项(限幅)→输出限幅
{
	// 更新偏差
    // 保存上一拍偏差，并计算本拍偏差 error = reference - feedback
    vPID->lasterror=vPID->error; // 上一拍偏差 = 本拍偏差(滚动保存)
    vPID->error=reference-feedback; // 本拍偏差 = 设定值 - 反馈值
	// 微分滤波
    // 一阶低通：本次微分 = gama×上次滤波值 + (1-gama)×测量值，抑制微分噪声
    differentiation = vPID->gama * differentiation + (1-vPID->gama) * vPID-> lastPv; // differentiation 更新为滤波结果(历史量取自 lastPv)
	// 计算微分
    // 微分项 = 滤波后微分 × kd
    vPID->output = differentiation * vPID->kd; // 微分项输出
	// 计算比例
    // 比例项 = error × kp，累加到输出
    vPID->output+=vPID->error*vPID->kp; // 累加比例项
	// 计算积分
    // 积分累加 = 累加 error × ki
    vPID->integral+=vPID->error*vPID->ki; // 积分累加
    LIMIT(vPID->integral,-vPID->maxIntegral,vPID->maxIntegral);  // 积分限幅：抗积分饱和，避免积分项无限增大导致超调
    vPID->output+=vPID->integral; // 把积分项并入输出
	// 输出限幅
    // 最终输出钳位到 ±maxOutput，保护执行机构
    LIMIT(vPID->output,-vPID->maxOutput,vPID->maxOutput); // 输出限幅
	// 更新微分滤波
    // 保存本次滤波结果，供下一拍作历史(lastPv)
    vPID-> lastPv = differentiation; // 更新 lastPv 状态
}

// 位置式(单级) PID 单次计算
// reference 设定值；feedback 反馈值；微分采用增量式(error-lastError)
void PID_SingleCalc(PID *pid,float reference,float feedback) // 入口：更新偏差 → 死区 → 微分/比例/积分 → 输出限幅
{
	// 更新偏差
    pid->lastError=pid->error; // 保存上一拍偏差
    if(fabsf(reference-feedback) < pid->deadzone) // 偏差落在死区内则直接置 0(抑制抖动)
	    pid->error=0; // 死区内：偏差按 0 处理
    else
	    pid->error=reference-feedback; // 死区外：偏差 = 设定值 - 反馈值
	// 计算微分
    pid->output=(pid->error-pid->lastError)*pid->kd; // 微分项 = (error - lastError) × kd(对偏差差分，即增量式微分)
	// 计算比例
    pid->output+=pid->error*pid->kp; // 累加比例项 = error × kp
	// 计算积分
    pid->integral+=pid->error*pid->ki; // 积分累加 = 累加 error × ki
    LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral); // 积分限幅，抗积分饱和
    pid->output+=pid->integral; // 并入积分项
	// 输出限幅
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput); // 最终输出钳位到 ±maxOutput
}

// 串级 PID 计算
// angleRef/angleFdb：外环(角度)设定与反馈；speedFdb：内环(速度)反馈
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb) // 外环输出作为内环设定值
{
    PID_SingleCalc(&(pid->outer),angleRef,angleFdb);  // 计算外环(角度环)：输入角度设定与反馈
    PID_SingleCalc(&(pid->inner),pid->outer.output ,speedFdb);  // 计算内环(速度环)：设定值取外环输出，反馈为速度
    pid->output=pid->inner.output; // 串级输出 = 内环输出
}

// 串级微分先行 PID 计算(多用于云台)
// TODO：拨弹等其他电机也可考虑采用
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb) // 外环用微分先行，内环用普通 PID
{
    PIDRegulation(&(pid->deOuter),angleRef,angleFdb,-speedFdb);  // 计算外环微分先行(角度环)：以 -speedFdb 作微分源
    PID_SingleCalc(&(pid->inner),pid->deOuter.output ,speedFdb); // 计算内环(速度环)：设定值取外环输出
    pid->output=pid->inner.output; // 串级输出 = 内环输出
}


// 并级 PD 计算：位置刚度与速度阻尼并联求和，叠加前馈力矩后做力矩限幅
// p_des/v_des：期望位置/速度；p_meas/v_meas：测量位置/速度；结果写入 outputTorque
void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas)
{
    float pos_err; // 位置误差
    float vel_err; // 速度误差
    /* 位置误差 = 期望位置 - 测量位置 */
    /* 位置死区：误差绝对值小于 deadzone 时按 0 处理 */
    pos_err = p_des - p_meas; // 位置误差 = 期望位置 - 测量位置
    if (fabsf(pos_err) < pd->deadzone)
        pos_err = 0.0f;
    /* 速度误差 = 期望速度 - 测量速度 */
    /* 速度死区 */
    vel_err = v_des - v_meas; // 速度误差 = 期望速度 - 测量速度
    if (fabsf(vel_err) < pd->deadzone)
        vel_err = 0.0f;
    /* 并级 PD + 前馈力矩 */
    /* 输出力矩 = kp×位置误差 + kd×速度误差 + torque_ff */
    pd->outputTorque = pd->kp * pos_err + pd->kd * vel_err + pd->torque_ff; // 位置刚度项 + 速度阻尼项 + 前馈项
    /* 力矩限幅 */
    /* 钳位到 ±maxTorque，保护电机 */
    LIMIT(pd->outputTorque,-pd->maxTorque,pd->maxTorque); // 力矩限幅
}

// 清空位置式 PID 历史状态
// 复位 error/lastError/integral/output，便于重新起步或模式切换
void PID_Clear(PID *pid)
{
    pid->error=0; // 清空本拍偏差
    pid->lastError=0; // 清空上一拍偏差
    pid->integral=0; // 清空积分累加
    pid->output=0; // 清空输出
}

// 清空微分先行 PID 历史状态
// 复位 error/lasterror/integral/output/lastPv
void DEPID_Clear(DEPID *pid)
{
    pid->error=0; // 清空本拍偏差
    pid->lasterror=0; // 清空上一拍偏差
    pid->integral=0; // 清空积分累加
    pid->output=0; // 清空输出
    pid->lastPv=0; // 清空微分滤波历史
}



// 重新设置位置式 PID 输出限幅
// maxOut：输出限幅上限(输出被钳位到 ±maxOut)
void PID_SetMaxOutput(PID *pid,float maxOut) // 设置输出限幅
{
    pid->maxOutput=maxOut; // 写入新的输出限幅
}

// 设置位置式 PID 死区
// deadzone：输入偏差死区(|error| 小于该值按 0 处理)
void PID_SetDeadzone(PID *pid,float deadzone) // 设置死区
{
    pid->deadzone=deadzone; // 写入新的死区值
}
