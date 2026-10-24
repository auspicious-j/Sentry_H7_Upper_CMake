/****************PID函数****************/

#include "PID.h"

#include <math.h>

//初始化pid函数
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
    pid->kp=p;
    pid->ki=i;
    pid->kd=d;
    pid->maxIntegral=maxI;
    pid->maxOutput=maxOut;
    pid->deadzone=0;
}

//初始化微分先行pid函数
void DEPID_Init(DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama)
{
    pid->kp=p;
    pid->ki=i;
    pid->kd=d;
    pid->maxIntegral=maxI;
    pid->maxOutput=maxOut;
    pid->gama=gama;
}

void PD_Init(PD_Controller *pd,float kp,float kd,float maxTorque)
{
    pd->kp = kp;
    pd->kd = kd;

    pd->deadzone = 0.0f;
    pd->torque_ff = 0.0f;

    pd->maxTorque = maxTorque;

    pd->outputTorque = 0.0f;
}


//计算微分先行pid函数
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation)
{
	//计算误差
    vPID->lasterror=vPID->error;
    vPID->error=reference-feedback;
	//微分滤波
    differentiation = vPID->gama * differentiation + (1-vPID->gama) * vPID-> lastPv;
	//计算微分
    vPID->output = differentiation * vPID->kd;
	//计算比例项
    vPID->output+=vPID->error*vPID->kp;
	//计算积分项
    vPID->integral+=vPID->error*vPID->ki;
    LIMIT(vPID->integral,-vPID->maxIntegral,vPID->maxIntegral);//积分限幅
    vPID->output+=vPID->integral;
	//输出限幅
    LIMIT(vPID->output,-vPID->maxOutput,vPID->maxOutput);
	//保存微分滤波
    vPID-> lastPv = differentiation;
}

//计算pid函数
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//计算误差
    pid->lastError=pid->error;
    if(fabsf(reference-feedback) < pid->deadzone)//误差在死区内，error直接置0
	    pid->error=0;
    else
	    pid->error=reference-feedback;
	//计算微分
    pid->output=(pid->error-pid->lastError)*pid->kd;
	//计算比例项
    pid->output+=pid->error*pid->kp;
	//计算积分项
    pid->integral+=pid->error*pid->ki;
    LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
    pid->output+=pid->integral;
	//输出限幅
    LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

//计算pid函数
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
    PID_SingleCalc(&(pid->outer),angleRef,angleFdb);//计算外环(角度环)
    PID_SingleCalc(&(pid->inner),pid->outer.output ,speedFdb);//计算内环(速度环)
    pid->output=pid->inner.output;
}

//计算微分先行pid函数		在哨兵上台 TODO：待验证
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
    PIDRegulation(&(pid->deOuter),angleRef,angleFdb,-speedFdb);//计算外环微分先行(角度环)
    PID_SingleCalc(&(pid->inner),pid->deOuter.output ,speedFdb);//计算内环(速度环)
    pid->output=pid->inner.output;
}


void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas)
{
    float pos_err;
    float vel_err;
    /* 位置误差 */
    pos_err = p_des - p_meas;
    if (fabsf(pos_err) < pd->deadzone)
        pos_err = 0.0f;
    /* 速度误差 */
    vel_err = v_des - v_meas;
    if (fabsf(vel_err) < pd->deadzone)
        vel_err = 0.0f;
    /* 计算 PD + 前馈 */
    pd->outputTorque = pd->kp * pos_err + pd->kd * vel_err + pd->torque_ff;
    /* 输出限幅 */
    LIMIT(pd->outputTorque,-pd->maxTorque,pd->maxTorque);
}

//清空一次pid的历史数据
void PID_Clear(PID *pid)
{
    pid->error=0;
    pid->lastError=0;
    pid->integral=0;
    pid->output=0;
}

void DEPID_Clear(DEPID *pid)
{
    pid->error=0;
    pid->lasterror=0;
    pid->integral=0;
    pid->output=0;
    pid->lastPv=0;
}



//用于设定pid输出限幅
void PID_SetMaxOutput(PID *pid,float maxOut)
{
    pid->maxOutput=maxOut;
}

//设置PID死区
void PID_SetDeadzone(PID *pid,float deadzone)
{
    pid->deadzone=deadzone;
}
