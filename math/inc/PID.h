#ifndef _USER_PID_H_
#define _USER_PID_H_

#include "stdint.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _PID
{
    float kp,ki,kd;
    float error,lastError;//误差、上次误差
    float integral,maxIntegral;//积分、积分限幅
    float output,maxOutput;//输出、输出限幅
    float deadzone;//死区
}PID;

/*串级结构体和控制函数*/
typedef struct _DEPID
{
  float kp;     //比例系数
  float ki;      //积分系数
  float kd;    //微分系数
  float lasterror;     //前一次偏差
    float error;				//当前error
  float output;     //输出值
  float integral;   //积分值
  float derivative;      //微分值
  float lastPv;     //前一次的测量值
  float gama;      //微分项滤波系数
    float maxOutput; //输出限幅
    float maxIntegral;//积分限幅
}DEPID;

typedef struct _CascadePID
{
    PID inner;//内环
    PID outer;//外环
    DEPID deOuter;//外环微分先行
    float output;//输出，等于inner.output
}CascadePID;

typedef struct
{
    float kp;            // 位置刚度
    float kd;            // 速度阻尼

    float deadzone;      // 死区

    float torque_ff;     // 前馈力矩 T_ff (Nm)

    float maxTorque;     // 最大输出力矩
    float outputTorque;  // 输出力矩 T_ref
} PD_Controller;


void PID_Init(PID *pid,float p,float i,float d,float maxSum,float maxOut);
void PID_SingleCalc(PID *pid,float reference,float feedback);
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void PID_Clear(PID *pid);
void DEPID_Clear(DEPID *pid);
void PID_SetMaxOutput(PID *pid,float maxOut);
void PID_SetDeadzone(PID *pid,float deadzone);
void DEPID_Init(DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama);
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation);
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void PD_Init(PD_Controller *pd,float kp,float kd,float maxTorque);
void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas);

#endif
