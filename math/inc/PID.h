#ifndef _USER_PID_H_
#define _USER_PID_H_

#include "stdint.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _PID
{
    float kp,ki,kd;
    float error,lastError;//���ϴ����
    float integral,maxIntegral;//���֡������޷�
    float output,maxOutput;//���������޷�
    float deadzone;//����
}PID;

/*����ṹ��͹�����*/
typedef struct _DEPID
{
  float kp;     //����ϵ��
  float ki;      //����ϵ��
  float kd;    //΢��ϵ��
  float lasterror;     //ǰһ��ƫ��
    float error;				//��ǰerror
  float output;     //���ֵ
  float integral;   //����ֵ
  float derivative;      //΢����
  float lastPv;     //ǰһ�ĵĲ���ֵ
  float gama;      //΢�������˲�ϵ��
    float maxOutput; //����޷�
    float maxIntegral;//�����޷�
}DEPID;

typedef struct _CascadePID
{
    PID inner;//�ڻ�
    PID outer;//�⻷
    DEPID deOuter;//�⻷΢������
    float output;//�������������inner.output
}CascadePID;

typedef struct
{
    float kp;            // λ�øն�
    float kd;            // �ٶ�����

    float deadzone;      // ����

    float torque_ff;     // ǰ������ T_ff (Nm)

    float maxTorque;     // �����������
    float outputTorque;  // ������� T_ref
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
