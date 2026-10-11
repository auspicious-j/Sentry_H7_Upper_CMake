/**
 ******************************************************************************
 * @file    USER_Moto.h
 * @brief   电机驱动接口层头文件：大疆(DJI)电机与达妙(DM)电机的数据结构、量程宏与函数声明
 ******************************************************************************
 * @details 本文件提供：
 *          1. “编码器值 <-> 角度(°)”换算宏（M3508 / M2006 / M6020）；
 *          2. 达妙 MIT 协议的量程宏（位置/速度/Kp/Kd/力矩）与模式偏移宏；
 *          3. 电机数据结构体：DJI_Motor_t（大疆）与 DM_motor_t（达妙）；
 *          4. 舵电机结构体 Double_motor_t、电机状态结构体 motor_state_t、
 *             达妙反馈参数结构体 motor_fbpara_t；
 *          5. 浮点/整型换算函数，以及电机收发、反馈解码、累计角度相关函数的声明。
 ******************************************************************************
 */
#ifndef _USER_MOTO_H_
#define _USER_MOTO_H_

#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "PID.h"

/* 各种电机“编码器值 <-> 角度(°)”换算宏
 * 说明：编码值 = 电机编码器原始读数；M3508/M2006/M6020 编码器单圈均为 0~8191，
 *       对应 0~360°，即 8191/360 ≈ 22.7528 计数每度。
 *       减速比把输出轴角度换算到编码器侧，故系数 = 减速比 × 8191/360。
 *       DGR2CODE：角度(°) 转编码值；CODE2DGR：编码值 转角度(°)。 */
#define MOTOR_M3508_DGR2CODE(dgr) ((int32_t)((dgr)*436.9263f)) // 角度(°)转编码值：系数 436.9263 = (3591/187) × (8191/360)，M3508 减速比 3591/187 ≈ 19.2:1
#define MOTOR_M3508_CODE2DGR(code) ((float)((code)/436.9263f)) // 编码值转角度(°)：上式的逆运算（除以同一系数）

#define MOTOR_M2006_DGR2CODE(dgr) ((int32_t)((dgr)*819.1f)) // 角度(°)转编码值：系数 819.1 = 36 × (8191/360)，M2006 减速比 36:1
#define MOTOR_M2006_CODE2DGR(code) ((float)((code)/819.1f)) // 编码值转角度(°)

#define MOTOR_M6020_DGR2CODE(dgr) ((int32_t)((dgr)*22.7528f)) // 角度(°)转编码值：系数 22.7528 = 8191/360，M6020 直连无减速比
#define MOTOR_M6020_CODE2DGR(code) ((float)((code)/22.7528f)) // 编码值转角度(°)

/* 达妙电机控制模式偏移：实际发送的 CAN ID = 电机 ID + 模式偏移 */
#define MIT_MODE     0x000 // MIT 模式：位置/速度/前馈力矩混合控制
#define POS_MODE    0x100 // 位置模式
#define SPD_MODE    0x200 // 速度模式
#define PSI_MODE    0x300 // 位置-速度（力位混合）模式

/* 达妙 MIT 协议各物理量的量程上下限：float_to_uint / uint_to_float 编码解码时使用 */
#define P_MIN -3.14159265f // 目标位置下限(rad)，约 -180°
#define P_MAX 3.14159265f // 目标位置上限(rad)，约 +180°
#define V_MIN -30.0f // 目标速度下限(rad/s)
#define V_MAX 30.0f // 目标速度上限(rad/s)
#define KP_MIN 0.0f // 位置增益 Kp 下限（为 0 表示不使用位置刚度）
#define KP_MAX 500.0f // 位置增益 Kp 上限
#define KD_MIN 0.0f // 速度增益 Kd 下限（为 0 表示不使用速度阻尼）
#define KD_MAX 5.0f // 速度增益 Kd 上限
#define T_MIN -10.0f // 前馈力矩下限(N·m)
#define T_MAX 10.0f // 前馈力矩上限(N·m)

/* 大疆(DJI)电机结构体：保存电机反馈、控制目标、累计角度与 PID 调节器 */
typedef struct DJIMOTOR
{
    int16_t angle, speed, torque; // 编码器机械角度(原始值) / 转子转速(rpm) / 实际转矩电流(原始值)
    int8_t temp; // 电机温度(℃)

    int16_t lastAngle; // 上一次的编码器角度，用于差分计算角度增量

    int16_t targetSpeed; // 目标转速(rpm，速度环给定值)
    int32_t targetAngle; // 目标角度(编码器值，位置环给定值)

    int32_t totalAngle; // 累计转过的编码器值（含多圈，正负表示方向）

    int16_t targetCurrent; // 目标转矩电流（-16384~16384 对应 -20A~20A）

    uint8_t ERRORFLAG; // 电机错误标志（0=正常，非 0=异常，具体含义见大疆协议）

    PID speedPID;		 // 速度pid(单级)
    CascadePID anglePID; // 角度pid，串级
} DJI_Motor_t;

/* 达妙电机反馈参数：*_int 为 CAN 报文位域打包的整型量，pos/vel/tor/Kp/Kd 为按量程还原后的物理量 */
typedef struct
{
    int id;                // 电机内部设置的 can id
    int state;             // 电机状态
    int p_int;             // 整型位置信息
    int v_int;             // 整型速度信息
    int t_int;             // 整型扭矩信息
    int kp_int;            // 整型Kp信息
    int kd_int;            // 整型Kd信息
    float pos;             // 最终解析出来的位置信息
    float vel;             // 最终解析出来的速度信息
    float tor;             // 最终解析出来的扭矩信息
    float Kp;              // 最终解析出来的Kp数据
    float Kd;              // 最终解析出来的Kd数据
    float Tmos;            // 板子MOS温度
    float Tcoil;           // 电机线圈温度
} motor_fbpara_t;

/* 达妙(DM)电机结构体：保存控制模式、反馈解析结果、多圈累计角度与 PID 调节器 */
typedef struct
{
	    uint16_t mode;          // 电机控制模式
        motor_fbpara_t para;  	// 电机的反馈信息结构体

	    float totalAngle;				//电机旋转总角度 单位°
	    float lastAngle;				//电机上一时刻角度 单位°
	    float nowAngle;					//电机当前角度 单位°
	    float targetTurnAngle;  //电机目标旋转角度  单位°

	    PID speedPID;					  // 速度pid(单级)
	    CascadePID anglePID;	  // 角度pid(串级)

} DM_motor_t;

/* 舵(转向)电机结构体：用于底盘转向舵，内含校准值、多圈角度与轮电机转速。
 * 字段含义：
 *   TurnOffset       舵电机校准值(°)，上电标定得到的零点偏差
 *   now_angle        校准后的当前角度(°)
 *   TurnAngle        当前实际角度(°)
 *   lastAngle        上一时刻的多圈角度(°)，用于差分
 *   totalAngle       多圈累计总角度(°)
 *   targetTurnAngle  需要旋转到的目标角度(°)
 *   targetDriveSpeed 轮电机目标转速(给定值)
 *   now_Speed        轮电机当前转速 */
typedef struct
{
    float TurnOffset; //舵电机校准值			单位为° //舵电机校准值		    单位为°
    float now_angle; //舵电机校准后当前角度	单位为° //舵电机校准后当前角度    单位为°
    float TurnAngle; //舵电机当前实际角度	单位为° //舵电机当前实际角度    单位为°
    float lastAngle; //舵电机当前多圈角度	单位为° //舵电机当前多圈角度    单位为°
    float totalAngle; //舵电机总角度(多圈) 	单位为° //舵电机总角度(多圈)     单位为°
    float targetTurnAngle; //需要旋转的角度		单位为° //需要旋转的角度	    单位为°

    int32_t targetDriveSpeed;			//轮电机转速
    int16_t now_Speed; // 轮电机当前转速
} Double_motor_t;

/* 电机状态结构体：用于上报单个电机的温度与错误状态 */
typedef struct state{
    int8_t temperature; // 电机温度(℃)
    uint8_t error_state; // 电机错误码（0=正常，非 0=异常）
}motor_state_t;

/* ========== 浮点 <-> 整型换算（达妙 MIT 协议编解码） ========== */
// 整型编码值按量程线性还原为浮点物理量
float uint_to_float(int x_int, float x_min, float x_max, int bits);
// 浮点物理量按量程线性编码为整型
int float_to_uint(float x_float, float x_min, float x_max, int bits);

/* ========== 反馈更新与指令发送接口 ========== */
// 在 CAN 接收回调中调用：把一帧反馈写入大疆电机结构体
void DJIMotor_Update(DJI_Motor_t *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp);
// 解析达妙 4310 的 8 字节反馈帧（MIT 位域打包），还原位置/速度/力矩/温度
void dm4310_fbdata(DM_motor_t *motor, uint8_t *rx_data);
// 使能达妙电机（帧尾 0xFC）
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
// 失能达妙电机（帧尾 0xFD）
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
// 达妙 MIT 模式混合控制：位置/速度/增益/前馈力矩（tau = kp*e_pos + kd*e_vel + torq）
void mit_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
// 大疆电机电流控制：一帧 8 字节同时控制 4 个电机的转矩电流
void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
// 初始化大疆电机累计角度（记录零点与差分基准）
void Motor_StartCalcAngle(DJI_Motor_t *motor);
// 计算大疆电机累计角度（含跨编码器零点绕圈识别）
void Motor_CalcAngle(DJI_Motor_t *motor);
// 初始化达妙舵电机累计角度
void Motor_StartCalcAngle_J4310(Double_motor_t *motor);
// 计算达妙舵电机累计角度（含跨零点绕圈识别）
void Motor_CalcAngle_J4310(Double_motor_t *motor);
// 清除达妙电机错误（帧尾 0xFB）
void clear_err(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, uint16_t mode_id);

#endif
