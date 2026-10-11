/**
 ******************************************************************************
 * @file    kalman filter.h
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   通用卡尔曼滤波器(Kalman Filter)的接口声明。
 *
 *          术语:卡尔曼滤波是一种递归的最优状态估计算法,用"带噪声的观测"去估计
 *          "无法直接测量的系统真实状态"。每个控制周期分两步,循环往复:
 *            (1) 预测(Predict):用上一时刻的状态 + 系统模型,推算出当前时刻的
 *                先验状态,以及它的不确定度(协方差矩阵 P);
 *            (2) 更新(Update):用当前传感器量测去修正先验状态,得到后验(最优)
 *                估计,同时修正不确定度。
 *          两步反复迭代,估计值会逐步收敛到真实值附近。
 *
 *          本实现的特点:
 *            - 基于 CMSIS-DSP 的浮点矩阵库(arm_math)完成全部矩阵运算;
 *            - 支持"量测维度可变":某些周期部分传感器没有新数据时,自动把量测
 *              矩阵 H、观测噪声矩阵 R、卡尔曼增益 K 的维数缩小(降维);
 *            - 提供 7 个用户回调(User_Func0..6_f)和 5 个跳过标志(SkipEq1..5),
 *              可替换/扩展标准流程的任一环节,从而扩展成 EKF/UKF/ESKF/AUKF 等。
 ******************************************************************************
 * @attention
 *          依赖 CMSIS-DSP 的矩阵库与快速数学库;各矩阵的内存由 .c 文件中的
 *          Kalman_Filter_Init 通过 pvPortMalloc(FreeRTOS 堆)动态分配。
 ******************************************************************************
 */
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

// cortex-m4 DSP lib
// 下面这段是 Keil(MDK)+CMSIS-DSP 所需的编译宏;本工程已改用 CMake + GCC 工具链,
// 这些宏由构建系统统一提供,因此整段保留但注释掉,仅作历史参考。
/*
#define __CC_ARM    // Keil
#define ARM_MATH_CM4
#define ARM_MATH_MATRIX_CHECK
#define ARM_MATH_ROUNDING
#define ARM_MATH_DSP    // define in arm_math.h
*/



// 头文件包含
//#include "dsp/matrix_functions.h"            // 与下方重复,保留历史注释行
//#include "math.h"                             // 标准数学库(本实现未直接使用)
#include "stdint.h"                            // 标准整型定义(uint8_t / uint16_t 等)
#include "stdlib.h"                            // 标准库(内存分配相关声明)
//#include "arm_math.h"                         // 旧版 CMSIS-DSP 总头文件,改用按需包含
#include "dsp/matrix_functions.h"            // CMSIS-DSP 矩阵运算库:初始化/加/减/乘/转置/求逆
#include "dsp/fast_math_functions.h"        


// 以下宏把 CMSIS-DSP 的底层函数名简写,便于本文件书写矩阵运算
#define mat arm_matrix_instance_f32          // 矩阵实例类型别名(CMSIS-DSP 浮点矩阵结构体:行列数 + 数据指针)
#define Matrix_Init arm_mat_init_f32         // 矩阵初始化:绑定行数/列数和一个已分配的浮点缓冲区
#define Matrix_Add arm_mat_add_f32           // 矩阵加法:C = A + B(要求 A/B/C 尺寸相同)
#define Matrix_Subtract arm_mat_sub_f32      // 矩阵减法:C = A - B(要求 A/B/C 尺寸相同)
#define Matrix_Multiply arm_mat_mult_f32     // 矩阵乘法:C = A * B(要求 A 的列数等于 B 的行数)
#define Matrix_Transpose arm_mat_trans_f32   // 矩阵转置:C = A 的转置(行列互换)
#define Matrix_Inverse arm_mat_inverse_f32   // 矩阵求逆:C = A 的逆(仅方阵,且需可逆)

typedef struct kf_t
{
    // ===== 对外数据指针(指向 Kalman_Filter_Init 中分配的浮点数组) =====
    float *FilteredValue;         // 滤波输出(即状态估计 xhat),长度 xhatSize,单位随具体状态定义
    float *MeasuredVector;        // 量测输入缓冲 z,长度 zSize;由传感器回调写入,滤波更新后自动清零;元素 0 表示该量测本周期无效
    float *ControlVector;         // 控制输入缓冲 u,长度 uSize;元素 0 通常表示无控制输入

    // ===== 各向量维度(即矩阵行列数),由 Kalman_Filter_Init 指定 =====
    uint8_t xhatSize;             // 状态向量维度 n(例如 3 表示 高度/速度/加速度)
    uint8_t uSize;                // 控制向量维度 m,可为 0(表示无控制输入)
    uint8_t zSize;                // 量测向量维度(可接入的传感器最大数量)

    // ===== 量测自动调整相关状态 =====
    uint8_t UseAutoAdjustment;    // 是否启用"量测维度自动调整":1=启用(按有效量测动态改 H/R/K 维数);0=关闭
    uint8_t MeasurementValidNum;  // 本周期有效量测数量(由 H_K_R_Adjustment 统计);0 表示无有效量测

    uint8_t *MeasurementMap;      // 量测与状态的关系 how measurement relates to the state;第 i 个量测对应状态的第几个分量(从 1 开始计数)
    float *MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H;通常为 1
    float *MatR_DiagonalElements; // 量测方差 variance for each measurement;作为 R 的对角元,越大表示越不信任该量测
    float *StateMinVariance;      // 最小方差 避免方差过度收敛 suppress filter excessive convergence;即 P 对角元的下限
    uint8_t *temp;                // 临时数组,长度 zSize;记录有效量测在原 z 中的下标,供重构 R 使用

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    // SkipEqN 非 0 = 跳过第 N 个黄金五式环节(改由用户函数自行实现);0 = 执行内置实现
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    // 以下每个 mat 都是 CMSIS-DSP 的矩阵实例:只记录行列数 + 数据指针,真实数据存放在下方对应的 *_data 中
    mat xhat;      // x(k|k)       后验(已融合量测)状态估计, n×1
    mat xhatminus; // x(k|k-1)     先验(仅预测未融合)状态估计, n×1
    mat u;         // control vector u 控制向量, m×1
    mat z;         // measurement vector z 量测向量, zSize×1
    mat P;         // covariance matrix P(k|k) 后验状态协方差, n×n,反映估计的不确定度
    mat Pminus;    // covariance matrix P(k|k-1) 先验状态协方差, n×n
    mat F, FT;     // state transition matrix F FT 状态转移矩阵及其转置, n×n
    mat B;         // control matrix B 控制矩阵, n×m
    mat H, HT;     // measurement matrix H 量测矩阵及其转置(维数随有效量测个数变化)
    mat Q;         // process noise covariance matrix Q 过程噪声协方差, n×n(刻画系统模型不确定度)
    mat R;         // measurement noise covariance matrix R 观测噪声协方差(维数随有效量测个数变化)
    mat K;         // kalman gain  K 卡尔曼增益(维数随有效量测个数变化)
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1; // 中间缓存:S=新息协方差,其余为临时矩阵/向量

    int8_t MatStatus;              // 矩阵运算返回状态码(arm_status),用于调试;当前实现未逐次判断

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct kf_t *kf);   // 环节0:取完量测之后调用(自定义量测处理)
    void (*User_Func1_f)(struct kf_t *kf);   // 环节1:先验状态 xhat'(k) 更新后调用(可替换成非线性状态方程)
    void (*User_Func2_f)(struct kf_t *kf);   // 环节2:先验协方差 P'(k) 更新后调用
    void (*User_Func3_f)(struct kf_t *kf);   // 环节3:卡尔曼增益 K 计算后调用
    void (*User_Func4_f)(struct kf_t *kf);   // 环节4:状态融合 xhat(k) 更新后调用
    void (*User_Func5_f)(struct kf_t *kf);   // 环节5:融合/仅预测分支结束后调用(可用于后处理、加约束)
    void (*User_Func6_f)(struct kf_t *kf);   // 环节6:输出滤波值之前的最后一个钩子
    
    // 矩阵存储空间指针(为上面各 mat 提供真实的数据缓冲区,均由 Init 动态分配)
    float *xhat_data, *xhatminus_data;   // xhat / xhatminus 数据区,各 n 个浮点
    float *u_data;                       // u 数据区,m 个浮点(uSize=0 时不分配)
    float *z_data;                       // z 数据区,zSize 个浮点
    float *P_data, *Pminus_data;         // P / Pminus 数据区,各 n*n 个浮点
    float *F_data, *FT_data;             // F / FT 数据区,各 n*n 个浮点
    float *B_data;                       // B 数据区,n*m 个浮点(uSize=0 时不分配)
    float *H_data, *HT_data;             // H / HT 数据区,各 zSize*n 个浮点
    float *Q_data;                       // Q 数据区,n*n 个浮点
    float *R_data;                       // R 数据区,zSize*zSize 个浮点
    float *K_data;                       // K 数据区,n*zSize 个浮点
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1; // S 及各临时矩阵/向量的数据区
} KalmanFilter_t;

extern uint16_t sizeof_float, sizeof_double;   // 全局缓存的 sizeof(float)/sizeof(double)(在 .c 中赋值),避免重复求值

// ---- 初始化与复位 ----
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);   // 按维度分配各矩阵内存并清零,绑定矩阵实例
void Kalman_Filter_Reset(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);  // 仅把已分配的各矩阵/缓冲区清零(不动内存分配)

// ---- 黄金五式(可单独调用,也可由 Kalman_Filter_Update 统一调度) ----
void Kalman_Filter_Measure(KalmanFilter_t *kf);            // 环节0:获取量测,必要时自动调整 H/R/K
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);    // 环节1:先验状态 xhat'(k) = F*xhat(k-1) + B*u
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);       // 环节2:先验协方差 P'(k) = F*P(k-1)*F转置 + Q
void Kalman_Filter_SetK(KalmanFilter_t *kf);               // 环节3:卡尔曼增益 K = P'*H转置 / (H*P'*H转置 + R)
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);         // 环节4:后验状态融合 xhat(k) = xhat'(k) + K*(z - H*xhat'(k))
void Kalman_Filter_P_Update(KalmanFilter_t *kf);           // 环节5:后验协方差 P(k) = P'(k) - K*H*P'(k)

// ---- 一站式更新 ----
float *Kalman_Filter_Update(KalmanFilter_t *kf);           // 依次执行黄金五式并抑制过度收敛,返回滤波输出 FilteredValue

#endif //__KALMAN_FILTER_H
