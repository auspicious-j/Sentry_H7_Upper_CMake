/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 *          基于四元数的扩展卡尔曼滤波(EKF)姿态解算,融合陀螺仪与加速度计
 ******************************************************************************
 * @attention
 *
 * 术语说明:
 *   - EKF(扩展卡尔曼滤波): 当系统/观测模型为非线性时,先在工作点对非线性函数做一阶
 *                          线性化(求雅可比矩阵),再套用标准卡尔曼滤波公式的滤波方法。
 *   - 四元数 q0..q3: 描述三维旋转的单位四元数(q0 为实部,q1..q3 为虚部),无万向锁。
 *   - 雅可比矩阵 H / F: 非线性观测函数 h(x)、状态转移函数 f(x) 对状态求偏导得到的矩阵。
 *
 * 状态量(6 维): x = [q0 q1 q2 q3  bx by]^T
 *   q0..q3 : 姿态四元数(单位四元数,模长恒为 1)
 *   bx, by : 陀螺仪 x/y 轴零偏估计(单位 rad/s);z 轴零偏不估计(载体直立时 yaw 不可观测)
 * 观测量(3 维): z = 归一化后的重力加速度方向向量(由加速度计测得)
 * 输出量: Roll / Pitch / Yaw 欧拉角(单位 度) 及 GyroBias 零偏估计
 *
 * 本文件仅为接口声明;解算算法的实现见同目录的 QuaternionEKF.c。
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "kalman_filter.h" /* 依赖通用卡尔曼滤波数据结构与运算(KalmanFilter_t、Matrix_* 等) */

/* QEKF_INS_t: 四元数 EKF 姿态解算器的完整运行状态与可调参数集合。
 * 单位约定: 角度为 度(deg),角速度为 rad/s,加速度为 m/s^2,时间为 s。
 * 全局唯一实例为 QEKF_INS,定义在 QuaternionEKF.c 中。 */
typedef struct
{
    uint8_t Initialized;              // 初始化完成标志: 0=未初始化, 1=已初始化
    KalmanFilter_t IMU_QuaternionEKF; // 底层通用滤波器对象(6 维状态 / 3 维量测),承载 F、P、Q、R、K、H 等矩阵
    uint8_t ConvergeFlag;             // 收敛标志: 1=残差足够小(已收敛), 0=尚未收敛或已判定发散
    uint8_t StableFlag;               // 平稳标志: 1=角速度小且加速度模长接近 9.8 m/s^2(可信), 0=处于机动
    uint64_t ErrorCount;              // 连续未通过卡方检验的计数;载体静止却持续异常超过 50 次即判定发散
    uint64_t UpdateCount;             // 累计更新次数(第 1 次用于初始化加速度低通滤波)

    float q[4];        // 融合后的姿态四元数估计 [q0 q1 q2 q3](单位四元数,模长恒为 1)
    float GyroBias[3]; // 陀螺仪零偏估计 [bx by bz],单位 rad/s(z 轴恒为 0,不作估计)

    float Gyro[3]; // 扣零偏后的角速度 [wx wy wz],单位 rad/s
    float Accel[3]; // 低通滤波后的机体加速度 [ax ay az],单位 m/s^2

    float OrientationCosine[3]; // 预测重力方向与各轴的方向余弦 cos(|h_i|),用于按轴缩放零偏修正增益

    float accLPFcoef;        // 加速度低通滤波时间常数,单位 s(即一阶 LPF 的 a 系数)
    float gyro_norm;         // 角速度模长 sqrt(wx^2+wy^2+wz^2),单位 rad/s,用于判稳
    float accl_norm;         // 加速度模长 sqrt(ax^2+ay^2+az^2),单位 m/s^2,静止时约为 9.8
    float AdaptiveGainScale; // 自适应增益缩放系数,范围 [0,1];残差越小越接近 1

    float Roll;  // Roll 横滚角,单位 度(deg),范围 (-180, 180]
    float Pitch; // Pitch 俯仰角,单位 度(deg),范围 (-90, 90]
    float Yaw;   // Yaw 偏航角,单位 度(deg),范围 (-180, 180]

    float YawTotalAngle; // 累计偏航角(不折叠到 ±180),单位 度(deg) = 360*YawRoundCount + Yaw

    float Q1; // 四元数过程噪声(姿态模型误差强度),默认 10
    float Q2; // 陀螺仪零偏过程噪声(零偏随机游走强度),默认 0.001
    float R;  // 加速度计量测噪声(越大越不信任加速度计),默认 1000000

    float dt; // 姿态更新周期,单位 s(通常 0.001,即 1000Hz)
    mat ChiSquare;                // 卡方检验统计量的矩阵封装(1x1)
    float ChiSquare_Data[1];      // 卡方检验检测函数值 rk,越小表示预测与量测越一致
    float ChiSquareTestThreshold; // 卡方检验阈值,默认 1e-8;超过则判量测异常,仅做预测
    float lambda;                 // 渐消因子,范围 (0,1],默认 0.9996;用于放大零偏协方差防止过度收敛

    int16_t YawRoundCount; // yaw 跨越 ±180 边界的翻转计数(每翻转一次 ±1),用于累计角度

    float YawAngleLast; // 上一拍的 yaw 角,单位 度(deg),用于检测角度跨越
} QEKF_INS_t;

/* 全局实例与调试变量 */
extern QEKF_INS_t QEKF_INS;          // 姿态解算器的全局运行状态(定义见 QuaternionEKF.c)
extern float chiSquare;              // 卡方检验结果的外部访问变量(预留,未在本模块定义)
extern float ChiSquareTestThreshold; // 卡方检验阈值的外部访问变量(预留,未在本模块定义)
/*
 * @brief 初始化四元数 EKF 姿态解算器
 * @param process_noise1 四元数过程噪声 Q1,建议 10
 * @param process_noise2 陀螺仪零偏过程噪声 Q2,建议 0.001
 * @param measure_noise  加速度计量测噪声 R,建议 1000000
 * @param lambda         渐消因子,范围 (0,1],建议 0.9996(>1 时会被截断为 1)
 * @param dt             姿态更新周期,单位 s
 * @param lpf            加速度低通滤波时间常数,单位 s(注意: 当前实现未把该值写入 accLPFcoef)
 * @return 无
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt, float lpf);
/*
 * @brief 四元数 EKF 单次更新(陀螺预测 + 加速度计观测融合)
 * @param gx,gy,gz 陀螺仪三轴角速度,单位 rad/s
 * @param ax,ay,az 加速度计三轴比力,单位 m/s^2
 * @return 无
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az);
/*
 * @brief 复位滤波器(重新初始化 P、xhat),用于判定发散或姿态需重新收敛时
 * @return 无
 */
void IMU_QuaternionEKF_Reset(void);

float Get_Pitch(void);// 返回 Pitch 俯仰角,单位 度(deg),范围 (-90, 90]
float Get_Roll(void);// 返回 Roll 横滚角,单位 度(deg),范围 (-180, 180]
float Get_Yaw(void);// 返回 Yaw 偏航角,单位 度(deg),范围 (-180, 180]
#endif
