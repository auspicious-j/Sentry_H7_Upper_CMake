/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 *          基于四元数的扩展卡尔曼滤波(EKF)姿态解算: 融合陀螺仪(高频、短时准)
 *          与加速度计(低频、长期准),并在线估计陀螺仪 x/y 轴零偏;含卡方检验
 *          与自适应增益,以抑制机动/振动时的错误量测。
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 *
 * 模块职责与实现要点:
 *   本文件是姿态解算的核心实现,对外仅暴露 IMU_QuaternionEKF_Init / _Update /
 *   _Reset 与三个角度 Getter。内部把通用卡尔曼滤波框架(kalman_filter.c)的若干
 *   标准步骤替换为姿态解算专用回调:
 *     - User_Func0 = Observe                  : 把 P/K/H 拷贝出来供调试观察
 *     - User_Func1 = F_Linearization_P_Fading : 在工作点线性化 F,并对零偏协方差做渐消
 *     - User_Func2 = SetH                     : 在工作点计算观测雅可比矩阵 H
 *     - User_Func3 = xhatUpdate               : 自适应后验估计(含卡方检验与发散保护)
 *
 *   状态量(6 维): x = [q0 q1 q2 q3  bx by]^T(姿态四元数 + 陀螺 x/y 零偏)
 *   观测量(3 维): z = 归一化重力方向(由加速度计得到)
 *   预测: 用陀螺积分推进四元数;更新: 用加速度计观测重力方向来矫正姿态与零偏。
 ******************************************************************************
 */
#include "QuaternionEKF.h"
#include "arm_math.h"

#include <stdbool.h>

/* 姿态解算器全局唯一实例,初始全零(未初始化状态) */
QEKF_INS_t QEKF_INS={0};

/* 状态转移矩阵 F(6x6,行主序): 第 k 行第 j 列的元素索引为 k*6+j。
 * 此处给出单位阵初值;左上 4x4 四元数块在预测步按公式线性化,
 * 右下 2x2 为单位阵(零偏自身不随时间传播)。 */
const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

/* 初始协方差矩阵 P 的常量模板(6x6,行主序),Reset 时用于重置 P。
 * 对角元: 四个四元数状态初始不确定度很大(100000),两个零偏状态为 100;
 * 非对角元为 0.1(较小的先验相关项)。 */
const float IMU_QuaternionEKF_P_Const[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};

/* P 的工作副本(6x6,行主序),滤波过程中被不断更新;
 * 由 P_Const 拷贝初值,并在 Reset 时被重新拷贝。 */
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
float IMU_QuaternionEKF_K[18]; // 卡尔曼增益 K 的调试副本(6x3 = 18 个元素,行主序)
float IMU_QuaternionEKF_H[18]; // 观测雅可比矩阵 H 的调试副本(3x6 = 18 个元素,行主序)

/* ---- 内部静态函数前置声明 ---- */
static float invSqrt(float x);                                              // 快速计算 1/sqrt(x)
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);                  // 观察回调: 拷出 P/K/H
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf); // 线性化 F + 零偏协方差渐消
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);                     // 计算观测雅可比 H
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);               // 自适应后验估计 + 卡方检验

/*
 * @brief 初始化四元数 EKF 姿态解算器(设置噪声参数、渐消因子、更新周期并挂载回调)
 * @param[in] process_noise1 四元数过程噪声 Q1,建议 10
 * @param[in] process_noise2 陀螺仪零偏过程噪声 Q2,建议 0.001
 * @param[in] measure_noise  加速度计量测噪声 R,建议 1000000
 * @param[in] lambda         渐消因子,范围 (0,1],建议 0.9996(>1 时会被截断为 1)
 * @param[in] dt             姿态更新周期,单位 s
 * @param[in] lpf            加速度低通滤波时间常数,单位 s(注意: 当前实现未把该值写入 accLPFcoef)
 * @return 无
 * @note 内部还完成: 姿态初值置为单位四元数、关键标志位复位、F/P 矩阵初值装载。
 */
/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda		    fading coefficient          0.9996
 * @param[in] dt				    update period in s
 */
/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda			fading coefficient          0.9996
 * @param[in] dt					update period in s
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt, float lpf)
{
    QEKF_INS.Initialized = 1;               // 标记已初始化
    QEKF_INS.Q1 = process_noise1;           // 四元数过程噪声(姿态模型误差强度)
    QEKF_INS.Q2 = process_noise2;           // 陀螺仪零偏过程噪声(零偏随机游走强度)
    QEKF_INS.R = measure_noise;             // 加速度计量测噪声
    QEKF_INS.ChiSquareTestThreshold = 1e-8; // 卡方检验阈值(硬编码为 1e-8)
    QEKF_INS.ConvergeFlag = 0;              // 收敛标志清零(初始未收敛)
    QEKF_INS.ErrorCount = 0;                // 连续异常计数清零
    QEKF_INS.UpdateCount = 0;               // 更新计数清零(下一拍将初始化加速度低通滤波)
    // 记录姿态更新周期(单位 s),后续 F 线性化与加速度 LPF 都会用到
	    QEKF_INS.dt = dt;

    // 渐消因子必须 <= 1,否则会放大协方差导致发散,这里做上限保护
    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda; // 写入渐消因子(供 F_Linearization 中放大零偏协方差)

    // 初始化矩阵维度信息: 状态 6 维、控制 0 维、量测 3 维
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    // 把卡方检验量封装成 1x1 矩阵,其数据区指向 ChiSquare_Data
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    // 姿态初始化: 设为单位四元数(无旋转),即先验 xhat = [1 0 0 0 | 0 0]
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能:
    //   User_Func0=观测拷贝, User_Func1=F线性化+渐消, User_Func2=置H, User_Func3=后验更新
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;                  // 观察: 拷出 P/K/H
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading; // F 线性化 + 渐消
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;                     // 计算观测雅可比 H
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;               // 自适应后验更新

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = true; // 跳过标准第3步(SetK),K 由 xhatUpdate 内自定义计算
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = true; // 跳过标准第4步(xhat 更新),由 xhatUpdate 内部完成

    // 把 F、P 的初值装载进滤波器(F=单位阵,P=P 模板)
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/*
 * @brief 复位四元数 EKF: 重新初始化矩阵维度与协方差,姿态归零(单位四元数)
 * @return 无
 * @note 用于判定发散或姿态需要重新收敛时调用;P 重置为 P_Const 模板。
 */
void IMU_QuaternionEKF_Reset(void)
{
    // 初始化矩阵维度信息: 状态 6 维、控制 0 维、量测 3 维(与 Init 一致)
    Kalman_Filter_Reset(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);

	    memcpy(IMU_QuaternionEKF_P, IMU_QuaternionEKF_P_Const, sizeof(IMU_QuaternionEKF_P));

	// 姿态初始化
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1; // 单位四元数实部 q0
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0; // 虚部 q1
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0; // 虚部 q2
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0; // 虚部 q3

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = true; // 跳过标准第3步(SetK)
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = true; // 跳过标准第4步(xhat 更新)

    // 重新装载 F、P 初值
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}
/**
 * @brief Quaternion EKF update
 * @note 单次姿态更新(每控制周期调用一次): 先按陀螺做预测(线性化状态转移 F),
 *       再用加速度计做观测更新(自适应增益 + 卡方检验 + 发散保护),最后换算欧拉角。
 * @param[in]     quaternion need to be updated
* @param[in]     gyro x y z in rad/s
 * @param[in]     accel x y z in m/s²
 * @param[in]     update period in s
 */
/**
 * @brief Quaternion EKF update
 * @param[in] 	quaternion need to be updated
* @param[in] 	gyro x y z in rad/s
 * @param[in] 	accel x y z in m/s²
 * @param[in] 	update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    volatile float halfgxdt, halfgydt, halfgzdt; // 0.5*各轴角速度*dt,四元数离散化系数
    volatile float accelInvNorm;                 // 加速度模长的倒数,用于归一化重力方向

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
         上表是 6x6 的 F(索引 = 行*6+列),带 * 的元素在运行时被赋值;
         左上 4x4 是四元数微分方程 q_dot = 0.5*Omega*q 的一阶离散化,
         右上 4x2 为零矩阵(零偏不参与预测步,其影响在协方差更新阶段才被填入)。
    */

    // 用当前零偏估计补偿原始角速度: Gyro = 测量值 - 零偏
    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F,先算好 0.5*w*dt 三个中间量
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * QEKF_INS.dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * QEKF_INS.dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * QEKF_INS.dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;  // F(0,1)
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;  // F(0,2)
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;  // F(0,3)

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;   // F(1,0)
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;   // F(1,2)
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;  // F(1,3)

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;  // F(2,0)
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt; // F(2,1)
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;  // F(2,3)

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;  // F(3,0)
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;  // F(3,1)
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt; // F(3,2)

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (QEKF_INS.UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        QEKF_INS.Accel[0] = ax; // 首拍直接用原始加速度初始化,避免滤波从 0 缓慢爬升
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
        QEKF_INS.UpdateCount++;
    }
    // 一阶低通: y = y*(a/(dt+a)) + x*(dt/(dt+a)),其中 a = accLPFcoef(时间常数,单位 s)
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    // set z,单位化重力加速度向量(静止时加速度计测得的方向即为重力方向)
    // 计算加速度模长(静止时应约为 9.8 m/s^2,用于后续判稳)
    QEKF_INS.accl_norm = sqrtf(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    accelInvNorm = 1.0f / QEKF_INS.accl_norm; // 模长倒数,避免逐分量做除法


    QEKF_INS.IMU_QuaternionEKF.MeasuredVector[0] = QEKF_INS.Accel[0] * accelInvNorm; // 用归一化加速度作为量测向量 z 的分量
    QEKF_INS.IMU_QuaternionEKF.MeasuredVector[1] = QEKF_INS.Accel[1] * accelInvNorm;
    QEKF_INS.IMU_QuaternionEKF.MeasuredVector[2] = QEKF_INS.Accel[2] * accelInvNorm;

    // get body state,计算角速度模长(用于判断是否静止/低速)
    QEKF_INS.gyro_norm = sqrtf(    QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                    QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                    QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);


    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f && QEKF_INS.accl_norm < 9.8f + 0.5f) // w<0.3rad/s 且 |a| 在 [9.3,10.3]
    {
        QEKF_INS.StableFlag = 1; // 静止: 加速度可信
    }
    else
    {
        QEKF_INS.StableFlag = 0; // 机动/异常: 加速度不可信
    }

    // set Q R,过程噪声和观测噪声矩阵(仅对角元非零)
    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;  // Q(0,0) 四元数 q0
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;  // Q(1,1) 四元数 q1
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt; // Q(2,2) 四元数 q2
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt; // Q(3,3) 四元数 q3
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt; // Q(4,4) 陀螺 x 零偏
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt; // Q(5,5) 陀螺 y 零偏
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R; // R(0,0)
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R; // R(1,1)
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R; // R(2,2)

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 获取融合后的数据,包括四元数(前 4 项)和 xy 零飘(第 5、6 项)
	    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1]; // 四元数虚部 q1
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2]; // 四元数虚部 q2
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3]; // 四元数虚部 q3
    // 由四元数换算欧拉角(单位 度): Roll/Yaw 用 atan2,Pitch 用 asin

	    arm_atan2_f32(QEKF_INS.q[0]*QEKF_INS.q[1] + QEKF_INS.q[2]*QEKF_INS.q[3], 0.5f - QEKF_INS.q[1]*QEKF_INS.q[1] - QEKF_INS.q[2]*QEKF_INS.q[2],&QEKF_INS.Roll);
	    QEKF_INS.Roll  *=57.29578f;
	    QEKF_INS.Pitch =57.29578f * asinf(-2.0f * (QEKF_INS.q[1]*QEKF_INS.q[3] - QEKF_INS.q[0]*QEKF_INS.q[2]));
	    arm_atan2_f32(QEKF_INS.q[1]*QEKF_INS.q[2] + QEKF_INS.q[0]*QEKF_INS.q[3], 0.5f - QEKF_INS.q[2]*QEKF_INS.q[2] - QEKF_INS.q[3]*QEKF_INS.q[3],&QEKF_INS.Yaw);
    QEKF_INS.Yaw   *=57.29578f;
    // 取出陀螺 x/y 零偏估计(z 轴不做估计,恒置 0)
	    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移
		// get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) // 从正侧越过 ±180
    {
        QEKF_INS.YawRoundCount--; // 圈数减一
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) // 从负侧越过 ±180
    {
        QEKF_INS.YawRoundCount++; // 圈数加一
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw; // 展开为连续角度
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw; // 记录本拍 yaw,供下一拍比较
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 *        具体地: 先把预测先验四元数归一化,再填入 F 的 4x2 分块(表示陀螺零偏 bx/by
 *        对四元数预测的影响),随后对零偏对应的协方差项做渐消(lambda 放大)与限幅。
 *
 * @param kf 卡尔曼滤波器对象(xhatminus_data 为预测先验状态,F_data/P_data 为对应矩阵)
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    volatile float q0, q1, q2, q3; // 预测先验四元数副本
    volatile float qInvNorm;       // 四元数模长倒数,用于归一化

    q0 = kf->xhatminus_data[0]; // 先验四元数 q0
    q1 = kf->xhatminus_data[1]; // 先验四元数 q1
    q2 = kf->xhatminus_data[2]; // 先验四元数 q2
    q3 = kf->xhatminus_data[3]; // 先验四元数 q3

    // quaternion normalize,归一化先验四元数(预测步的积分会引入模长漂移)
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
         上表是 6x6 的 F(索引 = 行*6+列),这里只需填右上 4x2 分块
         (列 4、5 对应零偏 bx、by),它表示零偏对四元数预测的偏导。
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;  // F(0,4)
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;  // F(0,5)

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2; // F(1,4)
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;  // F(1,5)

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2; // F(2,4)
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2; // F(2,5)

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;  // F(3,4)
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2; // F(3,5)

    // fading filter,防止零飘参数过度收敛: 除以 lambda(<1)相当于放大零偏对应的协方差
    kf->P_data[28] /= QEKF_INS.lambda; // P(4,4) 陀螺 x 零偏
    kf->P_data[35] /= QEKF_INS.lambda; // P(5,5) 陀螺 y 零偏

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *        h(x) 为"由四元数预测的重力方向",H 即其对四元数 q0..q3 的偏导(3x4);
 *        后两列(零偏 bx/by)恒为 0,因为重力方向不依赖陀螺零偏。
 *
 * @param kf 卡尔曼滤波器对象(xhatminus_data 为预测先验状态,H_data 为输出)
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    volatile float doubleq0, doubleq1, doubleq2, doubleq3; // 2*q0..2*q3,化简偏导计算
    /* H(3x6,行主序)
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
         每行对应一个量测分量(重力方向的 x/y/z),每列对应一个状态;
         后两列(零偏)恒为 0。
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0]; // 2*q0
    doubleq1 = 2 * kf->xhatminus_data[1]; // 2*q1
    doubleq2 = 2 * kf->xhatminus_data[2]; // 2*q2
    doubleq3 = 2 * kf->xhatminus_data[3]; // 2*q3

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize); // 先整体清零,只需再填左侧 3x4

    kf->H_data[0] = -doubleq2; // H(0,0)
    kf->H_data[1] = doubleq3;  // H(0,1)
    kf->H_data[2] = -doubleq0; // H(0,2)
    kf->H_data[3] = doubleq1;  // H(0,3)

    kf->H_data[6] = doubleq1;  // H(1,0)
    kf->H_data[7] = doubleq0;  // H(1,1)
    kf->H_data[8] = doubleq3;  // H(1,2)
    kf->H_data[9] = doubleq2;  // H(1,3)

    kf->H_data[12] = doubleq0;  // H(2,0)
    kf->H_data[13] = -doubleq1; // H(2,1)
    kf->H_data[14] = -doubleq2; // H(2,2)
    kf->H_data[15] = doubleq3;  // H(2,3)
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 *        步骤概览: 计算新息协方差 S=H·P'·HT+R 及其逆 → 计算残差 z-h(x') →
 *        卡方检验判定量测可信度 → 计算卡尔曼增益 K 并做自适应缩放 →
 *        得到状态修正量 K·(z-h(x')) → 修正零偏(限幅)并输出后验 xhat。
 *
 * @param kf 卡尔曼滤波器对象(输入先验 xhatminus/Pminus/z/H,输出后验 xhat/K)
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    volatile float q0, q1, q2, q3;

    // ---- 1) 计算新息协方差 S = H·P'(k)·HT + R 及其逆 ----
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // H 转置: z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H·P'(k)·HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(S)

    q0 = kf->xhatminus_data[0]; // 先验四元数 q0
    q1 = kf->xhatminus_data[1]; // q1
    q2 = kf->xhatminus_data[2]; // q2
    q3 = kf->xhatminus_data[3]; // q3

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的),即观测函数 h(xhat'(k))
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测重力方向与各轴的方向余弦,用于按轴缩放零偏修正增益
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.OrientationCosine[i] = arm_cos_f32(fabsf(kf->temp_vector_data[i]));
    }

    // 利用加速度计数据修正: 计算残差(新息) r = z(k) - h(xhat'(k))
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验: rk = r'·inv(S)·r(马氏距离平方),衡量残差是否异常
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(S)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = (z(k) - h(xhat'(k)))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare); // chiSquare = rk
    // rk is small,filter converged/converging: rk 小于半个阈值,认为滤波器已收敛
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged: rk 超过阈值且此前已收敛,可能发散
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            // 滤波器发散
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = false; // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = true; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
        {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = false;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

float Get_Pitch()
{
    return QEKF_INS.Pitch;
}

float Get_Roll()
{
    return QEKF_INS.Roll;
}

float Get_Yaw()
{
    return QEKF_INS.Yaw;
}
/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    volatile float tmp = 1.0f;
    tmp /= sqrtf(x);
    return tmp;
}
