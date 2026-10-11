/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 ******************************************************************************
 * @attention
 * 该卡尔曼滤波器可以在传感器采样频率不同的情况下，动态调整矩阵H R和K的维数与数值。
 * This implementation of kalman filter can dynamically adjust dimension and
 * value of matrix H R and K according to the measurement validity under any
 * circumstance that the sampling rate of component sensors are different.
 *
 * 因此矩阵H和R的初始化会与矩阵P A和Q有所不同。另外的，在初始化量测向量z时需要额外写
 * 入传感器量测所对应的状态与这个量测的方式，详情请见例程
 * Therefore, the initialization of matrix P, F, and Q is sometimes different
 * from that of matrices H R. when initialization. Additionally, the corresponding
 * state and the method of the measurement should be provided when initializing
 * measurement vector z. For more details, please see the example.
 *
 * 若不需要动态调整量测向量z，可简单将结构体中的Use_Auto_Adjustment初始化为0，并像初
 * 始化矩阵P那样用常规方式初始化z H R即可。
 * If automatic adjustment is not required, assign zero to the UseAutoAdjustment
 * and initialize z H R in the normal way as matrix P.
 *
 * 要求量测向量z与控制向量u在传感器回调函数中更新。整数0意味着量测无效，即自上次卡尔曼
 * 滤波更新后无传感器数据更新。因此量测向量z与控制向量u会在卡尔曼滤波更新过程中被清零
 * MeasuredVector and ControlVector are required to be updated in the sensor
 * callback function. Integer 0 in measurement vector z indicates the invalidity
 * of current measurement, so MeasuredVector and ControlVector will be reset
 * (to 0) during each update.
 *
 * 此外，矩阵P过度收敛后滤波器将难以再适应状态的缓慢变化，从而产生滤波估计偏差。该算法
 * 通过限制矩阵P最小值的方法，可有效抑制滤波器的过度收敛，详情请见例程。
 * Additionally, the excessive convergence of matrix P will make filter incapable
 * of adopting the slowly changing state. This implementation can effectively
 * suppress filter excessive convergence through boundary limiting for matrix P.
 * For more details, please see the example.
 *
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 设置最小方差
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 开启自动调整
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 设置矩阵值
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 循环更新
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 测量数据更新应按照以下形式 即向MeasuredVector赋值
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */

#include "kalman_filter.h"
#include "FreeRTOS.h"
#include "task.h"

uint16_t sizeof_float, sizeof_double;   // 全局缓存 sizeof(float)/sizeof(double),供各函数做内存尺寸计算时复用(避免重复 sizeof)

static void H_K_R_Adjustment(KalmanFilter_t *kf);   // 静态内部函数:按本周期有效量测动态重构 H/R/K 的维数与数值

/**
 * @brief 内存分配封装:统一从 FreeRTOS 堆申请内存,便于统一内存来源/统计。
 * @param size 需要分配的字节数
 * @return void* 分配到的内存首地址;若堆不足则返回 0(NULL)
 * @note 调用方(Kalman_Filter_Init)未做判空,内存不足时会带着空指针继续运行。
 */
void* user_malloc(size_t size)
{
    void* tmp = 0;                 // 结果指针,初值 0(NULL)表示尚未分配
    tmp = pvPortMalloc(size);      // 从 FreeRTOS 堆分配 size 字节
    return tmp;                    // 返回分配结果
}


/**
 * @brief 初始化卡尔曼滤波器:记录各向量维度,为所有矩阵/缓冲区分配内存,把矩阵实例绑定到缓冲区,最后清零。
 *        调用后需由用户自行给 P/F/Q(以及 H/R 的配置数组)等填入初值,滤波器才能正常工作。
 *
 * @param kf       KalmanFilter_t 结构体指针(待初始化的滤波器对象)
 * @param xhatSize 状态变量维度 n
 * @param uSize    控制变量维度 m(可为 0,表示无控制输入)
 * @param zSize    观测量(量测)维度,即最多可接入的传感器数量
 * @note  每个矩阵都用 Matrix_Init 绑定"行列数 + 数据缓冲区";内存来自 user_malloc,全程不释放。
 * @note  各矩阵数学含义见头文件 KalmanFilter_t 结构体成员注释。
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{

    sizeof_float = sizeof(float);      // 缓存 float 字节数(通常 4)
    sizeof_double = sizeof(double);    // 缓存 double 字节数(通常 8)

    kf->xhatSize = xhatSize;      // 记录状态维度 n
    kf->uSize = uSize;            // 记录控制维度 m
    kf->zSize = zSize;            // 记录量测维度

    kf->MeasurementValidNum = 0;  // 初始时本周期无有效量测

    // measurement flags 量测相关配置数组:分配内存并清零,后续由用户按例程填写
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);   // 量测→状态映射表,长度 zSize
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);     // 各量测在 H 中的系数,长度 zSize
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize); // 各量测观测噪声(方差),长度 zSize
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);   // 各状态方差下限,长度 xhatSize
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);             // 有效量测下标暂存,长度 zSize
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data 对外输入/输出缓冲区
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);      // 滤波输出,长度 xhatSize
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);        // 量测输入,长度 zSize
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);         // 控制输入,长度 uSize
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat x(k|k) 后验状态估计(n×1),数据存于 xhat_data
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus x(k|k-1) 先验状态估计(n×1),数据存于 xhatminus_data
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0)
    {
        // control vector u 控制向量(m×1),数据存于 u_data
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z 量测向量(zSize×1),数据存于 z_data
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k) 后验状态协方差(n×n),数据存于 P_data
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    // create covariance matrix P(k|k-1) 先验状态协方差(n×n),数据存于 Pminus_data
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F FT 状态转移矩阵及其转置(n×n)
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0)
    {
        // control matrix B 控制矩阵(n×m),仅当 uSize 不为 0 时分配
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    // measurement matrix H 量测矩阵及其转置(维数随有效量测变化)
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q 过程噪声协方差(n×n),刻画模型不确定度
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R 观测噪声协方差(维数随有效量测变化)
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K 卡尔曼增益(维数随有效量测变化)
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    // 中间计算缓存:S(新息协方差)与若干临时矩阵/向量,仅在算法内部使用
    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->S_data, 0, sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->temp_matrix_data, 0, sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->temp_matrix_data1, 0, sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    memset(kf->temp_vector_data, 0, sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    memset(kf->temp_vector_data, 0, sizeof_float * kf->xhatSize);

    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    // 五个环节默认都不跳过,全部走内置实现(为 0 即执行;置非 0 可由用户函数替换该环节)
    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
 * @brief 复位滤波器:把已分配的各矩阵/缓冲区全部清零(不重新分配内存),通常用于重新启动滤波。
 *
 * @param kf       KalmanFilter_t 结构体指针
 * @param xhatSize 状态变量维度 n(须与初始化时一致)
 * @param uSize    控制变量维度 m
 * @param zSize    观测量维度
 * @note 只清数据、不释放内存;若需重新填初值,由用户自行重做。
 */
void Kalman_Filter_Reset(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    // 量测配置数组清零
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // 对外输入/输出缓冲区清零
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // 状态估计与协方差等各矩阵缓冲区清零
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);

    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);

    if(uSize != 0)
    {memset(kf->u_data, 0, sizeof_float * uSize);}

    memset(kf->z_data, 0, sizeof_float * zSize);

    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);

    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);

    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);

    if(uSize != 0)
    {memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);}

    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);

    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);

    memset(kf->R_data, 0, sizeof_float * zSize * zSize);

    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}


/**
 * @brief 环节0:获取量测。把用户写入的 MeasuredVector/ControlVector 搬到内部 z/u,
 *        并按需(UseAutoAdjustment)调用 H_K_R_Adjustment 动态调整 H/R/K,随后清零 MeasuredVector。
 *
 * @param kf KalmanFilter_t 结构体指针
 * @note 搬运量测同时起到"消费一次量测"的作用:搬完即清零,等待下个周期传感器重新写入。
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 矩阵H K R根据量测情况自动调整
    // matrix H K R auto adjustment:启用时按本周期有效量测重构 H/R/K 并降维
    if (kf->UseAutoAdjustment != 0)
    {
        H_K_R_Adjustment(kf);                                                  // 内部函数:识别有效量测并重构 H/R/K
    }
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);      // 关闭自动调整:整段拷贝量测到 z
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);               // 清零,等待下次传感器写入
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);           // 拷贝控制输入到 u
}

/**
 * @brief 环节1:先验状态估计(预测)。xhat'(k) = F * xhat(k-1) + B * u
 *        用状态转移矩阵 F 把上一时刻的后验状态外推到当前时刻;若有控制输入,再叠加 B*u 的作用。
 *
 * @param kf KalmanFilter_t 结构体指针
 * @note 结果写入 kf->xhatminus;SkipEq1 非 0 时跳过(改由用户回调实现)。
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;    // 临时向量 temp_vector 设为 n×1
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);         // temp_vector = F * xhat(k-1)
            kf->temp_vector1.numRows = kf->xhatSize;   // 临时向量 temp_vector1 设为 n×1
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);           // temp_vector1 = B * u
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus); // xhat'(k) = F*xhat + B*u
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);           // 无控制输入:xhat'(k) = F * xhat
        }
    }
}

/**
 * @brief 环节2:先验协方差预测。P'(k) = F * P(k-1) * F转置 + Q
 *        把上一时刻的估计不确定度沿状态转移传播,并叠加过程噪声 Q(模型自身的误差)。
 *
 * @param kf KalmanFilter_t 结构体指针
 * @note 结果写入 kf->Pminus;SkipEq2 非 0 时跳过。
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);                              // FT = F 的转置(供左右两乘使用)
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);                   // Pminus = F * P(k-1)
        kf->temp_matrix.numRows = kf->Pminus.numRows;                                   // 临时矩阵尺寸对齐为 n×n
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);              // P'(k) = F*P*F转置 + Q
    }
}
/**
 * @brief 环节3:计算卡尔曼增益 K。
 *        S = H * P'(k) * H转置 + R(新息协方差,包含观测噪声);
 *        K = P'(k) * H转置 * inv(S)。K 决定"预测值"与"量测值"的融合权重:K 越大越信任量测。
 *
 * @param kf KalmanFilter_t 结构体指针
 * @note 结果写入 kf->K;SkipEq3 非 0 时跳过;矩阵求逆由 CMSIS-DSP 完成,要求 S 可逆。
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}
/**
 * @brief 环节4:后验状态融合(量测更新)。
 *        xhat(k) = xhat'(k) + K * (z(k) - H * xhat'(k));
 *        其中 (z(k) - H*xhat'(k)) 称为"新息"(量测与预测之间的偏差),K 乘新息后修正先验状态。
 *
 * @param kf KalmanFilter_t 结构体指针
 * @note 结果写入 kf->xhat;SkipEq4 非 0 时跳过。
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}
/**
 * @brief 环节5:后验协方差修正。P(k) = P'(k) - K * H * P'(k)
 *        融合量测之后不确定度下降;该式等价于常见的 P(k) = (I - K*H) * P'(k)。
 *
 * @param kf KalmanFilter_t 结构体指针
 * @note 结果写入 kf->P;SkipEq5 非 0 时跳过。
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 *
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 获取量测信息
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 先验估计
    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 预测更新
    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 量测更新
        // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 融合
        // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 修正方差
        // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 无有效量测,仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);              // 后验状态 = 先验状态
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);     // 后验协方差 = 先验协方差
    }

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 避免滤波器过度收敛
    // suppress filter excessive convergence
    // 逐个检查 P 的对角元(各状态分量的方差),低于下限时抬升到下限,使滤波器保留对状态缓慢变化的适应能力
    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])     // P 为对称方阵,对角元下标 = i*xhatSize + i
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];     // 方差下限钳位
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);  // 把状态估计拷贝到对外输出 FilteredValue

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);                                               // 输出前的最后钩子

    return kf->FilteredValue;                                               // 返回滤波输出指针
}

/**
 * @brief 根据本周期"哪些量测有效"(z 中非 0 视为有效),动态重构量测矩阵 H、观测噪声 R 与增益 K 的维数。
 *        做法:统计有效量测个数 MeasurementValidNum,把有效量测紧凑重排到 z 前部,并按
 *        MeasurementMap/Degree/MatR_DiagonalElements 重建 H 和 R(仅保留有效量测对应的行/对角元),
 *        再把 H/HT/R/K/z 的行列数改为有效量测个数,从而在部分传感器无数据时自动降维。
 *
 * @param kf KalmanFilter_t 结构体指针
 * @note 依赖用户预先填好 MeasurementMap/MeasurementDegree/MatR_DiagonalElements;
 *       以 z 元素是否为 0 判定量测有效,故真实量测值恰好为 0 时会被误判为无效。
 */
static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;                                              // 有效量测计数清零,下面重新统计

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);         // 把本周期量测搬到 z
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);                  // 清零量测输入,等待下次写入

    // 识别量测数据有效性并调整矩阵H R K
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0)
        {
            // 重构向量z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // 重构矩阵H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];   // H 的第 MeasurementValidNum 行、第 (map-1) 列写入该量测系数
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        // 重构矩阵R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];   // R 的第 i 个对角元 = 第 i 个有效量测的观测方差
    }

    // 调整矩阵维数
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}

