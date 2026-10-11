/*
 * ============================== 文件说明 ==============================
 * 文件：imu_temp_ctrl.c
 * 模块：IMU（惯性测量单元）——BMI088 姿态采样、零偏标定与温度控制
 *
 * 主要职责：
 *   1) 初始化：配置四元数扩展卡尔曼滤波（EKF），并等待 BMI088 上线。
 *   2) 采样与解算：周期读取陀螺仪 / 加速度 / 温度，对陀螺仪做零偏补偿后
 *      送入 EKF，输出 roll / pitch / yaw 姿态角到全局 INS。
 *   3) 启动标定：上电后先等芯片温度接近目标温度（RefTemp）并稳定，再静止
 *      统计若干次陀螺仪输出作为零偏，然后转入正常解算。
 *   4) 温度控制：通过加热 PWM 让 BMI088 保持恒温，减小零偏随温度的漂移
 *      （相关 PID / PWM 代码目前被注释保留，见下文）。
 *
 * 术语速览：
 *   - 零偏(bias)：传感器静止时的非零输出，是姿态发散的主要误差来源之一。
 *   - EKF：扩展卡尔曼滤波，融合陀螺仪/加速度计来估计姿态。
 *   - 位姿：刚体姿态，这里指 roll/pitch/yaw 欧拉角。
 *   - 恒温：把芯片加热到固定温度，使零偏稳定、便于线性补偿。
 * =====================================================================
 */

#include "main.h" // STM32 HAL 总头文件（提供 HAL 类型与全局定义）
#include "cmsis_os.h" // CMSIS-RTOS 封装（FreeRTOS）：提供 osDelay 等接口
#include "BMI088driver.h" // BMI088 六轴 IMU 驱动：BMI088_init / BMI088_read
#include "gpio.h" // GPIO 初始化（板上相关引脚）
//#include "tim.h" // 【已停用】定时器头文件：需要用加热 PWM 时再打开
#include "kalman_filter.h" // 卡尔曼滤波器通用实现
#include "QuaternionEKF.h" // 四元数 EKF：初始化、更新与姿态读取接口
#include "imu_temp_ctrl.h" // 本模块对外接口（INS_t / INS / INS_Init 等）

//#include "bsp_PWM.h" // 【已停用】PWM 封装：做温度控制输出时再打开
#include <math.h> // 数学库：fabsf 等
#include <stdbool.h> // 布尔类型 true / false

#define cheat true // 调试开关：为 true 时启用下方“小角速度归零”处理
#define correct_Time_define 1 // 零偏标定的平均次数：对多次采样求平均得到零偏
#define temp_times 1 // 温度稳定判定所需的连续命中次数阈值
#define RefTemp 40 // 目标温度，单位 ℃：恒温控制希望维持的芯片温度

INS_t INS; // 全局 IMU 状态实例：保存姿态、角速度、加速度、温度

float gyro_correct[3]; // 陀螺仪零偏估计[X,Y,Z]，单位 rad/s，用于从原始角速度中扣除
uint8_t attitude_flag; // 姿态/标定状态机：0=等温度稳定，1=正在标定零偏，2=正常解算
uint32_t correct_times; // 零偏标定已累计的采样次数，用于判断是否达到 correct_Time_define

void INS_Init(void)
{
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0); // 初始化四元数 EKF：写入过程/观测噪声等参数

    while (BMI088_init()) // 反复尝试初始化 BMI088，直到成功（返回 0）
    {
        osDelay(10); // 失败则延时 10 ms 后重试，避免忙等
    }
}

//    PID_Calculate(&TempCtrl, INS.temp, RefTemp); // 【保留】温度 PID：输入当前温度 INS.temp 与目标温度 RefTemp
//    TIM_Set_PWM(&htim3, TIM_CHANNEL_4, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX)); // 【保留】把 PID 输出限幅后写入 TIM3_CH4 加热 PWM
void INS_Task(void)
{
    static uint32_t count = 0; // 调用计数器：用于把不同频率的任务按周期分频执行

    if ((count % 1) == 0) // 每 1 次调用执行一次（当前等价于每次都执行）：姿态采样与解算
    {
        BMI088_read(INS.gyro, INS.accel, &INS.temp); // 读取 BMI088：三轴角速度、三轴加速度、芯片温度

        if (attitude_flag == 2) // 状态 2：零偏已确定，进入正常姿态解算
        {
            gyro_correct[0] = -0.0013251846; // 直接写入固定的 X 轴零偏[rad/s]
            gyro_correct[1] = -0.00224610348; // 固定的 Y 轴零偏[rad/s]
            gyro_correct[2] = 0.000502272975; // 固定的 Z 轴零偏[rad/s]

            INS.gyro[0] -= gyro_correct[0]; // 原始角速度扣除零偏：X 轴
            INS.gyro[1] -= gyro_correct[1]; // 扣除零偏：Y 轴
            INS.gyro[2] -= gyro_correct[2]; // 扣除零偏：Z 轴

#if cheat
            if (fabsf(INS.gyro[2]) < 0.003f) // 死区：Z 轴角速度绝对值小于 0.003 rad/s 时
            {
                INS.gyro[2] = 0; // 视为静止直接置零，抑制积分漂移
            }
#endif
		    IMU_QuaternionEKF_Update(INS.gyro[1],-INS.gyro[0],INS.gyro[2],INS.accel[1],-INS.accel[0],INS.accel[2]); // 用角速度+加速度更新 EKF（X/Y 轴按安装方向做了交换取反）
            INS.pitch = Get_Pitch(); // 取俯仰角 pitch，单位 rad
            INS.roll = Get_Roll(); // 取横滚角 roll，单位 rad
            INS.yaw = Get_Yaw(); // 取偏航角 yaw，单位 rad
        }
        else if (attitude_flag == 1) // 状态 1：静止标定中，累加陀螺仪输出以便求平均零偏
        {
            gyro_correct[0] += INS.gyro[0]; // 累加 X 轴角速度
            gyro_correct[1] += INS.gyro[1]; // 累加 Y 轴角速度
            gyro_correct[2] += INS.gyro[2]; // 累加 Z 轴角速度
            correct_times++; // 采样次数 +1

            if (correct_times >= correct_Time_define) // 达到规定采样次数后
            {
                gyro_correct[0] /= correct_Time_define; // 求平均：X 轴零偏
                gyro_correct[1] /= correct_Time_define; // 求平均：Y 轴零偏
                gyro_correct[2] /= correct_Time_define; // 求平均：Z 轴零偏
                attitude_flag = 2; // 标定完成，切换到正常解算状态
            }
        }
    }

    if ((count % 10) == 0) // 每 10 次调用执行一次（低频）：温度稳定检测
    {
        //        IMU_Temperature_Ctrl(); // 【保留】温度控制（含 PID + 加热 PWM），当前被停用
        static uint32_t temp_Ticks = 0; // 温度稳定命中计数器：连续满足条件才认为稳定
        if ((fabsf(INS.temp - RefTemp) < 30.0f) && attitude_flag == 0) // 温度在目标 ±30 ℃ 以内且尚未开始标定
        {
            temp_Ticks++; // 命中一次，计数 +1
            if (temp_Ticks > temp_times) // 连续命中超过阈值
            {
                attitude_flag = 1; // 认为温度已稳定，进入零偏标定状态
            }
        }
    }

    count++; // 调用计数自增
}

void OS_IMUCallback(void const *argument) // IMU 任务的入口函数（RTOS 任务回调）
{
    INS_Init(); // 先初始化 IMU 与 EKF

    for (;;) // 无限循环：任务主体
    {
        INS_Task(); // 执行一次采样 / 解算 / 控制
        osDelay(1); // 延时 1 ms，形成约 1 kHz 的调用周期
    }
}
