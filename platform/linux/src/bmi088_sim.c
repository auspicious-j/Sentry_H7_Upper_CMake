/**
 ******************************************************************************
 * @file    bmi088_sim.c
 * @brief   [LINUX-PORT] BMI088 惯性传感器模拟数据源
 *
 * 原工程的 IMU/BMI088/BMI088Middleware.c 通过 SPI2 + 片选 GPIO 读取真实芯片，
 * Ubuntu 上没有这条总线。这里用“软件模拟数据源”替代：
 *
 *   BMI088_init()  -> 直接返回成功（不做真实自检）
 *   BMI088_read()  -> 按正弦规律生成角速度/加速度，便于验证姿态解算链路
 *
 * 输出约定与原驱动一致：
 *   gyro[3]  : rad/s
 *   accel[3] : m/s^2（静止水平时 accel[2] ≈ +9.8）
 *   temp     : ℃
 *
 * 若要接入真实 IMU，可换成串口/I2C 设备或 ROS2 imu 话题，
 * 只要保持 BMI088_read() 的接口不变即可。
 ******************************************************************************
 */
#include <math.h>

#include "BMI088driver.h"
#include "port_linux.h"

#define SIM_GRAVITY 9.80665f
#define SIM_TWO_PI (2.0f * 3.14159265358979f)

static bool s_inited = false;
static uint64_t s_t0_us = 0;

static float sim_seconds(void)
{
    if (!s_inited)
    {
        s_t0_us = port_monotonic_us();
        s_inited = true;
    }
    return (float)((double)(port_monotonic_us() - s_t0_us) * 1e-6);
}

uint8_t BMI088_init(void)
{
    (void)sim_seconds();
    port_log("[imu-sim] BMI088 模拟数据源初始化\n");
    return BMI088_NO_ERROR;
}

uint8_t bmi088_accel_init(void)
{
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    return BMI088_NO_ERROR;
}

void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    const float t = sim_seconds();

    if (gyro != NULL)
    {
        /* 缓慢摆动：yaw 0.15Hz、pitch 0.2Hz、roll 0.25Hz */
        gyro[0] = 0.050f * sinf(SIM_TWO_PI * 0.25f * t);
        gyro[1] = 0.050f * cosf(SIM_TWO_PI * 0.20f * t);
        gyro[2] = 0.150f * sinf(SIM_TWO_PI * 0.15f * t);
    }

    if (accel != NULL)
    {
        /* 近似重力矢量 + 微小倾斜变化 */
        accel[0] = SIM_GRAVITY * 0.02f * sinf(SIM_TWO_PI * 0.10f * t);
        accel[1] = SIM_GRAVITY * 0.02f * cosf(SIM_TWO_PI * 0.13f * t);
        accel[2] = SIM_GRAVITY;
    }

    if (temperate != NULL)
    {
        /* 温度缓慢上升并在 40℃ 附近稳定（温度控制环节需要它） */
        *temperate = 40.0f + 0.5f * sinf(SIM_TWO_PI * 0.01f * t);
    }
}
