/**
************************************************************************
* @file    BMI088Middleware.h
* @brief   BMI088 硬件接口层头文件(声明片选/SPI收发/延时接口)
* @details 本层只负责把字节发出去、收回来，以及片选与延时；
*          寄存器含义与数据解析由上层 BMI088driver 完成。
*          SPI：同步串行总线，主机发 1 字节的同时会收 1 字节。
*          片选(CS)：拉低选中、拉高释放；BMI088 的加速度计与陀螺仪
*          是同一封装内的两个独立从机，各有独立片选脚。
************************************************************************
**/
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

// 标准整型定义头文件(stdint.h)
#include "stdint.h"

// 通信方式选择(编译期二选一)：
// BMI088_USE_SPI = 走 SPI(本工程用 SPI2)；BMI088_USE_IIC = 走 I2C。
#define BMI088_USE_SPI
//#define BMI088_USE_IIC

// ===== 基础接口：初始化与延时 =====
extern void BMI088_GPIO_init(void);
// 通信接口初始化(SPI 已由 CubeMX 初始化，本工程为空实现)
extern void BMI088_com_init(void);
// 毫秒级延时
extern void BMI088_delay_ms(uint16_t ms);
// 微秒级延时(基于 SysTick 忙等)
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
// ===== SPI 模式：片选与收发接口 =====
// 加速度计片选：L=拉低选中，H=拉高释放
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

// 陀螺仪片选：L=拉低选中，H=拉高释放
extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

// SPI 单字节全双工收发：发 1 字节同时收 1 字节(形参名 reg 实为待发字节)
extern uint8_t BMI088_read_write_byte(uint8_t reg);

// 若改用 I2C，则在此声明对应实现(本工程未启用)
#elif defined(BMI088_USE_IIC)

#endif

#endif
