/**
 * @file    arm_math.h
 * @brief   [LINUX-PORT] CMSIS-DSP 的 Linux 替代
 *
 * 原工程链接 Middlewares/ST/ARM/DSP/Lib/libCMSISDSP.a（ARM 架构预编译库），
 * 在 x86-64 Ubuntu 上不能使用。这里用 glibc 的 libm + 纯 C 实现等价功能：
 *
 *   PI / 常量        -> 本文件
 *   arm_atan2_f32    ->  dsp/fast_math_functions.h（实现见 dsp_port.c）
 *   arm_mat_*_f32    ->  dsp/matrix_functions.h（实现见 dsp_port.c）
 *
 * 说明：矩阵运算用朴素实现，性能远不如 CMSIS-DSP 的 SIMD 优化版，
 *       但控制周期内的矩阵规模很小（<= 10x10），足够使用。
 */
#ifndef ARM_MATH_H
#define ARM_MATH_H

#include <stdint.h>
#include <math.h>

#include "arm_math_types.h"
#include "dsp/matrix_functions.h"
#include "dsp/fast_math_functions.h"

/* [LINUX-PORT] 与原 CMSIS-DSP arm_math.h 中的定义保持一致 */
#ifndef PI
#define PI 3.14159265358979f
#endif

#endif /* ARM_MATH_H */
