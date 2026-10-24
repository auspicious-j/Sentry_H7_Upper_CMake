/**
 * @file    dsp/fast_math_functions.h
 * @brief   [LINUX-PORT] CMSIS-DSP 快速数学函数的 Linux 替代声明
 *
 * 实现见 platform/linux/src/dsp_port.c（直接调用 libm）。
 */
#ifndef FAST_MATH_FUNCTIONS_H
#define FAST_MATH_FUNCTIONS_H

#include <stdint.h>
#include "arm_math_types.h"

void arm_atan2_f32(float32_t y, float32_t x, float32_t *result);
float32_t arm_sin_f32(float32_t x);
float32_t arm_cos_f32(float32_t x);
float32_t arm_sqrt_f32(float32_t x);

#endif /* FAST_MATH_FUNCTIONS_H */
