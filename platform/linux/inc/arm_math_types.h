/**
 * @file    arm_math_types.h
 * @brief   [LINUX-PORT] CMSIS-DSP 基础类型定义
 */
#ifndef ARM_MATH_TYPES_H
#define ARM_MATH_TYPES_H

#include <stdint.h>

#ifndef ARM_MATH_H  /* 允许被 arm_math.h 先包含 */
#endif

typedef float float32_t;
typedef double float64_t;

/* 与原 CMSIS-DSP 的状态码保持一致 */
typedef enum
{
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR = -2,
    ARM_MATH_SIZE_MISMATCH = -3,
    ARM_MATH_NANINF = -4,
    ARM_MATH_SINGULAR = -5,
    ARM_MATH_TEST_FAILURE = -6
} arm_status;

#endif /* ARM_MATH_TYPES_H */
