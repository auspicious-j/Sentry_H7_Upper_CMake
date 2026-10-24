/**
 * @file    dsp/matrix_functions.h
 * @brief   [LINUX-PORT] CMSIS-DSP 矩阵运算的 Linux 替代声明
 *
 * 工程中的 kalman_filter.c 通过宏 Matrix_Add/Matrix_Multiply/... 使用这些函数。
 * 实现见 platform/linux/src/dsp_port.c。
 */
#ifndef MATRIX_FUNCTIONS_H
#define MATRIX_FUNCTIONS_H

#include <stdint.h>
#include "arm_math_types.h"

typedef struct
{
    uint16_t numRows;   /**< 行数 */
    uint16_t numCols;   /**< 列数 */
    float32_t *pData;   /**< 行优先存储的数据指针 */
} arm_matrix_instance_f32;

void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nColumns, float32_t *pData);

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst);

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst);

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst);

#endif /* MATRIX_FUNCTIONS_H */
