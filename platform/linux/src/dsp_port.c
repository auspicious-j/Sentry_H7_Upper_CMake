/**
 ******************************************************************************
 * @file    dsp_port.c
 * @brief   [LINUX-PORT] CMSIS-DSP 在 Ubuntu 上的替代实现
 *
 * ARM 的 libCMSISDSP.a 是 Cortex-M 目标码，x86-64 上不可用。
 * 这里用标准 C + libm 实现工程真正用到的那部分接口：
 *   arm_mat_init_f32 / add / sub / mult / trans / inverse
 *   arm_atan2_f32 / arm_sin_f32 / arm_cos_f32 / arm_sqrt_f32
 *
 * 说明：矩阵求逆使用列主元高斯-约当消元，数值稳定性足够本工程的 EKF 使用。
 ******************************************************************************
 */
#include <math.h>
#include <string.h>

#include "dsp/matrix_functions.h"
#include "dsp/fast_math_functions.h"

void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nColumns, float32_t *pData)
{
    if (S == NULL)
        return;

    S->numRows = nRows;
    S->numCols = nColumns;
    S->pData = pData;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    if ((pSrcA == NULL) || (pSrcB == NULL) || (pDst == NULL))
        return ARM_MATH_ARGUMENT_ERROR;

    if ((pSrcA->numRows != pSrcB->numRows) || (pSrcA->numCols != pSrcB->numCols) ||
        (pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols))
        return ARM_MATH_SIZE_MISMATCH;

    const uint32_t n = (uint32_t)pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < n; i++)
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    if ((pSrcA == NULL) || (pSrcB == NULL) || (pDst == NULL))
        return ARM_MATH_ARGUMENT_ERROR;

    if ((pSrcA->numRows != pSrcB->numRows) || (pSrcA->numCols != pSrcB->numCols) ||
        (pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols))
        return ARM_MATH_SIZE_MISMATCH;

    const uint32_t n = (uint32_t)pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < n; i++)
        pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst)
{
    if ((pSrcA == NULL) || (pSrcB == NULL) || (pDst == NULL))
        return ARM_MATH_ARGUMENT_ERROR;

    if ((pSrcA->numCols != pSrcB->numRows) ||
        (pDst->numRows != pSrcA->numRows) || (pDst->numCols != pSrcB->numCols))
        return ARM_MATH_SIZE_MISMATCH;

    const uint16_t rows = pSrcA->numRows;
    const uint16_t inner = pSrcA->numCols;
    const uint16_t cols = pSrcB->numCols;

    for (uint16_t r = 0; r < rows; r++)
    {
        for (uint16_t c = 0; c < cols; c++)
        {
            float32_t sum = 0.0f;
            for (uint16_t k = 0; k < inner; k++)
                sum += pSrcA->pData[r * inner + k] * pSrcB->pData[k * cols + c];
            pDst->pData[r * cols + c] = sum;
        }
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst)
{
    if ((pSrc == NULL) || (pDst == NULL))
        return ARM_MATH_ARGUMENT_ERROR;

    if ((pDst->numRows != pSrc->numCols) || (pDst->numCols != pSrc->numRows))
        return ARM_MATH_SIZE_MISMATCH;

    for (uint16_t r = 0; r < pSrc->numRows; r++)
    {
        for (uint16_t c = 0; c < pSrc->numCols; c++)
            pDst->pData[c * pSrc->numRows + r] = pSrc->pData[r * pSrc->numCols + c];
    }

    return ARM_MATH_SUCCESS;
}

/* 列主元高斯-约当消元求逆，A 必须为方阵 */
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst)
{
    if ((pSrc == NULL) || (pDst == NULL))
        return ARM_MATH_ARGUMENT_ERROR;

    if (pSrc->numRows != pSrc->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    if ((pDst->numRows != pSrc->numRows) || (pDst->numCols != pSrc->numCols))
        return ARM_MATH_SIZE_MISMATCH;

    const uint16_t n = pSrc->numRows;
    if (n == 0U)
        return ARM_MATH_LENGTH_ERROR;

    /* 使用固定上限的栈上工作区，避免动态内存（控制线程中不宜 malloc） */
    enum
    {
        MAT_MAX_N = 32
    };
    if (n > MAT_MAX_N)
        return ARM_MATH_LENGTH_ERROR;

    float32_t a[MAT_MAX_N][2 * MAT_MAX_N];

    for (uint16_t r = 0; r < n; r++)
    {
        for (uint16_t c = 0; c < n; c++)
            a[r][c] = pSrc->pData[r * n + c];
        for (uint16_t c = 0; c < n; c++)
            a[r][n + c] = (r == c) ? 1.0f : 0.0f;
    }

    for (uint16_t col = 0; col < n; col++)
    {
        /* 选主元 */
        uint16_t pivot = col;
        float32_t max_val = fabsf(a[col][col]);
        for (uint16_t r = (uint16_t)(col + 1U); r < n; r++)
        {
            float32_t v = fabsf(a[r][col]);
            if (v > max_val)
            {
                max_val = v;
                pivot = r;
            }
        }

        if (max_val < 1e-12f)
            return ARM_MATH_SINGULAR;

        if (pivot != col)
        {
            for (uint16_t c = 0; c < (uint16_t)(2U * n); c++)
            {
                float32_t tmp = a[col][c];
                a[col][c] = a[pivot][c];
                a[pivot][c] = tmp;
            }
        }

        /* 归一化主元行 */
        const float32_t inv_p = 1.0f / a[col][col];
        for (uint16_t c = 0; c < (uint16_t)(2U * n); c++)
            a[col][c] *= inv_p;

        /* 消去其他行 */
        for (uint16_t r = 0; r < n; r++)
        {
            if (r == col)
                continue;
            const float32_t factor = a[r][col];
            if (factor == 0.0f)
                continue;
            for (uint16_t c = 0; c < (uint16_t)(2U * n); c++)
                a[r][c] -= factor * a[col][c];
        }
    }

    for (uint16_t r = 0; r < n; r++)
    {
        for (uint16_t c = 0; c < n; c++)
            pDst->pData[r * n + c] = a[r][n + c];
    }

    return ARM_MATH_SUCCESS;
}

/* ---------------- 数学函数 ---------------- */
void arm_atan2_f32(float32_t y, float32_t x, float32_t *result)
{
    if (result != NULL)
        *result = atan2f(y, x);
}

float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}

float32_t arm_sqrt_f32(float32_t x)
{
    return (x > 0.0f) ? sqrtf(x) : 0.0f;
}
