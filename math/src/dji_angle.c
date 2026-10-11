/* 引入本模块头文件：内含 ModularDJIAngleTowards 的函数声明 */
#include "dji_angle.h"

/* 引入数学库：本文件使用 roundf（把 float 四舍五入到最近的整数） */
#include <math.h>

/*
 * 函数：ModularDJIAngleTowards
 * 用途：把大疆电机编码器角度做“模 8192”展开，得到与 target 最接近的那个等价角，
 *       即求编码器角度的最短路径。常在大疆电机（一圈 8192 计数，读数 0 ~ 8191）
 *       跨圈时使用，避免读数回绕造成 ±8192 的假跳变。
 *
 * 原理（最短路径展开公式）：
 *   result = dji_angle - round((dji_angle - target) / 8192) * 8192
 *   解读：
 *     1) (dji_angle - target)：待展开角与目标角的原始差值（单位：编码器计数）；
 *     2) 除以 8192.0f：把差值换算成“相差多少圈”；
 *     3) roundf(...)：四舍五入到最近的整数圈，得到“就近圈数”；
 *     4) 乘以 8192 再强转回 int32_t：把圈数还原成计数偏移量；
 *     5) 从 dji_angle 中减去该整圈偏移：结果与 dji_angle 机械等价，同时被拉到
 *        target 的半圈（±4096 计数）以内。
 *
 * 参数：
 *   dji_angle —— 待展开的编码器角度（单位：编码器计数，0 ~ 8191 为一圈，可表示多圈）
 *   target    —— 参考目标角度（同一编码器计数制），返回值会向其靠拢
 * 返回值：展开后的整型角度（编码器计数制），与 dji_angle 相差 8192 的整数倍，
 *         即机械指向不变，且最接近 target。
 *
 * 说明：8192.0f 为大疆编码器每圈的计数（分辨率）；使用浮点除法与 roundf 是为了
 *       对负数差值也能正确取到最近的整数圈。
 */
int32_t ModularDJIAngleTowards(int32_t dji_angle, int32_t target)
{
    /* 核心一步：先算相差圈数并就近取整，再换算成计数从原角中减去（见上方公式） */
    return dji_angle - (int32_t)roundf((dji_angle - target) / 8192.0f) * 8192;
}
