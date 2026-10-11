#include "degree.h"   /* 本模块自身的接口声明 */

#include <math.h>     /* 用到 roundf：对浮点数做四舍五入 */

/*
 * 用途：把当前角度 degree 折算到以 target 为中心、跨度 360 度的区间内的等价角度，
 *       使 (返回值 - target) 落在 [-180, 180] 度之间，从而直接得到最短转向路径。
 * 参数：
 *   - degree：当前角度（单位：度，任意实数）。
 *   - target：目标角度（单位：度，任意实数）。
 * 返回：与 degree 相差 360 度整数倍、且最接近 target 的等价角度（单位：度），
 *       取值范围 [target-180, target+180]。
 */
float ModularDegreeTowards(float degree, float target)
{
    /* 计算步骤（每一步都在把"角度"换算成"圈数"再换算回"角度"）：
     * 1) (degree - target) / 360.0f：把 degree 相对 target 的偏差换算成"圈数"（浮点，未必是整数）。
     * 2) roundf(...)：四舍五入取到最近的整数圈数，即"应当扣掉的圈数"（正圈或负圈都能处理）。
     * 3) * 360.0f：把圈数还原为角度（度）。
     * 4) degree 减去该角度：得到与 degree 等价、但已折叠到 target 附近 ±180 度内的角度。
     * 为什么这样做：一次性消掉多余的正圈/负圈，得到最短转向路径所需的角度，且结果落在 ±180 度内。 */
    return degree - roundf((degree - target) / 360.0f) * 360.0f;
}
