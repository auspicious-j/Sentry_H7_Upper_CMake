#include "degree.h"

#include <math.h>

// 滤掉 360° 的整数倍，返回最接近 target 的等价角度。
float ModularDegreeTowards(float degree, float target)
{
    return degree - roundf((degree - target) / 360.0f) * 360.0f;
}