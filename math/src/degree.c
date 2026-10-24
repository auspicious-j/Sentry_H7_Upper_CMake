#include "degree.h"

#include <math.h>

// 将 Target 限制在360度范围内
float ModularDegreeTowards(float degree, float target)
{
    return degree - roundf((degree - target) / 360.0f) * 360.0f;
}