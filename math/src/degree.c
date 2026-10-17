#include "degree.h"

#include <math.h>

// target四舍五入成圈数
float ModularDegreeTowards(float degree, float target)
{
    return degree - roundf((degree - target) / 360.0f) * 360.0f;
}