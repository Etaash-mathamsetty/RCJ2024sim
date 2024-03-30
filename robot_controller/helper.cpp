#include "constants.h"

#include <cmath>

double r2d(double decimal)
{
    return std::round(decimal * 100) / 100;
}