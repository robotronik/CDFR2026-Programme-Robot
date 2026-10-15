#pragma once

#include <cmath>

namespace vision {

// Angle helpers shared by the localisers and the pose conversions.
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

// Wraps an angle into ]-180, 180].
inline double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

} // namespace vision
