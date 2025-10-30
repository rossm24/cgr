#pragma once
#include "camera.h" // for Vec3, Ray

struct Hit {
    double t = std::numeric_limits<double>::infinity(); // world-space distance along ray
    Vec3   p{};      // world-space hit point
    Vec3   n{};      // world-space geometric normal (unit)
    int    shape_id = -1;
};
