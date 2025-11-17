#pragma once
#include "camera.h" // for Vec3, Ray

class Shape; // forward declare

struct Hit {
    double t = std::numeric_limits<double>::infinity(); // world-space distance along ray
    Vec3   p{};      // world-space hit point
    Vec3   n{};      // world-space geometric normal (unit)
    int    shape_id = -1;

    double u = 0.0; 
    double v = 0.0;

    const Shape* shape = nullptr; // hit shape (nullptr if none)
};
