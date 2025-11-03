#pragma once
#include "camera.h" // for Vec3

struct Material {
    Vec3 kd{0.8,0.8,0.8};  // diffuse
    Vec3 ks{0.0,0.0,0.0};  // specular
    double shininess = 32.0;

    double reflectivity = 0.0;  // [0,1]
    double transparency = 0.0;  // [0,1]
    double ior = 1.5;           // glass ~1.5
};

struct Light {
    Vec3 pos{0,0,0};    // point light
    Vec3 color{1,1,1};  // RGB
    double intensity = 1.0;
};
