#pragma once
#include "camera.h" // for Vec3

struct Material {
    Vec3 kd;  // diffuse
    Vec3 ks;  // specular
    double shininess;

    double kr = 0.0;  
    double kt = 0.0;
    

    double reflectivity = 0.0;  // [0,1]
    double transparency = 0.0;  // [0,1]
    double ior = 1.5;           // glass ~1.5

    const Texture* texture = nullptr;
};

struct Light {
    Vec3 pos{0,0,0};    // point light
    Vec3 color{1,1,1};  // RGB
    double intensity = 1.0;
};