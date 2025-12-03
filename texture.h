#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include "camera.h"   // your Vec3

struct Texture {
    int width  = 0;
    int height = 0;
    std::vector<Vec3> pixels; // row-major, size = width * height

    bool loadPPM(const std::string& filename);

    // (u,v) in [0,1], repeated tiling
    Vec3 sample(double u, double v) const;
};

