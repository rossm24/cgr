#pragma once
#include <string>
#include <cmath>        // for std::floor
#include "camera.h"     // for Vec3
#include "image_ppm.h"  // your existing image class

class Texture {
public:
    ImagePPM img;

    bool load(const std::string& filename) {
        try {
            img.load(filename);   // uses your existing P3/P6 loader
            return true;
        } catch (const std::exception& e) {
            // You can add a debug print here if you want
            return false;
        }
    }

    // Sample with [0,1] x [0,1] UV; wrapping & v-flip
    Vec3 sample(double u, double v) const {
        const int w = img.width();
        const int h = img.height();
        if (w <= 0 || h <= 0) {
            // bright magenta to show missing textures
            return Vec3{1.0, 0.0, 1.0};
        }

        // Wrap to [0,1)
        u = u - std::floor(u);
        v = v - std::floor(v);

        int x = static_cast<int>(u * w);
        int y = static_cast<int>((1.0 - v) * h); // flip v so v=0 is bottom

        if (x < 0) x = 0;
        if (x >= w) x = w - 1;
        if (y < 0) y = 0;
        if (y >= h) y = h - 1;

        const Pixel& p = img.get(x, y);   // you already use get(x,y) in save()

        // Convert 0–255 -> 0–1
        return Vec3{
            p.r / 255.0,
            p.g / 255.0,
            p.b / 255.0
        };
    }
};

