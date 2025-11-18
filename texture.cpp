#include "texture.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>   // for std::min / std::max

bool Texture::loadPPM(const std::string& filename)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        std::cerr << "Cannot open texture file " << filename << "\n";
        return false;
    }

    std::string magic;
    in >> magic;
    if (magic != "P6") {
        std::cerr << "Only P6 binary PPM is supported: " << filename << "\n";
        return false;
    }

    int maxval;
    in >> width >> height >> maxval;
    in.get(); // consume single whitespace after header

    if (width <= 0 || height <= 0 || maxval <= 0) {
        std::cerr << "Bad PPM header in " << filename << "\n";
        return false;
    }

    std::vector<unsigned char> buf(3 * width * height);
    in.read(reinterpret_cast<char*>(buf.data()), buf.size());
    if (!in) {
        std::cerr << "Unexpected EOF in " << filename << "\n";
        return false;
    }

    pixels.resize(width * height);
    double inv = 1.0 / double(maxval);
    for (int i = 0; i < width * height; ++i) {
        double r = buf[3*i + 0] * inv;
        double g = buf[3*i + 1] * inv;
        double b = buf[3*i + 2] * inv;
        pixels[i] = Vec3{r,g,b};
    }
    return true;
}

Vec3 Texture::sample(double u, double v) const
{
    if (width == 0 || height == 0 || pixels.empty()) {
        return Vec3{1.0, 1.0, 1.0}; // fallback: white
    }

    // Wrap so that values outside [0,1] repeat
    u = u - std::floor(u);
    v = v - std::floor(v);

    // PPM is top-to-bottom; flip v
    double x = u * (width  - 1);
    double y = (1.0 - v) * (height - 1);

    int ix = (int)std::round(x);
    int iy = (int)std::round(y);

    ix = std::max(0, std::min(width  - 1, ix));
    iy = std::max(0, std::min(height - 1, iy));

    return pixels[iy * width + ix];
}
