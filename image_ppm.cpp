#include "image_ppm.h"
#include <iostream>
#include <cstring>
#include <limits>


// ---------- Constructors ----------

ImagePPM::ImagePPM(int width, int height, Pixel fill)
    : w_(width), h_(height), data_(width * height, fill)
{}

ImagePPM::ImagePPM(const std::string& filename) {
    load(filename);
}

// ---------- Pixel Access ----------

Pixel ImagePPM::get(int x, int y) const {
    if (x < 0 || x >= w_ || y < 0 || y >= h_)
        throw std::out_of_range("ImagePPM::get() out of range");
    return data_[y * w_ + x];
}

void ImagePPM::set(int x, int y, const Pixel& p) {
    if (x < 0 || x >= w_ || y < 0 || y >= h_)
        throw std::out_of_range("ImagePPM::set() out of range");
    data_[y * w_ + x] = p;
}

// ---------- I/O ----------

void ImagePPM::load(const std::string& filename) {
    std::ifstream f(filename, std::ios::binary);
    if (!f)
        throw std::runtime_error("Cannot open file: " + filename);

    std::string magic;
    f >> magic;
    if (magic != "P3" && magic != "P6")
        throw std::runtime_error("Unsupported PPM format (expected P3 or P6)");

    // skip comments
    char ch;
    while (f >> std::ws && f.peek() == '#')
        f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    int maxval;
    f >> w_ >> h_ >> maxval;
    if (!f || w_ <= 0 || h_ <= 0 || maxval != 255)
        throw std::runtime_error("Invalid PPM header");

    f.get(ch); // consume single whitespace after header

    data_.resize(w_ * h_);

    if (magic == "P6") {
        f.read(reinterpret_cast<char*>(data_.data()), w_ * h_ * 3);
        if (!f) throw std::runtime_error("Error reading binary PPM data");
    } else { // P3 (ASCII)
        for (int i = 0; i < w_ * h_; ++i) {
            int r, g, b;
            f >> r >> g >> b;
            if (!f) throw std::runtime_error("Error reading ASCII PPM data");
            data_[i] = {static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)};
        }
    }
}

// Save as P6 by default
void ImagePPM::save(const std::string& filename, bool binary) const {
    std::ofstream f(filename, std::ios::binary);
    if (!f)
        throw std::runtime_error("Cannot write to file: " + filename);

    if (binary) {
        f << "P6\n" << w_ << " " << h_ << "\n255\n";
        f.write(reinterpret_cast<const char*>(data_.data()), w_ * h_ * 3);
    } else {
        f << "P3\n" << w_ << " " << h_ << "\n255\n";
        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) {
                const Pixel& p = get(x, y);
                f << (int)p.r << " " << (int)p.g << " " << (int)p.b << " ";
            }
            f << "\n";
        }
    }

    if (!f)
        throw std::runtime_error("Error writing PPM file: " + filename);
}
