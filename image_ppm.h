#pragma once
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cstdint>

struct Pixel {
    uint8_t r{}, g{}, b{};
    Pixel() = default;
    Pixel(uint8_t R, uint8_t G, uint8_t B): r(R), g(G), b(B) {}
};

class ImagePPM {
public:
    ImagePPM() = default;

    // (a) Constructor: loads from file
    explicit ImagePPM(const std::string& filename);

    // (b) Create blank image
    ImagePPM(int width, int height, Pixel fill = {0,0,0});

    int width()  const { return w_; }
    int height() const { return h_; }

    // (b) Accessors and modifiers
    Pixel get(int x, int y) const;
    void set(int x, int y, const Pixel& p);

    // (c) Save to file (.ppm)
    void save(const std::string& filename, bool binary = true) const;

    void load(const std::string& filename);

private:
    int w_{}, h_{};
    std::vector<Pixel> data_;

    
};
