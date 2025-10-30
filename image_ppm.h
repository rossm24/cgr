/*
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <sstream>

class ImagePPM {
public:
    struct Pixel { uint8_t r,g,b; };

    // (a) constructor loading from file
    explicit ImagePPM(const std::string& filename) { load(filename); }

    // optional: blank image
    ImagePPM(int w, int h, Pixel fill = {0,0,0})
        : w_(w), h_(h), data_(w*h*3u, 0)
    {
        for (int y=0; y<h; ++y)
            for (int x=0; x<w; ++x)
                set(x,y,fill);
    }

    // (b) read/modify pixel values
    int  width()  const { return w_; }
    int  height() const { return h_; }

    Pixel get(int x, int y) const {
        size_t i = (size_t(y)*w_ + x)*3;
        return { data_[i+0], data_[i+1], data_[i+2] };
    }

    void set(int x, int y, Pixel p) {
        size_t i = (size_t(y)*w_ + x)*3;
        data_[i+0]=p.r; data_[i+1]=p.g; data_[i+2]=p.b;
    }

    // (c) write to file (P6, maxval=255)
    void save(const std::string& filename) const {
        std::ofstream f(filename, std::ios::binary);
        if(!f) throw std::runtime_error("Cannot write " + filename);
        f << "P6\n" << w_ << " " << h_ << "\n255\n";
        f.write(reinterpret_cast<const char*>(data_.data()),
                std::streamsize(data_.size()));
        if(!f) throw std::runtime_error("Write failed");
    }

private:
    int w_ = 0, h_ = 0;
    std::vector<uint8_t> data_;

    void load(const std::string& filename) {
        std::ifstream f(filename, std::ios::binary);
        if(!f) throw std::runtime_error("Cannot open " + filename);

        std::string magic; f >> magic;
        if (magic != "P6") throw std::runtime_error("Only P6 supported");

        // read width, height, maxval
        int maxval = 0;
        f >> w_ >> h_ >> maxval;
        if(!f || w_<=0 || h_<=0 || maxval!=255)
            throw std::runtime_error("Bad header (need maxval=255)");

        // consume single whitespace after header before binary pixel data
        f.get(); // one whitespace

        data_.resize(size_t(w_)*h_*3u);
        f.read(reinterpret_cast<char*>(data_.data()),
               std::streamsize(data_.size()));
        if (size_t(f.gcount()) != data_.size())
            throw std::runtime_error("Unexpected EOF in pixel data");
    }
};

*/

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

private:
    int w_{}, h_{};
    std::vector<Pixel> data_;

    void load(const std::string& filename);
};
