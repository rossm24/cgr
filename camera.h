#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cctype>
#include <stdexcept>

struct Vec3 {
    double x{}, y{}, z{};
    Vec3() = default;
    Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }


    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    Vec3& operator-=(const Vec3& b){ x-=b.x; y-=b.y; z-=b.z; return *this; }

    // vector * scalar (right)
    Vec3& operator*=(double s){ x*=s; y*=s; z*=s; return *this; }

    // vector / scalar
    Vec3& operator/=(double s){ x/=s; y/=s; z/=s; return *this; }

    // unary minus
    Vec3 operator-() const { return Vec3{-x, -y, -z}; }


    // component-wise (Hadamard) multiply
    Vec3 operator*(const Vec3& b) const { return { x*b.x, y*b.y, z*b.z }; }
    Vec3& operator*=(const Vec3& b) { x*=b.x; y*=b.y; z*=b.z; return *this; }


};

// scalar * vector (left)
inline Vec3 operator*(double s, const Vec3& v){ return {v.x*s, v.y*s, v.z*s}; }

inline Vec3 cross(const Vec3& a, const Vec3& b){
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
inline double dot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
inline double length(const Vec3& v){ return std::sqrt(dot(v,v)); }
inline Vec3 normalize(const Vec3& v){ double L = length(v); return (L>0)? v/L : v; }

static inline Vec3 blenderToRenderer(const Vec3& v) {
    // Blender:  x, y=forward, z=up
    // Renderer: x, y=up,      z=forward
    return Vec3{ v.x, v.z, v.y };
}


struct Ray {
    Vec3 origin;
    Vec3 dir;   // normalized
};

class Camera {
public:
    // Loaded from scene.txt
    std::string name;
    Vec3 position{};          // world
    Vec3 forward{};           // world (gaze, from exporter)
    Vec3 right{};             // world (derived)
    Vec3 up{};                // world (derived)
    double focal_mm{};        // mm
    double sensor_w_mm{};     // mm
    double sensor_h_mm{};     // mm
    int    film_w_px{};       // pixels
    int    film_h_px{};       // pixels

    // Load the first "CAM ..." line in scene.txt
    void loadFromSceneTxt(const std::string& path);

    // Primary ray from pixel coords (px, py), pixel centers convention.
    // px in [0, film_w_px), py in [0, film_h_px), py increasing downwards.
    Ray rayFromPixel(float px, float py) const;

    // Rebuild world basis from 'forward' (+X right, +Y up, +Z world up-ish)
    void buildBasis();

private:
    // --- helpers for the ASCII format ---
    static std::string readFile(const std::string& path);
    static bool startsWith(const std::string& s, const std::string& pfx);
    static bool extractVec3(const std::string& line, const std::string& key, Vec3& out);
    static bool extractVec2(const std::string& line, const std::string& key, double& a, double& b);
    static bool extractVec2i(const std::string& line, const std::string& key, int& a, int& b);
    static bool extractNumber(const std::string& line, const std::string& key, double& out);
    static std::string extractName(const std::string& line);
};

