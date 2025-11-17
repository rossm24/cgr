#pragma once
#include <cmath>
#include "camera.h" // Vec3

struct Mat4 {
    // Row-major
    double m[16];

    // Constructors
    Mat4() {
        for (int i = 0; i < 16; ++i) m[i] = 0.0;
    }

    static Mat4 identity();
    static Mat4 translate(const Vec3& t);
    static Mat4 scale(const Vec3& s);
    static Mat4 rotateX(double radians);
    static Mat4 rotateY(double radians);
    static Mat4 rotateZ(double radians);

    // Matrix ops
    static Mat4 multiply(const Mat4& A, const Mat4& B);
    static Mat4 transpose(const Mat4& A);
    static bool invert(const Mat4& A, Mat4& out);

    // Apply to points/dirs
    static Vec3 mul_point(const Mat4& M, const Vec3& p);
    static Vec3 mul_dir  (const Mat4& M, const Vec3& v);
};
