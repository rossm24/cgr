#pragma once

#include <string>
#include <cmath>
#include "shape.h"   // Ray, Hit, Vec3, AABB, Shape base

class Sphere : public Shape {
public:
    Sphere(const std::string& name,
           float px, float py, float pz,
           float sx, float sy, float sz,
           float rx, float ry, float rz);

    void print_info() const;

    AABB objectBounds() const override {
        // unit sphere in object space
        return AABB(Vec3(-1.0, -1.0, -1.0),
                    Vec3( 1.0,  1.0,  1.0));
    }

    bool intersect(const Ray& worldRay,
                   double tMin,
                   double tMax,
                   Hit& hit) const override;

private:
    std::string m_name;
    float m_px, m_py, m_pz;   // centre
    float m_sx, m_sy, m_sz;   // per-axis “radius”
    float m_rx, m_ry, m_rz;   // Euler angles (same units as exporter)
};








