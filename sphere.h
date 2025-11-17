/*
#pragma once

#include <string>
#include "shape.h"   // for Shape, Ray, Hit, AABB, Vec3

class Sphere : public Shape {
public:
    // create an ellipsoid-like sphere
    Sphere(const std::string& name,
           float px, float py, float pz,
           float sx, float sy, float sz);

    // main ray–shape intersection (engine uses doubles here)
    bool intersect(const Ray& worldRay,
                   double tMin,
                   double tMax,
                   Hit& hit) const override;

    // local-space bounds (unit sphere)
    AABB objectBounds() const override;

    void print_info() const;

private:
    std::string m_name;

    // store in float (your requirement)
    float m_px, m_py, m_pz;   // position
    float m_sx, m_sy, m_sz;   // scale (non-uniform → ellipsoid)
    
};
*/

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

    AABB objectBounds() const override;

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








