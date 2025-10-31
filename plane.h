/*
#pragma once

#include "shape.h"
#include <string>

class Plane : public Shape {
public:
    // 4-corner plane, like your friend's
    Plane(const std::string& name,
          double c1x, double c1y, double c1z,
          double c2x, double c2y, double c2z,
          double c3x, double c3y, double c3z,
          double c4x, double c4y, double c4z);

    bool intersect(const Ray& worldRay,
                   double tMin,
                   double tMax,
                   Hit& hit) const override;

    AABB objectBounds() const override;

    void print_info() const;

private:
    std::string m_name;
    Vec3 c1, c2, c3, c4;   // corners in *object* space for us
};
*/

#pragma once

#include "shape.h"
#include <string>

class Plane : public Shape {
public:
    Plane(const std::string& name,
          double c0x, double c0y, double c0z,
          double c1x, double c1y, double c1z,
          double c2x, double c2y, double c2z,
          double c3x, double c3y, double c3z);

    bool intersect(const Ray& worldRay,
                   double tMin,
                   double tMax,
                   Hit& hit) const override;

    AABB objectBounds() const override;

    void print_info() const;

private:
    std::string m_name;
    Vec3 c0, c1, c2, c3;   // NOTE: 0,1,2,3 now
};

