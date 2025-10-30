#pragma once
#include "shape.h"

class Sphere : public Shape {
public:
    double radius = 1.0; // object-space radius (center at origin)

    bool intersect(const Ray& worldRay, double tMin, double tMax, Hit& hit) const override;

    AABB objectBounds() const override {
        return AABB{ {-radius,-radius,-radius}, {+radius,+radius,+radius} };
    }
};
