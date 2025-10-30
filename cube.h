#pragma once
#include "shape.h"

// Unit axis-aligned cube in object space: [-0.5, 0.5]^3
class Cube : public Shape {
public:
    bool intersect(const Ray& worldRay, double tMin, double tMax, Hit& hit) const override;

    AABB objectBounds() const override {
        return AABB{ {-0.5,-0.5,-0.5}, {+0.5,+0.5,+0.5} };
    }
};
