#pragma once
#include "shape.h"

// Infinite plane: canonical object-space plane is z=0 with +Z normal.
// Rotate/translate via setTransform() to get any oriented plane.
class Plane : public Shape {
public:
    bool intersect(const Ray& worldRay, double tMin, double tMax, Hit& hit) const override;

    bool isFinite() const override { return false; } // BVH should skip it
    AABB objectBounds() const override { return AABB{}; } // irrelevant for infinite
};
