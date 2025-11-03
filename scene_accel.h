// scene_accel.h
#pragma once
#include "bvh.h"
#include "shape.h"
#include <vector>

struct SceneAccel {
    const BVH* bvh = nullptr;
    const std::vector<Shape*>* infinite = nullptr;

    bool intersect(const Ray& r, double tMin, double tMax, Hit& hit) const {
        bool any = false;
        if (bvh) any |= bvh->intersect(r, tMin, tMax, hit);
        if (infinite) {
            for (Shape* s : *infinite)
                any |= s->intersect(r, tMin, tMax, hit);
        }
        return any;
    }
    bool occluded(const Ray& r, double tMin, double tMax) const {
        Hit h; h.t = tMax;
        return intersect(r, tMin, tMax, h);
    }
};
