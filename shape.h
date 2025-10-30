#pragma once
#include "camera.h" // Ray, Vec3, normalize, dot
#include "mat4.h"
#include "hit.h"
#include "aabb.h"

class Shape {
public:
    virtual ~Shape() = default;

    // public id (optional)
    int id = -1;

    // set/get transforms
    void setTransform(const Mat4& localToWorld);

    // intersect with world-space ray; update 'hit' if closer in [tMin, tMax]
    virtual bool intersect(const Ray& worldRay, double tMin, double tMax, Hit& hit) const = 0;

    virtual bool isFinite() const { return true; }     // Plane will override to false
    virtual AABB objectBounds() const = 0;             // Each shape gives its local-space bounds
    AABB worldBounds() const;                          // Converts to world-space using L2W

protected:
    // cached transforms
    Mat4 L2W = Mat4::identity();
    Mat4 W2L = Mat4::identity();
    Mat4 NORM = Mat4::identity(); // inverse-transpose for normals

    // utility for derived classes to finalize a hit given object-space t and normal
    bool commitFromObjectSpace(const Ray& worldRay,
                               const Ray& objRay,
                               double t_obj,
                               const Vec3& n_obj,
                               Hit& hit,
                               double tMin, double tMax) const;
    // transform a ray by a matrix (origin as point, dir as vector)
    static inline Ray xform_ray(const Mat4& M, const Ray& r){
        return { Mat4::mul_point(M, r.origin), Mat4::mul_dir(M, r.dir) };
    }
};
