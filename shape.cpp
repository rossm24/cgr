#include "shape.h"
#include "mat4.h"

void Shape::setTransform(const Mat4& localToWorld){
    L2W = localToWorld;
    Mat4::invert(L2W, W2L);
    NORM = Mat4::transpose(W2L); // inverse-transpose for normals
}

bool Shape::commitFromObjectSpace(const Ray& worldRay,
                                  const Ray& objRay,
                                  double t_obj,
                                  const Vec3& n_obj,
                                  Hit& hit,
                                  double tMin, double tMax) const
{
    if(t_obj <= 1e-6) return false;

    // object-space hit point
    Vec3 p_obj = objRay.origin + objRay.dir * t_obj;

    // to world
    Vec3 p_w = Mat4::mul_point(L2W, p_obj);
    Vec3 n_w = normalize(Mat4::mul_dir(NORM, n_obj));

    // compute world t via projection (assumes worldRay.dir normalized)
    double t_w = dot(p_w - worldRay.origin, worldRay.dir);

    if(t_w < tMin || t_w > tMax) return false;
    if(t_w >= hit.t) return false;

    hit.t = t_w;
    hit.p = p_w;
    hit.n = n_w;
    hit.shape_id = id;

    hit.shape = this;

    return true;
}

// transform 8 corners of a box and take min/max
static inline AABB transformAABB(const Mat4& M, const AABB& b){
    AABB out;
    for(int ix=0; ix<2; ++ix)
    for(int iy=0; iy<2; ++iy)
    for(int iz=0; iz<2; ++iz){
        Vec3 p{
            ix ? b.max.x : b.min.x,
            iy ? b.max.y : b.min.y,
            iz ? b.max.z : b.min.z
        };
        out.expand(Mat4::mul_point(M, p));
    }
    return out;
}

// Converts object-space AABB to world-space AABB
AABB Shape::worldBounds() const {
    return transformAABB(L2W, objectBounds());
}
