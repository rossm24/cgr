#include "cube.h"
#include <cmath>
#include <algorithm>

static inline std::pair<double,double> slab(double ro, double rd, double mn, double mx){
    double t0 = (mn - ro) / rd;
    double t1 = (mx - ro) / rd;
    if(t0 > t1) std::swap(t0,t1);
    return {t0,t1};
}

bool Cube::intersect(const Ray& worldRay, double tMin, double tMax, Hit& hit) const {
    Ray r = xform_ray(W2L, worldRay);

    double t0 = -std::numeric_limits<double>::infinity();
    double t1 =  std::numeric_limits<double>::infinity();

    // X
    if(std::abs(r.dir.x) < 1e-12){
        if(r.origin.x < -0.5 || r.origin.x > 0.5) return false;
    } else {
        auto [a,b] = slab(r.origin.x, r.dir.x, -0.5, 0.5);
        t0 = std::max(t0,a); t1 = std::min(t1,b);
    }
    // Y
    if(std::abs(r.dir.y) < 1e-12){
        if(r.origin.y < -0.5 || r.origin.y > 0.5) return false;
    } else {
        auto [a,b] = slab(r.origin.y, r.dir.y, -0.5, 0.5);
        t0 = std::max(t0,a); t1 = std::min(t1,b);
    }
    // Z
    if(std::abs(r.dir.z) < 1e-12){
        if(r.origin.z < -0.5 || r.origin.z > 0.5) return false;
    } else {
        auto [a,b] = slab(r.origin.z, r.dir.z, -0.5, 0.5);
        t0 = std::max(t0,a); t1 = std::min(t1,b);
    }

    if(t1 < t0 || t1 <= 1e-6) return false;

    // first positive intersection
    double t_obj = (t0 > 1e-6) ? t0 : t1;

    // object-space hit point (for normal)
    Vec3 p_obj = r.origin + r.dir * t_obj;

    // pick face normal from dominant axis
    Vec3 n_obj{0,0,0};
    double ax = std::abs(p_obj.x), ay = std::abs(p_obj.y), az = std::abs(p_obj.z);
    if(ax >= ay && ax >= az) n_obj = { (p_obj.x>0)? +1.0 : -1.0, 0, 0 };
    else if(ay >= ax && ay >= az) n_obj = { 0, (p_obj.y>0)? +1.0 : -1.0, 0 };
    else                          n_obj = { 0, 0, (p_obj.z>0)? +1.0 : -1.0 };

    // finalize (handles world tMin/tMax and closest hit logic)
    return commitFromObjectSpace(worldRay, r, t_obj, n_obj, hit, tMin, tMax);
}
