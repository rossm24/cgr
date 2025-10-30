#include "sphere.h"
#include <cmath>

bool Sphere::intersect(const Ray& worldRay, double tMin, double tMax, Hit& hit) const {
    Ray r = xform_ray(W2L, worldRay);

    // |r.o + t r.d|^2 = R^2
    Vec3 oc = r.origin;
    double a = dot(r.dir, r.dir);
    double b = 2.0 * dot(oc, r.dir);
    double c = dot(oc, oc) - radius*radius;
    double disc = b*b - 4*a*c;
    if(disc < 0.0) return false;
    double sdisc = std::sqrt(disc);

    auto try_root = [&](double t_obj)->bool{
        if(t_obj <= 1e-6) return false;
        Vec3 n_obj = normalize(oc + r.dir * t_obj); // outward normal
        return commitFromObjectSpace(worldRay, r, t_obj, n_obj, hit, tMin, tMax);
    };

    double t0 = (-b - sdisc) / (2*a);
    double t1 = (-b + sdisc) / (2*a);
    if(try_root(t0)) return true;
    if(try_root(t1)) return true;
    return false;
}
