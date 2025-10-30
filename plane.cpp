#include "plane.h"
#include <cmath>

bool Plane::intersect(const Ray& worldRay, double tMin, double tMax, Hit& hit) const {
    Ray r = xform_ray(W2L, worldRay);

    double denom = r.dir.z;
    if(std::abs(denom) < 1e-9) return false; // parallel

    double t_obj = -r.origin.z / denom;
    if(t_obj <= 1e-6) return false;

    // face normal points against incoming ray in object space
    Vec3 n_obj = {0,0, (denom < 0.0) ? 1.0 : -1.0};
    return commitFromObjectSpace(worldRay, r, t_obj, n_obj, hit, tMin, tMax);
}
