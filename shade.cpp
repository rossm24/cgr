#include "shade.h"
#include <algorithm>
#include <cmath>
#include "camera.h"


static inline Vec3 reflect(const Vec3& I, const Vec3& N){
    return I - 2.0 * dot(I, N) * N;
}

// I: incident toward surface (pointing from air into surface normal side)
// N: unit outward normal (will be flipped if ray is inside)
// eta = n1/n2
static inline bool refract(const Vec3& I, const Vec3& N, double eta, Vec3& T){
    double cosi = std::clamp(dot(I, N), -1.0, 1.0);
    double sin2t = eta*eta * (1.0 - cosi*cosi);
    if (sin2t > 1.0) return false; // total internal reflection
    double cost = std::sqrt(std::max(0.0, 1.0 - sin2t));
    T = eta * (-I) + (eta * cosi - cost) * N;
    return true;
}

// Small adapter: treat any hit < tMax as "occluded"
static inline bool bvhOccluded(const BVH& bvh, const Ray& r, double tMin, double tMax){
    Hit h; h.t = tMax;
    return bvh.intersect(r, tMin, tMax, h);
}

Vec3 shadeRay(const Ray& ray,
              int depth,
              const SceneAccel& scene,
              const std::vector<Light>& lights,
              const MaterialDB& mats,
              const RenderParams& P)
{
    Hit hit;
    if(!scene.intersect(ray, 1e-5, std::numeric_limits<double>::infinity(), hit)){
        // Background
        return {0,0,0};
    }

    // Geometry at hit
    const Vec3 Pworld = hit.p;
    Vec3 N = normalize(hit.n);
    const Vec3 V = normalize(-ray.dir); // toward camera

    // Material by shape_id (no Hit changes needed)
    const Material& M = mats.get(hit.shape_id);

    // Ensure normal faces against incoming ray for correct reflection/refraction
    const bool outside = dot(ray.dir, N) < 0.0;
    Vec3 Nf = outside ? N : -N;

    // --- Local Blinn-Phong ---
    Vec3 Lo{0,0,0};
    Lo += 0.15 * M.kd; // ambient

    for(const auto& Lgt : lights){
        Vec3 Ldir = Lgt.pos - Pworld;
        double dist2 = dot(Ldir, Ldir);
        double dist  = std::sqrt(std::max(0.0, dist2));
        if(dist > 0.0) Ldir /= dist;

        // Shadow test
        Ray sray{ Pworld + Nf * P.eps, Ldir };
        if(scene.occluded(sray, 1e-5, dist - 1e-4)) continue;

        double ndotl = std::max(0.0, dot(Nf, Ldir));
        Vec3 diff = M.kd * ndotl;

        Vec3 H = normalize(Ldir + V);
        double ndoth = std::max(0.0, dot(Nf, H));
        Vec3 spec = M.ks * std::pow(ndoth, M.shininess);

        //double att = 1.0 / std::max(1.0, dist2); // simple falloff
        double att = 1.0;
        Lo += (diff + spec) * Lgt.color * (Lgt.intensity * att);
    }

    if (depth >= P.maxDepth) return Lo;

    // --- Secondary (reflection / refraction) ---
    Vec3 Lr{0,0,0}, Lt{0,0,0};

    if (M.reflectivity > 0.0){
        Vec3 Rdir = reflect(-V, Nf);
        Ray rr{ Pworld + Nf * P.eps, normalize(Rdir) };
        Lr = shadeRay(rr, depth+1, scene, lights, mats, P);
    }

    if (M.transparency > 0.0){
        double n1 = outside ? 1.0 : M.ior;
        double n2 = outside ? M.ior : 1.0;
        double eta = n1 / n2;

        Vec3 I = -V;
        Vec3 Tdir;
        if (refract(I, Nf, eta, Tdir)){
            // When entering, offset along -Nf (opposite) to avoid self-hit on exit surfaces,
            // but with single-surface objects either side works; this is robust:
            Ray tr{ Pworld - Nf * P.eps, normalize(Tdir) };
            Lt = shadeRay(tr, depth+1, scene, lights, mats, P);
        }else{
            // TIR => pure reflection if not already reflective
            if (M.reflectivity == 0.0){
                Vec3 Rdir = reflect(-V, Nf);
                Ray rr{ Pworld + Nf * P.eps, normalize(Rdir) };
                Lr = shadeRay(rr, depth+1, scene, lights, mats, P);
            }
        }
    }

    // Simple energy split (you can switch to Fresnel later)
    double Kr = std::clamp(M.reflectivity, 0.0, 1.0);
    double Kt = std::clamp(M.transparency, 0.0, 1.0);
    double Kl = std::max(0.0, 1.0 - Kr - Kt);

    return Kl * Lo + Kr * Lr + Kt * Lt;
}
