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

static Vec3 shadeHit(const Ray& ray,
                     const Hit& hit,
                     const std::vector<Light>& lights,
                     const MaterialDB& mats)
{
    if (hit.shape_id < 0) {
        return Vec3{0.0, 0.0, 0.0};
    }

    // -----------------------
    // 1. Base material (kd, ks, shininess)
    // -----------------------
    Vec3  kd        = {0.8, 0.2, 0.2};   // fallback diffuse
    Vec3  ks        = {0.1, 0.1, 0.1};   // small specular
    double shininess = 32.0;

    if (auto it = mats.by_id.find(hit.shape_id); it != mats.by_id.end()) {
        kd        = it->second.kd;
        ks        = it->second.ks;
        shininess = it->second.shininess;
    }

    // -----------------------
    // 2. Texture modulation (like Blender "Multiply", but tamed)
    // -----------------------
    const Shape* sh = hit.shape;
    if (sh && sh->texture) {
        Vec3 texCol = sh->texture->sample(hit.u, hit.v);

        // Save base kd so we can blend
        Vec3 baseKd = kd;

        // First do a straight multiply (classic texture modulate)
        kd.x *= texCol.x;
        kd.y *= texCol.y;
        kd.z *= texCol.z;

        // Then blend between base colour and multiplied colour so it
        // doesn't blow out. texStrength in [0,1].
        const double texStrength = 0.9;
        kd.x = (1.0 - texStrength) * baseKd.x + texStrength * kd.x;
        kd.y = (1.0 - texStrength) * baseKd.y + texStrength * kd.y;
        kd.z = (1.0 - texStrength) * baseKd.z + texStrength * kd.z;
    }

    // Slightly reduce specular overall so highlights don’t saturate
    ks = ks * 0.3;

    // -----------------------
    // 3. Blinn-Phong lighting
    // -----------------------
    Vec3 N = normalize(hit.n);
    Vec3 V = normalize(-ray.dir);

    Vec3 color{0.0, 0.0, 0.0};

    // Small ambient term to avoid pure black
    Vec3 ambient = 0.05 * kd;

    // Global light scale: your exported intensities are quite large
    const double lightScale = 0.02;

    for (const Light& L : lights) {
        Vec3 Ldir = normalize(L.pos - hit.p);
        double NdotL = std::max(0.0, dot(N, Ldir));
        if (NdotL <= 0.0) continue;

        // Diffuse
        Vec3 diff = kd * (lightScale * L.intensity * NdotL);

        // Blinn-Phong specular
        Vec3 H = normalize(Ldir + V);
        double NdotH = std::max(0.0, dot(N, H));
        double specTerm = std::pow(NdotH, shininess);
        Vec3 spec = ks * (lightScale * L.intensity * specTerm);

        color += diff + spec;
    }

    color += ambient;

    // -----------------------
    // 4. Clamp to [0,1]
    // -----------------------
    color.x = std::min(1.0, std::max(0.0, color.x));
    color.y = std::min(1.0, std::max(0.0, color.y));
    color.z = std::min(1.0, std::max(0.0, color.z));

    return color;
}


Vec3 shadeRay(const Ray& ray,
              int depth,
              const SceneAccel& scene,
              const std::vector<Light>& lights,
              const MaterialDB& mats,
              const RenderParams& params)
{
    if (depth > params.maxDepth) {
        return Vec3{0.0, 0.0, 0.0};
    }

    Hit hit;
    const double tMin = params.eps;
    const double tMax = std::numeric_limits<double>::infinity();

    if (!scene.intersect(ray, tMin, tMax, hit)) {
        // Background colour (same as your ImagePPM background or a gradient)
        return Vec3{0.0, 0.0, 0.0};
    }

    // Look up material
    const Material* mat = nullptr;
    if (auto it = mats.by_id.find(hit.shape_id); it != mats.by_id.end()) {
        mat = &it->second;
    }

    double reflectivity = mat ? mat->reflectivity : 0.0;
    double transparency = mat ? mat->transparency : 0.0;
    double ior          = mat ? mat->ior          : 1.0;

    // Local shading (diffuse + spec + texture) using shadeHit
    Vec3 localColor = shadeHit(ray, hit, lights, mats);

    Vec3 finalColor = localColor;

    // Reflection
    if (reflectivity > 0.0 && depth < params.maxDepth) {
        Vec3 N = hit.n;
        Vec3 Rdir = reflect(ray.dir, N);
        Ray reflRay{ hit.p + params.eps * Rdir, Rdir };

        Vec3 reflCol = shadeRay(reflRay, depth + 1, scene, lights, mats, params);

        finalColor = (1.0 - reflectivity) * finalColor + reflectivity * reflCol;
    }

    // Refraction (if you used it in Module 2; otherwise you can skip this block)
    if (transparency > 0.0 && depth < params.maxDepth) {
        Vec3 N = hit.n;
        Vec3 Tdir;
        if (refract(ray.dir, N, ior, Tdir)) {
            Ray refrRay{ hit.p + params.eps * Tdir, Tdir };
            Vec3 refrCol = shadeRay(refrRay, depth + 1, scene, lights, mats, params);
            finalColor = (1.0 - transparency) * finalColor + transparency * refrCol;
        }
    }

    // Clamp
    finalColor.x = std::min(1.0, std::max(0.0, finalColor.x));
    finalColor.y = std::min(1.0, std::max(0.0, finalColor.y));
    finalColor.z = std::min(1.0, std::max(0.0, finalColor.z));

    return finalColor;
}


/*
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

*/
