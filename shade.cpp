#include "shade.h"
#include <algorithm>
#include <cmath>
#include "camera.h"
#include <limits>


static inline Vec3 reflect(const Vec3& I, const Vec3& N){
    return I - 2.0 * dot(I, N) * N;
}

// I: incident toward surface (pointing from air into surface normal side)
// N: unit outward normal (will be flipped if ray is inside)
// eta = n1/n2
static inline bool refract(const Vec3& I, const Vec3& N, double eta, Vec3& T_out)
{
    // I: incident direction (towards surface), assumed normalized
    // N: surface normal, assumed normalized
    double cosi = -std::max(-1.0, std::min(1.0, dot(I, N)));
    Vec3 n = N;

    // If we are inside the medium, flip normal and invert eta
    if (cosi < 0.0) {
        cosi = -cosi;
        n    = -N;
        eta  = 1.0 / eta;
    }

    double k = 1.0 - eta * eta * (1.0 - cosi * cosi);
    if (k < 0.0) {
        // Total internal reflection
        return false;
    }

    T_out = eta * I + (eta * cosi - std::sqrt(k)) * n;
    return true;
}

// Small adapter: treat any hit < tMax as "occluded"
static inline bool bvhOccluded(const BVH& bvh, const Ray& r, double tMin, double tMax){
    Hit h; h.t = tMax;
    return bvh.intersect(r, tMin, tMax, h);
}

static Vec3 shadeHit(const Ray& ray,
                     const Hit& hit,
                     const SceneAccel& scene,
                     const std::vector<Light>& lights,
                     const MaterialDB& mats,
                     const RenderParams& params)
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

        // ---------- SHADOW RAY ----------
        Ray shadowRay{ hit.p + params.eps * Ldir, Ldir };
        Hit shadowHit;

        Vec3 toL = L.pos - hit.p;
        double distToLight = length(toL);   // <--- THIS line is essential
        //Vec3 Ldir = toL / distToLight;

        bool inShadow = scene.intersect(
            shadowRay,
            params.eps,
            distToLight - params.eps,
            shadowHit
        );

        if (inShadow) {
            continue;   // skip this light contribution
        }


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




// assume reflect(...) and refract(...) helpers are already above this

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
    double tMin = params.eps;
    double tMax = std::numeric_limits<double>::infinity();

    if (!scene.intersect(ray, tMin, tMax, hit)) {
        // no hit, background
        return Vec3{0.0, 0.0, 0.0};
    }

    // ---- Local (direct) shading with your current shadeHit ----
    Vec3 localColor = shadeHit(ray, hit, scene, lights, mats, params);

    // Get per-shape material
    const Material& mat = mats.get(hit.shape_id);

    // Map your fields:
    // - kr/kt (if you ever set them explicitly)
    // - otherwise use reflectivity/transparency
    double kr = (mat.kr != 0.0) ? mat.kr : mat.reflectivity;
    double kt = (mat.kt != 0.0) ? mat.kt : mat.transparency;

    // clamp to [0,1]
    kr = std::max(0.0, std::min(1.0, kr));
    kt = std::max(0.0, std::min(1.0, kt));

    double ior = (mat.ior > 0.0) ? mat.ior : 1.0;

    // If nothing reflective/refractive or max depth reached, just return local shading
    if ((kr <= 0.0 && kt <= 0.0) || depth == params.maxDepth) {
        return localColor;
    }

    Vec3 N = normalize(hit.n);
    Vec3 V = -normalize(ray.dir);  // from surface to eye

    Vec3 result(0.0, 0.0, 0.0);

    // Keep some of the local term (you can tweak this)
    double baseWeight = std::max(0.0, 1.0 - kr - kt);
    result += baseWeight * localColor;

    // -------------------
    // Reflection
    // -------------------
    if (kr > 0.0) {
        Vec3 Rdir = reflect(-V, N);   // incident = -V
        Rdir = normalize(Rdir);

        Ray reflRay;
        reflRay.origin = hit.p + params.eps * Rdir;  // small bias
        reflRay.dir = Rdir;

        Vec3 reflColor = shadeRay(reflRay, depth + 1, scene, lights, mats, params);
        result += kr * reflColor;
    }

    // -------------------
    // Refraction
    // -------------------
    if (kt > 0.0 && ior != 1.0) {
        Vec3 Tdir;
        bool ok = refract(-V, N, 1.0 / ior, Tdir);  // air -> material

        if (ok) {
            Tdir = normalize(Tdir);

            Ray refrRay;
            refrRay.origin = hit.p + params.eps * Tdir;
            refrRay.dir = Tdir;

            Vec3 refrColor = shadeRay(refrRay, depth + 1, scene, lights, mats, params);
            result += kt * refrColor;
        }
        // else: total internal reflection (reflection already handled above)
    }

    return result;
}






/*
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
    Vec3 localColor = shadeHit(ray, hit, scene, lights, mats, params);

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

*/
