#include "shade.h"
#include <algorithm>
#include <cmath>
#include "camera.h"
#include <limits>
#include "RNG.h"


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

static double shadowVisibility(const Hit& hit,
                               const Light& L,
                               const SceneAccel& scene,
                               const RenderParams& params,
                               RNG& rng)
{
    // Vector and distance to the light *centre*
    Vec3 toL         = L.pos - hit.p;
    double distToLight = length(toL);
    if (distToLight <= 0.0) {
        return 0.0;
    }
    Vec3 baseDir     = toL / distToLight;

    // --- Old behaviour: hard shadows if disabled or 1 sample ---
    if (!params.enableSoftShadows ||
        params.shadowSamples <= 1 ||
        params.softShadowRadius <= 0.0)
    {
        Ray shadowRay;
        shadowRay.origin = hit.p + params.eps * baseDir;
        shadowRay.dir    = baseDir;

        Hit tmp;
        bool inShadow = scene.intersect(
            shadowRay,
            params.eps,
            distToLight - params.eps,
            tmp
        );

        return inShadow ? 0.0 : 1.0;
    }

    // --- Soft shadows: sample a small disk area around the light ---
    int   nSamples = params.shadowSamples;
    double radius  = params.softShadowRadius;
    double vis     = 0.0;

    for (int s = 0; s < nSamples; ++s) {
        // random point in unit disk (polar sampling)
        double r   = std::sqrt(rng.next());
        double phi = 2.0 * 3.14159265358979323846 * rng.next();
        double dx  = radius * r * std::cos(phi);
        double dz  = radius * r * std::sin(phi);

        // simple disk in XZ-plane around light centre
        Vec3 lp = L.pos + Vec3{dx, 0.0, dz};

        Vec3 toSample = lp - hit.p;
        double dist   = length(toSample);
        if (dist <= 0.0) continue;
        Vec3 dir      = toSample / dist;

        Ray shadowRay;
        shadowRay.origin = hit.p + params.eps * dir;
        shadowRay.dir    = dir;

        Hit tmp;
        bool blocked = scene.intersect(
            shadowRay,
            params.eps,
            dist - params.eps,
            tmp
        );

        if (!blocked) {
            vis += 1.0; // this sample sees the light
        }
    }

    return vis / double(nSamples);
}

static Vec3 sampleAroundDirection(const Vec3& dir,
                                  double roughness,
                                  RNG& rng)
{
    Vec3 w = normalize(dir);
    Vec3 a = (std::fabs(w.x) > 0.1) ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 v = normalize(cross(w, a));
    Vec3 u = cross(v, w);

    // Simple jitter in tangent plane
    double rx = (rng.next() - 0.5) * roughness;
    double ry = (rng.next() - 0.5) * roughness;

    Vec3 jittered = w + rx * u + ry * v;
    return normalize(jittered);
}




static Vec3 shadeHit(const Ray& ray,
                     const Hit& hit,
                     const SceneAccel& scene,
                     const std::vector<Light>& lights,
                     const MaterialDB& mats,
                     const RenderParams& params,
                     RNG& rng)
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

        // --- NEW: visibility in [0,1] instead of hard inShadow ---
        double vis = shadowVisibility(hit, L, scene, params, rng);
        if (vis <= 0.0) {
            continue;   // fully in shadow for this light
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

        // --- NEW: scale by visibility ---
        color += vis * (diff + spec);
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
              const RenderParams& params,
              RNG& rng)
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
    Vec3 localColor = shadeHit(ray, hit, scene, lights, mats, params, rng);

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
    // Reflection (perfect or glossy)
    // -------------------
    if (kr > 0.0) {
        Vec3 Rdir = reflect(-V, N);
        Rdir = normalize(Rdir);

        int   numSamples = 1;
        double rough     = 0.0;

        // Only use glossy if explicitly enabled and sensible params
        if (params.enableGlossy &&
            params.glossySamples > 1 &&
            params.glossyRoughness > 0.0)
        {
            numSamples = params.glossySamples;
            rough      = params.glossyRoughness;
        }

        Vec3 reflAccum{0,0,0};

        for (int i = 0; i < numSamples; ++i) {
            Vec3 dir = (rough > 0.0)
                    ? sampleAroundDirection(Rdir, rough, rng)
                    : Rdir;

            Ray reflRay;
            reflRay.origin = hit.p + params.eps * dir;
            reflRay.dir    = dir;

            Vec3 rc = shadeRay(reflRay, depth + 1, scene, lights, mats, params, rng);
            reflAccum += rc;
        }

        reflAccum /= double(numSamples);
        result += kr * reflAccum;
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

            Vec3 refrColor = shadeRay(refrRay, depth + 1, scene, lights, mats, params, rng);
            result += kt * refrColor;
        }
        // else: total internal reflection (reflection already handled above)
    }

    return result;
}
