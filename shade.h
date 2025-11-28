#pragma once
#include "bvh.h"
#include "material_db.h"
#include "scene_accel.h"
#include <vector>
#include "RNG.h"

struct Light;


struct RenderParams {
    // --- Core raytracer controls ---
    int    maxDepth   = 6;
    double eps        = 1e-4;   // bias for secondary rays

    // --- Sampling / anti-aliasing ---
    int    spp        = 1;      // samples per pixel
    bool   stratified = true;   // use n×n stratified vs pure jitter

    // --- Soft shadows (will use later) ---
    bool   enableSoftShadows = false;
    int    shadowSamples     = 1;   // shadow rays per light
    double softShadowRadius  = 0.25;  // radius of area light disk (tweak)

    // --- Glossy reflection (will use later) ---
    bool   enableGlossy      = false;
    int    glossySamples     = 1;
    double glossyRoughness   = 0.0; // 0 = perfect mirror

    // --- Depth of field (will use later) ---
    bool   enableDOF         = false;
    double apertureRadius    = 0.0;
    double focalDistance     = 5.0;

    // --- Motion blur (will use later) ---
    bool   enableMotionBlur  = false;
    double shutterOpen       = 0.0;
    double shutterClose      = 1.0;
    double camMotionAmount   = 0.0;  // how far camera moves along its right axis over shutter

    // --- Bloom  ---
    bool   enableBloom       = false;
    double bloomThreshold    = 1.0;  // luminance above this counts as "bright"
    double bloomStrength     = 0.7;  // how much bloom to add

    // --- Lens flare ---
    bool   enableLensFlare   = false;
    double flareStrength     = 0.4;  // overall strength of ghosts

    double lightScale = 1.0;   // default 
};

Vec3 shadeRay(const Ray& ray,
              int depth,
              const SceneAccel& scene,
              const std::vector<Light>& lights,
              const MaterialDB& mats,
              const RenderParams& params,
              RNG& rng);



/*
struct RenderParams {
    int    maxDepth = 6;
    double eps      = 1e-4; // bias for secondary rays
};
*/