#pragma once
#include "bvh.h"
#include "material_db.h"
#include "scene_accel.h"
#include <vector>

struct Light;


struct RenderParams {
    int    maxDepth = 6;
    double eps      = 1e-4; // bias for secondary rays
};

Vec3 shadeRay(const Ray& ray,
              int depth,
              const SceneAccel& scene,
              const std::vector<Light>& lights,
              const MaterialDB& mats,
              const RenderParams& params);
