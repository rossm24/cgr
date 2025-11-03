#pragma once
#include "material.h"
#include <unordered_map>

struct MaterialDB {
    std::unordered_map<int, Material> by_id; // key: shape_id

    // Fallback if shape_id not registered
    const Material& get(int shape_id) const {
        static Material defaultMat; // neutral grey diffuse
        auto it = by_id.find(shape_id);
        return (it == by_id.end()) ? defaultMat : it->second;
    }
};
