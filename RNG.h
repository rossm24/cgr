#pragma once 
#include <random>

struct RNG {
    std::mt19937 gen;
    std::uniform_real_distribution<double> U{0.0, 1.0};
    RNG(uint32_t seed=1337) : gen(seed) {}
    inline double next() { return U(gen); }
};