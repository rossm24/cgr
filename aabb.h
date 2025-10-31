#pragma once
#include "camera.h" // Vec3, Ray
#include <algorithm>
#include <limits>
#include <cmath>

// axis-aligned bounding box 
// if ray does not intersect with the bounding box then it cannot possibly hit any of the target objects inside
// so we can skip these computations completely as there is nothing of interest to us in this area

struct AABB {
    Vec3 min{  std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::infinity() };
    Vec3 max{ -std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity() };

    AABB() = default;
    AABB(const Vec3& mn, const Vec3& mx) : min(mn), max(mx) {}          

    void expand(const Vec3& p){
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
    }
    void expand(const AABB& b){ expand(b.min); expand(b.max); }

    Vec3 extent() const { return { max.x-min.x, max.y-min.y, max.z-min.z }; }
    double surfaceArea() const {
        Vec3 e = extent();
        return 2.0 * (e.x*e.y + e.y*e.z + e.z*e.x);
    }
    int longestAxis() const {
        Vec3 e = extent();
        if(e.x >= e.y && e.x >= e.z) return 0;
        if(e.y >= e.x && e.y >= e.z) return 1;
        return 2;
    }

    // Standard slab test
    bool intersect(const Ray& r, double tmin, double tmax) const {
        for(int a=0;a<3;++a){
            double ro = (a==0? r.origin.x : a==1? r.origin.y : r.origin.z);
            double rd = (a==0? r.dir.x    : a==1? r.dir.y    : r.dir.z);
            double inv = 1.0 / rd;
            double t0 = (((a==0)?min.x:(a==1)?min.y:min.z) - ro) * inv;
            double t1 = (((a==0)?max.x:(a==1)?max.y:max.z) - ro) * inv;
            if(t0 > t1) std::swap(t0, t1);
            tmin = t0 > tmin ? t0 : tmin;
            tmax = t1 < tmax ? t1 : tmax;
            if(tmax < tmin) return false;
        }
        return true;
    }
};
