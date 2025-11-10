#include "sphere.h"
#include <cmath>
#include <limits>
#include <iostream>

// helper tiny struct just for local float math
struct f3 {
    float x, y, z;
    f3() : x(0), y(0), z(0) {}
    f3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

    f3 operator+(const f3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    f3 operator-(const f3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    f3 operator*(float s)    const { return {x*s, y*s, z*s}; }
    f3 operator/(float s)    const { return {x/s, y/s, z/s}; }

    // component-wise
    f3 cdiv(const f3& b) const { return {x/b.x, y/b.y, z/b.z}; }
    f3 cmul(const f3& b) const { return {x*b.x, y*b.y, z*b.z}; }
};

static inline float fdot(const f3& a, const f3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline float flen(const f3& v) { return std::sqrt(fdot(v,v)); }
static inline f3 fnorm(const f3& v) {
    float L = flen(v);
    return (L > 0.0f) ? v / L : v;
}

// ---------------- ctor ----------------
Sphere::Sphere(const std::string& name,
               float px, float py, float pz,
               float sx, float sy, float sz)
    : m_name(name),
      m_px(px), m_py(py), m_pz(pz),
      m_sx(sx), m_sy(sy), m_sz(sz)
{ }

void Sphere::print_info() const {
    std::cout << "Sphere: " << m_name << "\n"
              << "  pos:   (" << m_px << ", " << m_py << ", " << m_pz << ")\n"
              << "  scale: (" << m_sx << ", " << m_sy << ", " << m_sz << ")\n\n";
}





AABB Sphere::objectBounds() const
{
    // unit sphere in object space
    return AABB(Vec3(-1.0, -1.0, -1.0),
                Vec3( 1.0,  1.0,  1.0));
}



bool Sphere::intersect(const Ray& worldRay,
                       double tMin,
                       double tMax,
                       Hit& hit) const
{
    // 1) world (double) -> float
    f3 rof( (float)worldRay.origin.x,
            (float)worldRay.origin.y,
            (float)worldRay.origin.z );
    f3 rdf( (float)worldRay.dir.x,
            (float)worldRay.dir.y,
            (float)worldRay.dir.z );

    f3 posf(m_px, m_py, m_pz);
    f3 scalef(m_sx, m_sy, m_sz);

    // 2) ray into ellipsoid local space (float)
    f3 oc  = (rof - posf).cdiv(scalef);
    f3 dir =  rdf.cdiv(scalef);

    float a = fdot(dir, dir);
    float b = 2.0f * fdot(oc, dir);
    float c = fdot(oc, oc) - 1.0f;

    float disc = b*b - 4.0f*a*c;
    if (disc < 0.0f)
        return false;

    float sqrtD = std::sqrt(disc);
    float t1 = (-b - sqrtD) / (2.0f * a);
    float t2 = (-b + sqrtD) / (2.0f * a);

    const float EPS = 1e-7f;     // slightly looser than before

    auto try_root = [&](float t_obj_f) -> bool {
        if (t_obj_f <= EPS)
            return false;

        // hit point in object space (float)
        f3 p_obj = oc + dir * t_obj_f;

        // normal in object space (float)
        f3 scale_sq = scalef.cmul(scalef);
        f3 n_obj_f( p_obj.x / scale_sq.x,
                    p_obj.y / scale_sq.y,
                    p_obj.z / scale_sq.z );
        n_obj_f = fnorm(n_obj_f);

        // to double
        Vec3 oc_d  = Vec3( (double)oc.x,     (double)oc.y,     (double)oc.z );
        Vec3 dir_d = Vec3( (double)dir.x,    (double)dir.y,    (double)dir.z );
        Vec3 n_d   = Vec3( (double)n_obj_f.x,(double)n_obj_f.y,(double)n_obj_f.z );

        Ray objRay;
        objRay.origin = oc_d;
        objRay.dir    = dir_d;

        return commitFromObjectSpace(
            worldRay,
            objRay,
            (double)t_obj_f,
            n_d,
            hit,
            tMin,
            tMax
        );
    };

    // try near, then far
    if (try_root(t1)) return true;
    if (try_root(t2)) return true;
    return false;
}
