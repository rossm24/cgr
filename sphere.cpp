/*
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

// --- tiny mat helpers (keep next to your existing f3 helpers) ---
static inline Sphere::m3 mul(const Sphere::m3& A, const Sphere::m3& B){
    return {
        A.a11*B.a11 + A.a12*B.a21 + A.a13*B.a31,
        A.a11*B.a12 + A.a12*B.a22 + A.a13*B.a32,
        A.a11*B.a13 + A.a12*B.a23 + A.a13*B.a33,

        A.a21*B.a11 + A.a22*B.a21 + A.a23*B.a31,
        A.a21*B.a12 + A.a22*B.a22 + A.a23*B.a32,
        A.a21*B.a13 + A.a22*B.a23 + A.a23*B.a33,

        A.a31*B.a11 + A.a32*B.a21 + A.a33*B.a31,
        A.a31*B.a12 + A.a32*B.a22 + A.a33*B.a32,
        A.a31*B.a13 + A.a32*B.a23 + A.a33*B.a33
    };
}
static inline Sphere::f3 mul(const Sphere::m3& A, const Sphere::f3& v){
    return { A.a11*v.x + A.a12*v.y + A.a13*v.z,
             A.a21*v.x + A.a22*v.y + A.a23*v.z,
             A.a31*v.x + A.a32*v.y + A.a33*v.z };
}
static inline Sphere::m3 transpose(const Sphere::m3& A){
    return { A.a11, A.a21, A.a31,
             A.a12, A.a22, A.a32,
             A.a13, A.a23, A.a33 };
}
static inline Sphere::m3 rotX(float x){
    float c=std::cos(x), s=std::sin(x);
    return {1,0,0,  0,c,-s,  0,s,c};
}
static inline Sphere::m3 rotY(float y){
    float c=std::cos(y), s=std::sin(y);
    return {c,0,s,  0,1,0,  -s,0,c};
}
static inline Sphere::m3 rotZ(float z){
    float c=std::cos(z), s=std::sin(z);
    return {c,-s,0,  s,c,0,  0,0,1};
}

// ---------------- ctor ----------------
Sphere::Sphere(const std::string& name,
               float px, float py, float pz,
               float sx, float sy, float sz,
               float rx, float ry, float rz,   // radians
               float radius)                   // can be 1 if unused
: m_name(name),
  m_px(px), m_py(py), m_pz(pz)
{
    // effective per-axis scale (choose one scheme!)
    m_sx = sx * radius;
    m_sy = sy * radius;
    m_sz = sz * radius;

    // build rotation (column-major, right-handed)
    Mat3 Rz = rotZ(rz), Ry = rotY(ry), Rx = rotX(rx);
    m_R  = Rz * Ry * Rx;        // or your engine’s Euler order
    m_RT = transpose(m_R);
}

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



/*
Sphere::Sphere(const std::string& name,
               float px, float py, float pz,
               float sx, float sy, float sz)
    : m_name(name),
      m_px(px), m_py(py), m_pz(pz),
      m_sx(sx), m_sy(sy), m_sz(sz)
{ }



// sphere.cpp
#include "sphere.h"
#include <cmath>
#include <iostream>

// ---------------- tiny float3 + helpers ----------------

struct f3 {
    float x, y, z;
    f3() : x(0), y(0), z(0) {}
    f3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

    f3 operator+(const f3& b) const { return {x + b.x, y + b.y, z + b.z}; }
    f3 operator-(const f3& b) const { return {x - b.x, y - b.y, z - b.z}; }
    f3 operator*(float s)    const { return {x * s,   y * s,   z * s  }; }
};

static inline f3 cmul(const f3& a, const f3& b) { return { a.x*b.x, a.y*b.y, a.z*b.z }; }
static inline f3 cdiv(const f3& a, const f3& b) { return { a.x/b.x, a.y/b.y, a.z/b.z }; }

static inline float fdot(const f3& a, const f3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline float flen(const f3& v) { return std::sqrt(fdot(v,v)); }
static inline f3 fnorm(const f3& v) {
    float L = flen(v);
    return (L > 0.0f) ? f3(v.x/L, v.y/L, v.z/L) : f3(0,0,0);
}






// ---------------- ctor & info ----------------

Sphere::Sphere(const std::string& name,
               float px, float py, float pz,
               float sx, float sy, float sz)
    : m_name(name),
      m_px(px), m_py(py), m_pz(pz),
      m_sx(sx), m_sy(sy), m_sz(sz)
{}

void Sphere::print_info() const {
    std::cout << "Sphere: " << m_name << "\n"
              << "  pos:   (" << m_px << ", " << m_py << ", " << m_pz << ")\n"
              << "  scale: (" << m_sx << ", " << m_sy << ", " << m_sz << ")\n";
}


// ---------------- intersection ----------------
//
// World ray -> object (unit sphere) space:
//   p_obj = S^{-1} R^T (p_world - pos)
//   d_obj = S^{-1} R^T d_world
//
// Then intersect unit sphere at origin, and transform normal back.

bool Sphere::intersect(const Ray& worldRay,
                       double tMin,
                       double tMax,
                       Hit& hit) const
{
    // world → float
    f3 rof((float)worldRay.origin.x,
           (float)worldRay.origin.y,
           (float)worldRay.origin.z);
    f3 rdf((float)worldRay.dir.x,
           (float)worldRay.dir.y,
           (float)worldRay.dir.z);

    f3 posf(m_px, m_py, m_pz);
    f3 scalef(m_sx, m_sy, m_sz);

    // ray in local space of unit sphere
    f3 oc  = cdiv(rof - posf, scalef);
    f3 dir =  cdiv(rdf, scalef);

    float a = fdot(dir, dir);
    float b = 2.0f * fdot(oc, dir);
    float c = fdot(oc, oc) - 1.0f;

    float disc = b*b - 4.0f*a*c;
    if (disc < 0.0f) return false;

    float sqrtD = std::sqrt(disc);
    float t1 = (-b - sqrtD) / (2.0f * a);
    float t2 = (-b + sqrtD) / (2.0f * a);

    const float EPS = 1e-7f;

    auto try_root = [&](float t_obj_f) -> bool {
        if (t_obj_f <= EPS)
            return false;

        f3 p_obj = oc + dir * t_obj_f;

        f3 scale_sq = cmul(scalef, scalef);
        f3 n_obj_f( p_obj.x / scale_sq.x,
                    p_obj.y / scale_sq.y,
                    p_obj.z / scale_sq.z );
        n_obj_f = fnorm(n_obj_f);

        // float → double
        Vec3 oc_d  = Vec3(oc.x,     oc.y,     oc.z);
        Vec3 dir_d = Vec3(dir.x,    dir.y,    dir.z);
        Vec3 n_d   = Vec3(n_obj_f.x,n_obj_f.y,n_obj_f.z);

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

    if (try_root(t1)) return true;
    if (try_root(t2)) return true;
    return false;
}

AABB Sphere::objectBounds() const {
    Vec3 half(std::fabs(m_sx), std::fabs(m_sy), std::fabs(m_sz));
    Vec3 c   (m_px, m_py, m_pz);
    return AABB(c - half, c + half);
}
*/

#include "sphere.h"
#include <iostream>
#include <math.h>
#include <cmath>

// ----- tiny float vec -----
struct f3 {
    float x,y,z;
    f3() : x(0),y(0),z(0) {}
    f3(float X,float Y,float Z) : x(X),y(Y),z(Z) {}

    f3 operator+(const f3& b) const { return {x+b.x,y+b.y,z+b.z}; }
    f3 operator-(const f3& b) const { return {x-b.x,y-b.y,z-b.z}; }
    f3 operator*(float s)    const { return {x*s,y*s,z*s}; }
};

static inline f3 cmul(const f3& a,const f3& b){ return {a.x*b.x,a.y*b.y,a.z*b.z}; }
static inline f3 cdiv(const f3& a,const f3& b){ return {a.x/b.x,a.y/b.y,a.z/b.z}; }

static inline float fdot(const f3& a,const f3& b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline float flen(const f3& v){ return std::sqrt(fdot(v,v)); }
static inline f3 fnorm(const f3& v){
    float L = flen(v);
    return (L>0.0f)? f3(v.x/L,v.y/L,v.z/L) : f3(0,0,0);
}

// ----- tiny 3x3 matrix with Blender’s Euler order Rz*Ry*Rx -----
struct m3 {
    float a11,a12,a13,
          a21,a22,a23,
          a31,a32,a33;
};

static inline m3 rotX(float x){
    float c=std::cos(x), s=std::sin(x);
    return {1,0,0,  0,c,-s,  0,s,c};
}
static inline m3 rotY(float y){
    float c=std::cos(y), s=std::sin(y);
    return {c,0,y? s:-s,  0,1,0,  -s,0,c}; // but easier to write properly:
}
static inline m3 rotY_fixed(float y){
    float c=std::cos(y), s=std::sin(y);
    return { c,0, s,
             0,1, 0,
            -s,0, c };
}
static inline m3 rotZ(float z){
    float c=std::cos(z), s=std::sin(z);
    return { c,-s,0,
             s, c,0,
             0, 0,1 };
}
static inline m3 m3_mul(const m3& A,const m3& B){
    m3 C;
    C.a11 = A.a11*B.a11 + A.a12*B.a21 + A.a13*B.a31;
    C.a12 = A.a11*B.a12 + A.a12*B.a22 + A.a13*B.a32;
    C.a13 = A.a11*B.a13 + A.a12*B.a23 + A.a13*B.a33;

    C.a21 = A.a21*B.a11 + A.a22*B.a21 + A.a23*B.a31;
    C.a22 = A.a21*B.a12 + A.a22*B.a22 + A.a23*B.a32;
    C.a23 = A.a21*B.a13 + A.a22*B.a23 + A.a23*B.a33;

    C.a31 = A.a31*B.a11 + A.a32*B.a21 + A.a33*B.a31;
    C.a32 = A.a31*B.a12 + A.a32*B.a22 + A.a33*B.a32;
    C.a33 = A.a31*B.a13 + A.a32*B.a23 + A.a33*B.a33;
    return C;
}
static inline f3 m3_mul(const m3& A,const f3& v){
    return {
        A.a11*v.x + A.a12*v.y + A.a13*v.z,
        A.a21*v.x + A.a22*v.y + A.a23*v.z,
        A.a31*v.x + A.a32*v.y + A.a33*v.z
    };
}
static inline m3 m3_transpose(const m3& A){
    return {
        A.a11,A.a21,A.a31,
        A.a12,A.a22,A.a32,
        A.a13,A.a23,A.a33
    };
}

// ----- ctor / info -----

Sphere::Sphere(const std::string& name,
               float px,float py,float pz,
               float sx,float sy,float sz,
               float rx,float ry,float rz)
    : m_name(name),
      m_px(px), m_py(py), m_pz(pz),
      m_sx(sx), m_sy(sy), m_sz(sz),
      m_rx(rx), m_ry(ry), m_rz(rz)
{}

void Sphere::print_info() const {
    std::cout << "Sphere " << m_name
              << " pos=(" << m_px << "," << m_py << "," << m_pz << ")"
              << " scale=(" << m_sx << "," << m_sy << "," << m_sz << ")"
              << " rot=(" << m_rx << "," << m_ry << "," << m_rz << ")\n";
}

// safe-ish world-space bounds: use max axis as radius
AABB Sphere::objectBounds() const {
    double r = std::max({ std::fabs(m_sx),
                          std::fabs(m_sy),
                          std::fabs(m_sz) });
    Vec3 c(m_px,m_py,m_pz);
    Vec3 half(r,r,r);
    return AABB(c - half, c + half);
}

// ----- intersection with rotated ellipsoid -----

bool Sphere::intersect(const Ray& worldRay,
                       double tMin,
                       double tMax,
                       Hit& hit) const
{
    // world → float
    f3 rof((float)worldRay.origin.x,
           (float)worldRay.origin.y,
           (float)worldRay.origin.z);
    f3 rdf((float)worldRay.dir.x,
           (float)worldRay.dir.y,
           (float)worldRay.dir.z);

    f3 posf(m_px,m_py,m_pz);
    f3 invS(1.0f/m_sx, 1.0f/m_sy, 1.0f/m_sz);

    // rotation matrix: match Python euler_xyz_to_matrix (Rz * Ry * Rx)
    m3 Rx = rotX(m_rx);
    m3 Ry = rotY_fixed(m_ry);
    m3 Rz = rotZ(m_rz);
    m3 R  = m3_mul(Rz, m3_mul(Ry, Rx));
    m3 RT = m3_transpose(R);

    // world → object (translate, inverse-rotate, inverse-scale)
    f3 ro_local = m3_mul(RT, rof - posf);
    ro_local = cmul(ro_local, invS);

    f3 rd_local = m3_mul(RT, rdf);
    rd_local = cmul(rd_local, invS);

    float a = fdot(rd_local, rd_local);
    float b = 2.0f * fdot(ro_local, rd_local);
    float c = fdot(ro_local, ro_local) - 1.0f;

    float disc = b*b - 4.0f*a*c;
    if (disc < 0.0f) return false;

    float sqrtD = std::sqrt(disc);
    float t1 = (-b - sqrtD) / (2.0f*a);
    float t2 = (-b + sqrtD) / (2.0f*a);
    const float EPS = 1e-6f;

    auto try_root = [&](float t_obj_f)->bool {
        if (t_obj_f <= EPS) return false;

        // hit point in object space (unit sphere)
        f3 p_obj = ro_local + rd_local * t_obj_f;

        // normal in object space
        f3 n_obj = fnorm(p_obj);

        // normal back to world: n_w ∝ R * (n_obj / S)
        f3 n_scaled(n_obj.x * invS.x,
                    n_obj.y * invS.y,
                    n_obj.z * invS.z);
        f3 n_wf = fnorm( m3_mul(R, n_scaled) );

        // build object-space ray for commitFromObjectSpace
        Ray objRay;
        objRay.origin = Vec3(ro_local.x, ro_local.y, ro_local.z);
        objRay.dir    = Vec3(rd_local.x, rd_local.y, rd_local.z);

        Vec3 n_w(n_wf.x, n_wf.y, n_wf.z);

        // ---- compute UVs in object space (unit sphere) ----
        // use the direction on the sphere for spherical mapping
        f3 n_for_uv = fnorm(p_obj);
        const double PI = 3.14159265358979323846;
        double u = 0.5 + std::atan2((double)n_for_uv.z,
                                    (double)n_for_uv.x) / (2.0 * PI);
        double v = 0.5 - std::asin((double)n_for_uv.y) / PI;            

        // Let commitFromObjectSpace decide if this hit is actually kept
        if (commitFromObjectSpace(
                worldRay,
                objRay,
                (double)t_obj_f,
                n_w,   // you were already passing world-space normal here
                hit,
                tMin,
                tMax
        )) {
            // Only now do we stamp UVs into the winning Hit
            hit.u = u;
            hit.v = v;
            return true;
        }

        return false;
    };

    if (try_root(t1)) return true;
    if (try_root(t2)) return true;
    return false;
}


