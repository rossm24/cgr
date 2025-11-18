#include "sphere.h"
#include "mat4.h"
#include <iostream>
#include <cmath>

// ----- small helpers: same convention as in main.cpp -----
// Column-vector convention, row-major storage.

static Mat4 eulerXYZ_to_R(double rx, double ry, double rz) {
    Mat4 Rx = Mat4::rotateX(rx);
    Mat4 Ry = Mat4::rotateY(ry);
    Mat4 Rz = Mat4::rotateZ(rz);
    // Blender default Euler: XYZ -> R = Rz * Ry * Rx
    return Mat4::multiply(Mat4::multiply(Rz, Ry), Rx);
}

static Mat4 composeTRS(const Vec3& t, const Vec3& euler, const Vec3& s) {
    Mat4 T = Mat4::translate(t);
    Mat4 R = eulerXYZ_to_R(euler.x, euler.y, euler.z);
    Mat4 S = Mat4::scale(s);
    // Final transform: T * R * S
    return Mat4::multiply(T, Mat4::multiply(R, S));
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
{
    // Build the same kind of TRS matrix we use for cubes.
    // NOTE: ASCII coordinates are already in renderer space,
    // so no B2R() here.
    Vec3 t   = Vec3{ px, py, pz };
    Vec3 eul = Vec3{ rx, ry, rz };
    Vec3 s   = Vec3{ sx, sy, sz };   // radii as per-axis scale

    Mat4 M = composeTRS(t, eul, s);
    setTransform(M);  // Shape base: sets L2W and W2L using Mat4::invert
}

void Sphere::print_info() const {
    std::cout << "Sphere " << m_name
              << " pos=("   << m_px << "," << m_py << "," << m_pz << ")"
              << " scale=(" << m_sx << "," << m_sy << "," << m_sz << ")"
              << " rot=("   << m_rx << "," << m_ry << "," << m_rz << ")\n";
}

// ----- intersection -----
//
// Object space: unit sphere at origin, radius 1.
// World TRS (including non-uniform scale + rotation) is encoded in L2W/W2L,
// just like for cubes.

bool Sphere::intersect(const Ray& worldRay,
                       double tMin,
                       double tMax,
                       Hit& hit) const
{
    // Bring ray into object space.
    Ray r = xform_ray(W2L, worldRay);

    // Quadratic for |r.origin + t*r.dir|^2 = 1
    const Vec3& o = r.origin;
    const Vec3& d = r.dir;

    double a = dot(d, d);
    double b = 2.0 * dot(o, d);
    double c = dot(o, o) - 1.0;

    double disc = b*b - 4.0*a*c;
    if (disc < 0.0) return false;

    double sqrtD = std::sqrt(disc);
    double t1 = (-b - sqrtD) / (2.0 * a);
    double t2 = (-b + sqrtD) / (2.0 * a);
    const double EPS = 1e-6;

    auto try_root = [&](double t_obj) -> bool {
        if (t_obj <= EPS) return false;

        // Hit point in object space (on the unit sphere)
        Vec3 p_obj = o + d * t_obj;

        // Object-space normal
        Vec3 n_obj = normalize(p_obj);

        // Let Shape::commitFromObjectSpace handle:
        //   * checking tMin/tMax
        //   * closest-hit update
        //   * transforming position & normal to world space
        if (!commitFromObjectSpace(worldRay, r, t_obj, n_obj, hit, tMin, tMax))
            return false;

        // Simple spherical UV mapping using n_obj (unit vector)
        const double PI = 3.14159265358979323846;
        double u = 0.5 + std::atan2(n_obj.z, n_obj.x) / (2.0 * PI);
        double v = 0.5 - std::asin(n_obj.y) / PI;

        // Clamp and store
        u = std::clamp(u, 0.0, 1.0);
        v = std::clamp(v, 0.0, 1.0);
        hit.u = u;
        hit.v = v;

        return true;
    };

    if (try_root(t1)) return true;
    if (try_root(t2)) return true;
    return false;
}

