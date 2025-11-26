#include "plane.h"
#include <limits>
#include <iostream>
#include <cmath>

Plane::Plane(const std::string& name,
             double c0x, double c0y, double c0z,
             double c1x, double c1y, double c1z,
             double c2x, double c2y, double c2z,
             double c3x, double c3y, double c3z)
    : m_name(name),
      c0(c0x, c0y, c0z),
      c1(c1x, c1y, c1z),
      c2(c2x, c2y, c2z),
      c3(c3x, c3y, c3z)
{ }

void Plane::print_info() const {
    std::cout << "Plane: " << m_name << "\n"
              << "  c0: (" << c0.x << ", " << c0.y << ", " << c0.z << ")\n"
              << "  c1: (" << c1.x << ", " << c1.y << ", " << c1.z << ")\n"
              << "  c2: (" << c2.x << ", " << c2.y << ", " << c2.z << ")\n"
              << "  c3: (" << c3.x << ", " << c3.y << ", " << c3.z << ")\n\n";
}

static bool intersectTriangle(const Ray& r,
                              const Vec3& a,
                              const Vec3& b,
                              const Vec3& c,
                              double& t_out,
                              Vec3& n_out)
{
    const double EPS = 1e-9;

    Vec3 edge1 = b - a;
    Vec3 edge2 = c - a;
    Vec3 pvec  = cross(r.dir, edge2);
    double det = dot(edge1, pvec);

    if (std::abs(det) < EPS)
        return false;

    double invDet = 1.0 / det;
    Vec3 tvec = r.origin - a;
    double u = dot(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0)
        return false;

    Vec3 qvec = cross(tvec, edge1);
    double v = dot(r.dir, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0)
        return false;

    double t = dot(edge2, qvec) * invDet;
    if (t < EPS)
        return false;

    t_out = t;
    n_out = normalize(cross(edge1, edge2));
    return true;
}

bool Plane::intersect(const Ray& worldRay,
                      double tMin,
                      double tMax,
                      Hit& hit) const
{
    // ray -> object space (so our c0..c3 make sense)
    Ray r = xform_ray(W2L, worldRay);

    bool anyHit = false;
    double bestT = std::numeric_limits<double>::infinity();
    Vec3 bestN;

    double t;
    Vec3  n;

    // ✅ correct split for quad c0,c1,c2,c3:
    // triangle 1: (c0, c1, c2)
    if (intersectTriangle(r, c0, c1, c2, t, n)) {
        if (t >= tMin && t <= tMax && t < bestT) {
            anyHit = true;
            bestT = t;
            bestN = n;
        }
    }
    // triangle 2: (c0, c2, c3)
    if (intersectTriangle(r, c0, c2, c3, t, n)) {
        if (t >= tMin && t <= tMax && t < bestT) {
            anyHit = true;
            bestT = t;
            bestN = n;
        }
    }

    if (!anyHit)
        return false;

    if (!commitFromObjectSpace(
            worldRay,
            r,
            bestT,
            bestN,
            hit,
            tMin,
            tMax
        ))
    {
        return false;
    }

    // -------------------
    // NEW: compute UVs
    // -------------------
    // We now have hit.p in world space.
    // Treat the plane as a tiled floor in XZ.
    // -------------------------------------------
    // Proper UV for the quad (c0,c1,c2,c3)
    // -------------------------------------------

    // hit point in object space:
    Vec3 p_obj = r.origin + r.dir * bestT;

    // Build in-plane axes from the quad corners:
    Vec3 U = c1 - c0;     // one edge
    Vec3 V = c3 - c0;     // the other edge (assuming quad is c0,c1,c2,c3 in order)

    // Lengths:
    double lenU = std::sqrt(dot(U, U));
    double lenV = std::sqrt(dot(V, V));

    if (lenU > 0.0 && lenV > 0.0) {
        // Normalised axes:
        Vec3 Udir = U / lenU;
        Vec3 Vdir = V / lenV;

        // Vector from c0 to hit point:
        Vec3 rel = p_obj - c0;

        // Project onto plane axes:
        double u_local = dot(rel, Udir);
        double v_local = dot(rel, Vdir);

        // Normalise to [0,1]
        double u = u_local / lenU;
        double v = v_local / lenV;

        // Optional tiling (repeat 3×3 etc.)
        double tile = 1.0;
        u *= tile;
        v *= tile;

        // Wrap to [0,1)
        u = u - floor(u);
        v = v - floor(v);

        hit.u = u;
        hit.v = v;

        return true;
    }

}

AABB Plane::objectBounds() const
{
    AABB box;
    box.expand(c0);
    box.expand(c1);
    box.expand(c2);
    box.expand(c3);
    return box;
}

