/*
#include "plane.h"
#include <limits>
#include <iostream>
#include <cmath>

Plane::Plane(const std::string& name,
             double c1x, double c1y, double c1z,
             double c2x, double c2y, double c2z,
             double c3x, double c3y, double c3z,
             double c4x, double c4y, double c4z)
    : m_name(name),
      c1(c1x, c1y, c1z),
      c2(c2x, c2y, c2z),
      c3(c3x, c3y, c3z),
      c4(c4x, c4y, c4z)
{ }

void Plane::print_info() const {
    std::cout << "Plane: " << m_name << "\n"
              << "  c1: (" << c1.x << ", " << c1.y << ", " << c1.z << ")\n"
              << "  c2: (" << c2.x << ", " << c2.y << ", " << c2.z << ")\n"
              << "  c3: (" << c3.x << ", " << c3.y << ", " << c3.z << ")\n"
              << "  c4: (" << c4.x << ", " << c4.y << ", " << c4.z << ")\n\n";
}

// Möller–Trumbore for double
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
        return false; // parallel

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
    // bring the ray into *our* object space, so corners c1..c4 make sense
    Ray r = xform_ray(W2L, worldRay);

    bool anyHit = false;
    double bestT = std::numeric_limits<double>::infinity();
    Vec3 bestN;

    // we split quad (c1,c2,c4) and (c1,c4,c3)
    double t;
    Vec3 n;

    if (intersectTriangle(r, c1, c2, c4, t, n)) {
        // like Sphere we now need to "commit" in world space,
        // but we only want it if it's the closest so far
        if (t >= tMin && t <= tMax && t < bestT) {
            anyHit = true;
            bestT = t;
            bestN = n;
        }
    }
    if (intersectTriangle(r, c1, c4, c3, t, n)) {
        if (t >= tMin && t <= tMax && t < bestT) {
            anyHit = true;
            bestT = t;
            bestN = n;
        }
    }

    if (!anyHit)
        return false;

    // finalize hit through the base class helper so normals/points go to world
    return commitFromObjectSpace(
        worldRay,
        r,              // the object-space ray we used
        bestT,          // object-space t
        bestN,          // object-space normal
        hit,
        tMin,
        tMax
    );
}

AABB Plane::objectBounds() const
{
    // just box the 4 corners in object space
    AABB box;
    box.expand(c1);
    box.expand(c2);
    box.expand(c3);
    box.expand(c4);
    return box;
}
*/

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

    return commitFromObjectSpace(
        worldRay,
        r,
        bestT,
        bestN,
        hit,
        tMin,
        tMax
    );
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

