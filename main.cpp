// main.cpp — render all shapes declared in scene.txt (CAM, LGT, CUB, SPH, PLN)

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <random>

#include "camera.h"     // Vec3, Ray, Camera
#include "image_ppm.h"
#include "mat4.h"
#include "shape.h"
#include "sphere.h"
#include "cube.h"
#include "plane.h"
#include "bvh.h"
#include "hit.h"
#include "material.h"
#include "material_db.h"
#include "shade.h"
#include "scene_accel.h"
#include "texture_manager.h"

// Blender -> Renderer coordinates
// Blender: X right, Y forward, Z up
// Renderer: X right, Y up, Z forward
static inline Vec3 B2R(const Vec3& pB)
{
    // (x, y, z) -> (x, z, -y)
    //return Vec3{ pB.x, pB.z, -pB.y };
    return pB;
}


// ---------------- tiny parsing helpers ----------------
static bool extractVec3(const std::string& line, const std::string& key, Vec3& out){
    std::string k = key + "=(";
    size_t p = line.find(k);
    if (p == std::string::npos) return false;
    p += k.size();
    std::istringstream iss(line.substr(p));
    double a,b,c;
    if (!(iss >> a >> b >> c)) return false;
    out = {a,b,c};
    return true;
}

static bool extractNumber(const std::string& line, const std::string& key, double& out){
    std::string k = key + "=";
    size_t p = line.find(k);
    if (p == std::string::npos) return false;
    p += k.size();
    std::istringstream iss(line.substr(p));
    if (!(iss >> out)) return false;
    return true;
}

// ---------------- transforms (match Blender exporter: R = Rz * Ry * Rx) ----------------
static Mat4 eulerXYZ_to_R(double rx, double ry, double rz){
    Mat4 Rx = Mat4::rotateX(rx);
    Mat4 Ry = Mat4::rotateY(ry);
    Mat4 Rz = Mat4::rotateZ(rz);
    return Mat4::multiply(Mat4::multiply(Rz, Ry), Rx); // Rz * Ry * Rx
}
static Mat4 composeTRS(const Vec3& t, const Vec3& euler, const Vec3& s){
    Mat4 T = Mat4::translate(t);
    Mat4 R = eulerXYZ_to_R(euler.x, euler.y, euler.z);
    Mat4 S = Mat4::scale(s);
    return Mat4::multiply(T, Mat4::multiply(R, S));
}

// ---------------- shading ----------------
static inline uint8_t clamp8(double v){
    int i = (int)std::round(std::clamp(v, 0.0, 255.0));
    return (uint8_t)i;
}

static Vec3 shade_diffuse(const Hit& hit, const Vec3& lightPos, double lightIntensity, const Vec3& albedo){
    Vec3 Lvec = lightPos - hit.p;
    double r2  = std::max(1e-8, dot(Lvec, Lvec));
    Vec3 Ldir  = normalize(Lvec);
    double ndl = std::max(0.0, dot(hit.n, Ldir));
    double scale = (lightIntensity / r2) * ndl;

    Vec3 c = { albedo.x * scale, albedo.y * scale, albedo.z * scale };
    // tiny ambient
    c.x += 0.02 * albedo.x;
    c.y += 0.02 * albedo.y;
    c.z += 0.02 * albedo.z;
    return c;
}

struct RNG {
    std::mt19937 gen;
    std::uniform_real_distribution<double> U{0.0, 1.0};
    RNG(uint32_t seed=1337) : gen(seed) {}
    inline double next() { return U(gen); }
};

static bool extractString(const std::string& line,
                          const std::string& key,
                          std::string& out)
{
    std::string k = key + "=";
    size_t p = line.find(k);
    if (p == std::string::npos) return false;
    p += k.size();

    // quoted string?
    if (line[p] == '"') {
        size_t q = line.find('"', p+1);
        if (q == std::string::npos) return false;
        out = line.substr(p+1, q - (p+1));
        return true;
    }

    // unquoted fallback
    size_t q = p;
    while (q < line.size() && !isspace(line[q])) q++;
    out = line.substr(p, q - p);
    return true;
}







int main(){
    try{
        const std::string scenePath = "../ASCII/scenetest33.txt";

        // ---------- 1) Load Camera ----------
        Camera cam;
        cam.loadFromSceneTxt(scenePath);
        std::cout << "Loaded camera: " << cam.name << " ("
                  << cam.film_w_px << "x" << cam.film_h_px << ")\n";

        // ---------- 2) Parse scene objects ----------
        // struct Light { Vec3 pos{}; double intensity=0.0; };
        std::vector<Light> lights;

        struct CubeAscii  { Vec3 center{}; Vec3 euler{}; double edge=1.0; Vec3 color; std::string texName;};
        struct SphereAscii{ std::string name; Vec3 center; Vec3 euler; Vec3 scale; double radius; Vec3 color; std::string texName; };
        struct PlaneAscii {
            Vec3 c0{}, c1{}, c2{}, c3{};
            Vec3 color{0.8,0.8,0.8};
            std::string texName;
        };

        std::vector<CubeAscii>   cubes;
        std::vector<SphereAscii> spheres;
        PlaneAscii pln{};
        bool havePlane = false;

        std::ifstream sf(scenePath);
        if(!sf) throw std::runtime_error("Cannot open scene.txt");
        std::string line;
        while (std::getline(sf, line)) {
            if (line.rfind("LGT ", 0) == 0) {
                Light L{};
                if (!extractVec3(line, "loc", L.pos))
                    throw std::runtime_error("LGT: loc missing");
                if (!extractNumber(line, "intensity", L.intensity)) {
                    double power=0.0;
                    if (!extractNumber(line, "power", power))
                        throw std::runtime_error("LGT: intensity/power missing");
                    L.intensity = power * (1.0/(4.0*3.14159265358979323846));
                }
                L.color = {1,1,1}; // white
                lights.push_back(L);
            }
            else if (line.rfind("CUB ", 0) == 0) {
                CubeAscii C{};
                if (!extractVec3(line, "trans", C.center))
                    throw std::runtime_error("CUB: trans missing");
                extractVec3(line, "rot", C.euler);
                extractVec3(line, "color", C.color);
                double s1=1.0;
                if (!extractNumber(line, "scale1d", s1))
                    throw std::runtime_error("CUB: scale1d missing");
                C.edge = s1;

                extractString(line, "tex", C.texName);

                cubes.push_back(C);
            }
            else if (line.rfind("SPH ", 0) == 0) {
                SphereAscii S{};
                if (!extractVec3(line, "loc", S.center))
                    throw std::runtime_error("SPH: loc missing");
                extractString(line, "name", S.name);
                extractVec3(line, "rot",   S.euler);
                extractVec3(line, "scale", S.scale);
                extractVec3(line, "color", S.color);
                if (!extractNumber(line, "radius", S.radius))
                    throw std::runtime_error("SPH: radius missing");

                extractString(line, "tex", S.texName);

                spheres.push_back(S);
            }
            else if (line.rfind("PLN ", 0) == 0) {
            if (!extractVec3(line, "c0", pln.c0)) throw std::runtime_error("PLN: c0 missing");
            if (!extractVec3(line, "c1", pln.c1)) throw std::runtime_error("PLN: c1 missing");
            if (!extractVec3(line, "c2", pln.c2)) throw std::runtime_error("PLN: c2 missing");
            if (!extractVec3(line, "c3", pln.c3)) throw std::runtime_error("PLN: c3 missing");
            extractVec3(line, "color", pln.color);

            extractString(line, "tex", pln.texName);    

            havePlane = true;
            }

        }

        if (lights.empty()) {
            std::cerr << "[warn] No lights; using a default light.\n";
            lights.push_back(Light{ /*pos*/{4,1,6}, /*color*/{1,1,1}, /*intensity*/80.0 });
        }

        

        // ============================================================
        // DEBUG: check whether any Blender spheres physically overlap
        // ============================================================
        std::cout << "=== Checking for sphere-sphere overlaps ===\n";

        for (size_t i = 0; i < spheres.size(); ++i) {
            for (size_t j = i + 1; j < spheres.size(); ++j) {

                const auto& A = spheres[i];
                const auto& B = spheres[j];

                // distance between centres
                Vec3 d = A.center - B.center;
                double dist = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);

                // crude “radius” = max axis (consistent with your C++ ellipsoid)
                double rA = std::max({ std::fabs(A.scale.x),
                                    std::fabs(A.scale.y),
                                    std::fabs(A.scale.z) });

                double rB = std::max({ std::fabs(B.scale.x),
                                    std::fabs(B.scale.y),
                                    std::fabs(B.scale.z) });

                if (dist < rA + rB) {
                    std::cout << "Overlap: SPH[" << i << "] and SPH[" << j << "] "
                            << "dist=" << dist
                            << " < rA+rB=" << (rA + rB) << "\n";
                }
            }
        }

        std::cout << "=== End of overlap test ===\n";

        TextureManager texMgr;

        

        // ---------- 3) Instantiate shapes ----------
        std::vector<Shape*> shapes;
        std::vector<std::unique_ptr<Shape>> owned;

        // Cubes
        for (size_t i = 0; i < cubes.size(); ++i) {
            auto cb = std::make_unique<Cube>();
            cb->id = int(100 + i);

            // ASCII centre (already in renderer coords)
            Vec3 centerR = B2R(cubes[i].center);   // currently just returns center

            // Euler from ASCII (in radians, XYZ order)
            Vec3 euler = cubes[i].euler;

            // Uniform scale from scale1d
            Vec3 scale = { cubes[i].edge, cubes[i].edge, cubes[i].edge };

            // T * R * S (this is correct with your composeTRS + new Mat4)
            Mat4 C = composeTRS(centerR, euler, scale);

            // --- DEBUG: world-space position of the cube's local origin ---
            Vec3 worldOrigin = Mat4::mul_point(C, Vec3{0,0,0});
            std::cout << "Cube " << i
                    << " center=(" << cubes[i].center.x << ", "
                                    << cubes[i].center.y << ", "
                                    << cubes[i].center.z << ")"
                    << " edge=" << cubes[i].edge
                    << " worldOrigin=(" << worldOrigin.x << ", "
                                        << worldOrigin.y << ", "
                                        << worldOrigin.z << ")\n";

            // ---- NEW: attach texture if present ----
            // NEW: texture hookup
            if (!cubes[i].texName.empty()) {
                cb->texName = cubes[i].texName;
                std::string fullPath = "../Textures/" + cb->texName;
                cb->texture = texMgr.get(fullPath);
            }                          


            cb->setTransform(C);
            shapes.push_back(cb.get());
            owned.push_back(std::move(cb));

            //if (i==5) break;
        }

        // --- Spheres: unit-sphere geometry driven entirely by the transform ---
        for (size_t i = 0; i < spheres.size(); ++i) {
            const auto& S = spheres[i];

            auto sp = std::make_unique<Sphere>(
                S.name,
                (float)S.center.x, (float)S.center.y, (float)S.center.z,
                (float)S.scale.x,  (float)S.scale.y,  (float)S.scale.z,  
                (float)S.euler.x,  (float)S.euler.y,  (float)S.euler.z
            );
            sp->id = int(200 + i);

            // NEW: texture hookup
            if (!S.texName.empty()) {
                sp->texName = S.texName;
                std::string fullPath = "../Textures/" + sp->texName;
                sp->texture = texMgr.get(fullPath);
            }

            shapes.push_back(sp.get());
            owned.push_back(std::move(sp));

            std::cerr << "[SPH] " << S.name << " center=("
                << S.center.x << ", " << S.center.y << ", " << S.center.z << ") "
                << "scale=("
                << S.scale.x  << ", " << S.scale.y  << ", " << S.scale.z  << ")\n";
        }



        // Plane — create with 4 corners like your friend's
        if (havePlane) {
            auto pl = std::make_unique<Plane>(
                "plane",
                pln.c0.x, pln.c0.y, pln.c0.z,
                pln.c1.x, pln.c1.y, pln.c1.z,
                pln.c2.x, pln.c2.y, pln.c2.z,
                pln.c3.x, pln.c3.y, pln.c3.z
            );
            pl->id = 50;

            if (!pln.texName.empty()) {
                pl->texName = pln.texName;
                pl->texture = texMgr.get("../Textures/" + pl->texName);
            }

            shapes.push_back(pl.get());
            owned.push_back(std::move(pl));
        }

        

        MaterialDB mats;

        // Plane (id 50)
        if (havePlane) {
            mats.by_id[50] = Material{
                .kd = pln.color,
                .ks = {0.1,0.1,0.1},
                .shininess = 32,
                .reflectivity = 0.05
            };
        }

        // Cubes
        for (size_t i = 0; i < cubes.size(); ++i) {
            int id = int(100 + i);
            mats.by_id[id] = Material{
                .kd = cubes[i].color,
                .ks = {0.2,0.2,0.2},
                .shininess = 64,
                .reflectivity = 0.0
            };
        }

        // Spheres
        for (size_t i = 0; i < spheres.size(); ++i) {
            int id = int(200 + i);
            mats.by_id[id] = Material{
                .kd = spheres[i].color,
                .ks = {0.3,0.3,0.3},
                .shininess = 128,
                .reflectivity = 0.4
            };
        }

        



        // ---------- 4) Build BVH ----------
        std::vector<Shape*> finiteShapes, infiniteShapes;
        for(auto* s : shapes){
            if(s->isFinite()) finiteShapes.push_back(s);
            else              infiniteShapes.push_back(s);
        }

        BVH bvh;
        auto tBuild0 = std::chrono::high_resolution_clock::now();
        bvh.build(finiteShapes);
        auto tBuild1 = std::chrono::high_resolution_clock::now();
        double msBuild = std::chrono::duration<double, std::milli>(tBuild1 - tBuild0).count();
        std::cout << "BVH built over " << finiteShapes.size() << " shapes in "
                  << msBuild << " ms.\n";

        // ---------- 5) Render ----------
        const int W = cam.film_w_px, H = cam.film_h_px;
        ImagePPM img(W, H, {135, 206, 235}); // background
        const Vec3 white{1,1,1};

        auto t0 = std::chrono::high_resolution_clock::now();

        SceneAccel scene{ &bvh, &infiniteShapes };

        RenderParams params;
        params.maxDepth = 6;
        params.eps = 1e-4;

        int spp = 4; // samples per pixel
        bool stratified = true;
        RNG rng(12345);

        for (int y = 0; y < H; ++y){
            for (int x = 0; x < W; ++x){
                Vec3 accum{0,0,0};

                if (stratified) {
                    int n = (int)std::floor(std::sqrt((double)spp) + 0.5);
                    if (n*n != spp) n = 0; // fall back to pure jitter if spp not square

                    if (n > 0) {
                        // n x n stratified jitter
                        for (int j = 0; j < n; ++j){
                            for (int i = 0; i < n; ++i){
                                double u = (i + rng.next()) / n;  // in [0,1)
                                double v = (j + rng.next()) / n;  // in [0,1)
                                // If rayFromPixel expects pixel *corners*, sample at x+u,y+v.
                                // If it expects *centers*, change to (x + (u - 0.5)), etc.
                                Ray ray = cam.rayFromPixel((float)(x + u), (float)(y + v));
                                Vec3 c  = shadeRay(ray, 0, scene, lights, mats, params);
                                accum += c;
                            }
                        }
                        accum /= double(spp);
                    } else {
                        // pure jitter (spp not a perfect square)
                        for (int s = 0; s < spp; ++s){
                            double u = rng.next(); // [0,1)
                            double v = rng.next(); // [0,1)
                            Ray ray = cam.rayFromPixel((float)(x + u), (float)(y + v));
                            Vec3 c  = shadeRay(ray, 0, scene, lights, mats, params);
                            accum += c;
                        }
                        accum /= double(spp);
                    }
                } else {
                    // plain jittered supersampling
                    for (int s = 0; s < spp; ++s){
                        double u = rng.next(); // [0,1)
                        double v = rng.next(); // [0,1)
                        Ray ray = cam.rayFromPixel((float)(x + u), (float)(y + v));
                        Vec3 c  = shadeRay(ray, 0, scene, lights, mats, params);
                        accum += c;
                    }
                    accum /= double(spp);
                }

                // (Optional) gamma correction — uncomment if you want nicer mid-tones
                auto gc = [](double a){ return std::pow(std::clamp(a,0.0,1.0), 1.0/2.2); };
                Vec3 out = { gc(accum.x), gc(accum.y), gc(accum.z) };

                img.set(x, y,
                        Pixel(clamp8(out.x * 255.0),
                            clamp8(out.y * 255.0),
                            clamp8(out.z * 255.0)));
            }
        }


        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Render completed in " << ms << " ms.\n";

        // ---------- 6) Save ----------
        const std::string outPath = "../Output/rendertest33.ppm";
        img.save(outPath);
        std::cout << "Rendered: " << outPath << "\n";

    }catch(const std::exception& e){
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}




