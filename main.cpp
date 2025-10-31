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

#include "camera.h"     // Vec3, Ray, Camera
#include "image_ppm.h"
#include "mat4.h"
#include "shape.h"
#include "sphere.h"
#include "cube.h"
#include "plane.h"
#include "bvh.h"
#include "hit.h"

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

int main(){
    try{
        const std::string scenePath = "../ASCII/scene1.txt";

        // ---------- 1) Load Camera ----------
        Camera cam;
        cam.loadFromSceneTxt(scenePath);
        std::cout << "Loaded camera: " << cam.name << " ("
                  << cam.film_w_px << "x" << cam.film_h_px << ")\n";

        // ---------- 2) Parse scene objects ----------
        struct Light { Vec3 pos{}; double intensity=0.0; };
        std::vector<Light> lights;

        struct CubeAscii  { Vec3 center{}; Vec3 euler{}; double edge=1.0; };
        struct SphereAscii{ Vec3 center{}; Vec3 euler{}; Vec3 scale{1,1,1}; double radius=1.0; };
        struct PlaneAscii {
            Vec3 c0{}, c1{}, c2{}, c3{};
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
                lights.push_back(L);
            }
            else if (line.rfind("CUB ", 0) == 0) {
                CubeAscii C{};
                if (!extractVec3(line, "trans", C.center))
                    throw std::runtime_error("CUB: trans missing");
                extractVec3(line, "rot", C.euler);
                double s1=1.0;
                if (!extractNumber(line, "scale1d", s1))
                    throw std::runtime_error("CUB: scale1d missing");
                C.edge = s1;
                cubes.push_back(C);
            }
            else if (line.rfind("SPH ", 0) == 0) {
                SphereAscii S{};
                if (!extractVec3(line, "loc", S.center))
                    throw std::runtime_error("SPH: loc missing");
                extractVec3(line, "rot",   S.euler);
                extractVec3(line, "scale", S.scale);
                if (!extractNumber(line, "radius", S.radius))
                    throw std::runtime_error("SPH: radius missing");
                spheres.push_back(S);
            }
            else if (line.rfind("PLN ", 0) == 0) {
            if (!extractVec3(line, "c0", pln.c0)) throw std::runtime_error("PLN: c0 missing");
            if (!extractVec3(line, "c1", pln.c1)) throw std::runtime_error("PLN: c1 missing");
            if (!extractVec3(line, "c2", pln.c2)) throw std::runtime_error("PLN: c2 missing");
            if (!extractVec3(line, "c3", pln.c3)) throw std::runtime_error("PLN: c3 missing");

            havePlane = true;
            }

        }

        if (lights.empty()) {
            std::cerr << "[warn] No lights; using a default light.\n";
            lights.push_back(Light{ {4,1,6}, 80.0 });
        }
        // your stronger tweak
        for (auto& L : lights) {
            L.intensity *= 2.0;
        }

        // ---------- 3) Instantiate shapes ----------
        std::vector<Shape*> shapes;
        std::vector<std::unique_ptr<Shape>> owned;

        // Cubes — your cube class already fits your framework
        for (size_t i = 0; i < cubes.size(); ++i) {
            auto cb = std::make_unique<Cube>();
            cb->id = int(100 + i);

            Mat4 C = composeTRS(
                cubes[i].center,
                cubes[i].euler,
                {cubes[i].edge, cubes[i].edge, cubes[i].edge}
            );

            cb->setTransform(C);
            shapes.push_back(cb.get());
            owned.push_back(std::move(cb));
        }

        // Spheres — create using the NEW ctor (name + 6 floats)
        for (size_t i = 0; i < spheres.size(); ++i) {
            const auto& S = spheres[i];

            auto sp = std::make_unique<Sphere>(
                "sphere_" + std::to_string(i),
                (float)S.center.x, (float)S.center.y, (float)S.center.z,
                (float)S.scale.x,  (float)S.scale.y,  (float)S.scale.z
            );
            sp->id = int(200 + i);

            // NOTE: our new Sphere already uses its own pos/scale in intersect,
            // so we don't *have* to call setTransform here.
            // If you want rotations from Blender to apply, you *could* do:
            //
            // Mat4 T = composeTRS(S.center, S.euler, S.scale);
            // sp->setTransform(T);
            //
            // but then you'd be mixing "internal float scale" and "matrix scale".
            // Let's keep it clean and skip transform for spheres for now.

            shapes.push_back(sp.get());
            owned.push_back(std::move(sp));
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

            // we can leave the transform as identity — the corners are already in object space
            // if later you want to move/rotate it, call pl->setTransform(...)

            shapes.push_back(pl.get());
            owned.push_back(std::move(pl));
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

        for(int y=0;y<H;++y){
            for(int x=0;x<W;++x){
                Ray ray = cam.rayFromPixel((float)x, (float)y); // dir normalized

                Hit best;
                bool any=false;

                // finite via BVH
                any |= bvh.intersect(ray, 1e-5, 1e9, best);
                // infinite (if any)
                for(Shape* s : infiniteShapes)
                    any |= s->intersect(ray, 1e-5, 1e9, best);

                if(any){
                    const auto& L = lights[0];
                    Vec3 c = shade_diffuse(best, L.pos, L.intensity, white);
                    img.set(x, y, Pixel(clamp8(c.x*255.0), clamp8(c.y*255.0), clamp8(c.z*255.0)));
                }
            }
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Render completed in " << ms << " ms.\n";

        // ---------- 6) Save ----------
        const std::string outPath = "../Output/render1.ppm";
        img.save(outPath);
        std::cout << "Rendered: " << outPath << "\n";

    }catch(const std::exception& e){
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

/*
else if (line.rfind("PLN ", 0) == 0) {
                if (!extractVec3(line, "c0", pln.c0)) throw std::runtime_error("PLN: c0 missing");
                if (!extractVec3(line, "c1", pln.c1)) throw std::runtime_error("PLN: c1 missing");
                if (!extractVec3(line, "c2", pln.c2)) throw std::runtime_error("PLN: c2 missing");
                if (!extractVec3(line, "c3", pln.c3)) throw std::runtime_error("PLN: c3 missing");
                havePlane = true;
*/



