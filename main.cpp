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
#include "RNG.h"

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

static Ray makeCameraRay(double px,
                         double py,
                         const Camera& cam,
                         const RenderParams& params,
                         RNG& rng)
{
    // Base pinhole ray from your camera
    Ray base = cam.rayFromPixel((float)px, (float)py);
    Vec3 origin = base.origin;  // usually cam.pos
    Vec3 dir    = normalize(base.dir);

    // -----------------------
    // 1) Pick a random time in shutter interval (for motion blur)
    // -----------------------
    double time = 0.0;
    if (params.enableMotionBlur) {
        double u = rng.next();
        time = params.shutterOpen + u * (params.shutterClose - params.shutterOpen);
    }

    // -----------------------
    // 2) Depth of field: sample lens disk & refocus at focalDistance
    // -----------------------
    if (params.enableDOF && params.apertureRadius > 0.0 && params.focalDistance > 0.0) {
        // Focus point along the original pinhole ray
        Vec3 focusPoint = origin + dir * params.focalDistance;

        // Sample point on lens disk
        double r   = std::sqrt(rng.next());
        double phi = 2.0 * 3.14159265358979323846 * rng.next();
        double dx  = params.apertureRadius * r * std::cos(phi);
        double dy  = params.apertureRadius * r * std::sin(phi);

        // Use camera basis (pos/right/up from your debug prints)
        Vec3 lensOffset = dx * cam.right + dy * cam.up;

        Vec3 newOrigin = cam.position + lensOffset;
        Vec3 newDir    = normalize(focusPoint - newOrigin);

        origin = newOrigin;
        dir    = newDir;
    }

    // -----------------------
    // 3) Camera motion blur: move camera along right over time
    // -----------------------
    if (params.enableMotionBlur && params.camMotionAmount != 0.0) {
        // Map time in [shutterOpen, shutterClose] to offset in [-0.5, +0.5]
        double mid   = 0.5 * (params.shutterOpen + params.shutterClose);
        double span  = (params.shutterClose - params.shutterOpen);
        double normT = (span > 0.0) ? (time - mid) / span : 0.0; // -0.5..+0.5 roughly

        origin += normT * params.camMotionAmount * cam.right;
    }

    Ray ray;
    ray.origin = origin;
    ray.dir    = dir;

    return ray;
}

static double luminance(const Vec3& c)
{
    return 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
}

static Vec3 sampleClamped(const std::vector<Vec3>& buf,
                          int x, int y, int W, int H)
{
    if (x < 0) x = 0;
    if (x >= W) x = W - 1;
    if (y < 0) y = 0;
    if (y >= H) y = H - 1;
    return buf[y * W + x];
}

static void applyPostFX(const std::vector<Vec3>& hdr,
                        ImagePPM& img,
                        int W, int H,
                        const RenderParams& params)
{
    // If bloom & lens flare are both off, just gamma copy
    if (!params.enableBloom && !params.enableLensFlare) {
        auto gc = [](double a){
            return std::pow(std::clamp(a, 0.0, 1.0), 1.0/2.2);
        };

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const Vec3& c = hdr[y * W + x];
                Vec3 out{ gc(c.x), gc(c.y), gc(c.z) };
                img.set(x, y,
                    Pixel(clamp8(out.x * 255.0),
                          clamp8(out.y * 255.0),
                          clamp8(out.z * 255.0)));
            }
        }
        return;
    }

    // -------- 1) Bright pass --------
    std::vector<Vec3> bright(W * H, Vec3{0.0, 0.0, 0.0});
    double threshold = params.bloomThreshold;

    for (int i = 0; i < W * H; ++i) {
        double L = luminance(hdr[i]);
        if (L > threshold) {
            bright[i] = hdr[i];
        }
    }

    // -------- 2) Separable blur (5-tap: 1 4 6 4 1 / 16) --------
    std::vector<Vec3> tmp(W * H);
    std::vector<Vec3> blurred(W * H);

    // Horizontal blur
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            Vec3 c =
                sampleClamped(bright, x-2, y,   W, H) * 1.0 +
                sampleClamped(bright, x-1, y,   W, H) * 4.0 +
                sampleClamped(bright, x,   y,   W, H) * 6.0 +
                sampleClamped(bright, x+1, y,   W, H) * 4.0 +
                sampleClamped(bright, x+2, y,   W, H) * 1.0;
            tmp[y * W + x] = c * (1.0 / 16.0);
        }
    }

    // Vertical blur
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            Vec3 c =
                sampleClamped(tmp, x, y-2, W, H) * 1.0 +
                sampleClamped(tmp, x, y-1, W, H) * 4.0 +
                sampleClamped(tmp, x, y,   W, H) * 6.0 +
                sampleClamped(tmp, x, y+1, W, H) * 4.0 +
                sampleClamped(tmp, x, y+2, W, H) * 1.0;
            blurred[y * W + x] = c * (1.0 / 16.0);
        }
    }

    // -------- 3) Optional: simple lens flare ghosts --------
    // ---- Strong Cinematic Lens Flare Ghosts ----
    if (params.enableLensFlare) {
        int brightestIdx = 0;
        double bestL = 0.0;

        for (int i = 0; i < W * H; ++i) {
            double L = luminance(hdr[i]);
            if (L > bestL) {
                bestL = L;
                brightestIdx = i;
            }
        }

        int bx = brightestIdx % W;
        int by = brightestIdx / W;
        int cx = W / 2;
        int cy = H / 2;

        double dx = bx - cx;
        double dy = by - cy;

        // Strong ghost positions
        std::vector<double> ghostPos = { -1.2, -0.6, 0.3, 0.7, 1.3 };

        for (double g : ghostPos) {
            int gx = int(cx + g * dx);
            int gy = int(cy + g * dy);

            // Large ghost radius
            const int radius = 40;
            for (int oy = -radius; oy <= radius; ++oy) {
                for (int ox = -radius; ox <= radius; ++ox) {

                    int x = gx + ox;
                    int y = gy + oy;
                    if (x < 0 || x >= W || y < 0 || y >= H) continue;

                    double dist = std::sqrt(ox * ox + oy * oy);
                    if (dist > radius) continue;

                    double w = 1.0 - (dist / radius);
                    Vec3 add = hdr[brightestIdx] * (params.flareStrength * w * 0.8);
                    blurred[y * W + x] = blurred[y * W + x] + add;
                }
            }
        }
    }


    // -------- 4) Combine HDR + bloom, gamma, write to img --------
    auto gc = [](double a){
        return std::pow(std::clamp(a, 0.0, 1.0), 1.0/2.2);
    };

    double bloomK = params.bloomStrength;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int idx = y * W + x;

            Vec3 c = hdr[idx] + bloomK * blurred[idx];

            Vec3 out{ gc(c.x), gc(c.y), gc(c.z) };
            img.set(x, y,
                Pixel(clamp8(out.x * 255.0),
                      clamp8(out.y * 255.0),
                      clamp8(out.z * 255.0)));
        }
    }
}




int main(int argc, char** argv){
    try{
        //const std::string scenePath = "../ASCII/scene.txt";

        std::string scenePath = "../ASCII/scene.txt";
        std::string outPath   = "../Output/output.ppm";

        RenderParams params;

        

        // Helper: does this string contain a path separator?
        auto hasPathSep = [](const std::string& s) {
            return s.find('/') != std::string::npos ||
                s.find('\\') != std::string::npos;
        };

        // comand line arguments 
        // Usage examples:
        //   ./raytrace                   -> uses defaults
        //   ./raytrace myscene.txt       -> overrides scenePath
        //   ./raytrace myscene.txt my.ppm --spp 16 --max-depth 8
        if (argc >= 2) {
            // If first arg is not a flag, treat it as scene name
            if (argv[1][0] != '-') {
                std::string name = argv[1];

                // If no '/' or '\' then assume it lives in ../ASCII/
                if (!hasPathSep(name)) {
                    scenePath = std::string("../ASCII/") + name;
                } else {
                    scenePath = name; // user gave full/relative path explicitly
                }
            }

            // If second arg exists and is not a flag, treat it as output name
            if (argc >= 3 && argv[2][0] != '-') {
                std::string name = argv[2];

                // If no '/' or '\' then save into ../Output/
                if (!hasPathSep(name)) {
                    outPath = std::string("../Output/") + name;
                } else {
                    outPath = name; // user gave full/relative path explicitly
                }
            }

            // Parse options (we just look at args that *are* flags)
            for (int i = 1; i < argc; ++i) {
                std::string arg = argv[i];

                if (arg == "--spp" && i + 1 < argc) {
                    params.spp = std::stoi(argv[++i]);
                } else if (arg == "--max-depth" && i + 1 < argc) {
                    params.maxDepth = std::stoi(argv[++i]);
                } else if (arg == "--eps" && i + 1 < argc) {
                    params.eps = std::stod(argv[++i]);
                } else if (arg == "--no-stratified") {
                    params.stratified = false;
                } else if (arg == "--soft-shadows" && i + 1 < argc) {
                    params.enableSoftShadows = true;
                    params.shadowSamples = std::stoi(argv[++i]);
                } else if (arg == "--soft-shadow-radius" && i + 1 < argc) {
                    params.softShadowRadius = std::stod(argv[++i]);
                } else if (arg == "--glossy" && i + 1 < argc) {
                    params.enableGlossy = true;
                    params.glossySamples = std::stoi(argv[++i]);
                } else if (arg == "-roughness" && i + 1 < argc) {
                    params.glossyRoughness = std::stod(argv[++i]);
                } else if (arg == "--dof" && i + 2 < argc) {
                    params.enableDOF = true;
                    params.apertureRadius = std::stod(argv[++i]);
                    params.focalDistance  = std::stod(argv[++i]);
                } else if (arg == "--motion-blur" && i + 3 < argc) {
                    params.enableMotionBlur = true;
                    params.camMotionAmount = std::stod(argv[++i]);
                    params.shutterOpen  = std::stod(argv[++i]);
                    params.shutterClose = std::stod(argv[++i]);        
                } else if (arg == "--bloom" && i + 2 < argc) {
                    params.enableBloom = true;
                    params.bloomThreshold = std::stod(argv[++i]);
                    params.bloomStrength  = std::stod(argv[++i]);
                } else if (arg == "--lens-flare" && i + 1 < argc) {
                    params.enableLensFlare = true;
                    params.flareStrength = std::stod(argv[++i]);
                } else if (arg == "--light-scale" && i + 1 < argc) {
                    params.lightScale = std::stod(argv[++i]);
                }
            }
        }

        // ---------- 1) Load Camera ----------
        Camera cam;
        cam.loadFromSceneTxt(scenePath);
        std::cout << "Loaded camera: " << cam.name << " ("
                  << cam.film_w_px << "x" << cam.film_h_px << ")\n";

        // ---------- 2) Parse scene objects ----------
        // struct Light { Vec3 pos{}; double intensity=0.0; };
        std::vector<Light> lights;

        //struct CubeAscii  { Vec3 center{}; Vec3 euler{}; double edge=1.0; Vec3 color; std::string texName;};
        struct CubeAscii  { Vec3 center{}; Vec3 euler{}; Vec3 scale{1.0, 1.0, 1.0}; Vec3 color; std::string texName;};
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
                //double s1=1.0;
                //if (!extractNumber(line, "scale1d", s1))
                  //  throw std::runtime_error("CUB: scale1d missing");
                //C.edge = s1;
                if (!extractVec3(line, "scale", C.scale))
                {
                    // Fall back to uniform scale1d=
                    double s1 = 1.0;
                    if (!extractNumber(line, "scale1d", s1))
                        throw std::runtime_error("CUB: scale/scale1d missing");
                    C.scale = {s1, s1, s1};
                }

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
            //Vec3 scale = { cubes[i].edge, cubes[i].edge, cubes[i].edge };

            Vec3 scale = cubes[i].scale;


            // T * R * S (this is correct with your composeTRS + new Mat4)
            Mat4 C = composeTRS(centerR, euler, scale);

            

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
                .shininess = 128,
                .reflectivity = 0.4
            };
        }

        // Spheres
        for (size_t i = 0; i < spheres.size(); ++i) {
            int id = int(200 + i);
            mats.by_id[id] = Material{
                .kd = spheres[i].color,
                .ks = {0.3,0.3,0.3},
                .shininess = 64,
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

        // HDR buffer (linear colour)
        std::vector<Vec3> hdr(W * H, Vec3{0.0, 0.0, 0.0});

        const Vec3 white{1,1,1};

        auto t0 = std::chrono::high_resolution_clock::now();

        SceneAccel scene{ &bvh, &infiniteShapes };

        
        RNG rng(12345);
        //params.maxDepth = 6;
        //params.eps = 1e-4;

        //int spp = 4; // samples per pixel
        //bool stratified = true;

        params.maxDepth   = 6;
        params.eps        = 1e-4;
        params.spp        = 4;       // formerly `int spp = 4`
        params.stratified = true;    // formerly `bool stratified = true`
        

        for (int y = 0; y < H; ++y){
            for (int x = 0; x < W; ++x){
                Vec3 accum{0,0,0};

                if (params.stratified) {
                    int n = (int)std::floor(std::sqrt((double)params.spp) + 0.5);
                    if (n*n != params.spp) n = 0; // fall back to pure jitter if spp not square
                    if (n > 0) {
                        // n x n stratified jitter
                        for (int j = 0; j < n; ++j){
                            for (int i = 0; i < n; ++i){
                                double u = (i + rng.next()) / n;  // in [0,1)
                                double v = (j + rng.next()) / n;  // in [0,1)
                                // If rayFromPixel expects pixel *corners*, sample at x+u,y+v.
                                // If it expects *centers*, change to (x + (u - 0.5)), etc.
                                //Ray ray = cam.rayFromPixel((float)(x + u), (float)(y + v));
                                Ray ray = makeCameraRay(x + u, y + v, cam, params, rng);
                                Vec3 c  = shadeRay(ray, 0, scene, lights, mats, params, rng);
                                accum += c;
                            }
                        }
                        accum /= double(params.spp);
                    } else {
                        // pure jitter (spp not a perfect square)
                        for (int s = 0; s < params.spp; ++s){
                            double u = rng.next(); // [0,1)
                            double v = rng.next(); // [0,1)
                            //Ray ray = cam.rayFromPixel((float)(x + u), (float)(y + v));
                            Ray ray = makeCameraRay(x + u, y + v, cam, params, rng);
                            Vec3 c  = shadeRay(ray, 0, scene, lights, mats, params, rng);
                            accum += c;
                        }
                        accum /= double(params.spp);
                    }
                } else {
                    // plain jittered supersampling
                    for (int s = 0; s < params.spp; ++s){
                        double u = rng.next(); // [0,1)
                        double v = rng.next(); // [0,1)
                        //Ray ray = cam.rayFromPixel((float)(x + u), (float)(y + v));
                        Ray ray = makeCameraRay(x + u, y + v, cam, params, rng);
                        Vec3 c  = shadeRay(ray, 0, scene, lights, mats, params, rng);
                        accum += c;
                    }
                    accum /= double(params.spp);
                }

                // (Optional) gamma correction — uncomment if you want nicer mid-tones
                //auto gc = [](double a){ return std::pow(std::clamp(a,0.0,1.0), 1.0/2.2); };
                //Vec3 out = { gc(accum.x), gc(accum.y), gc(accum.z) };

                //img.set(x, y,
                  //      Pixel(clamp8(out.x * 255.0),
                    //        clamp8(out.y * 255.0),
                      //      clamp8(out.z * 255.0)));
                int idx = y * W + x;
                hdr[idx] = accum;
            }
        }


        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "Render completed in " << ms << " ms.\n";

        applyPostFX(hdr, img, W, H, params);

        // ---------- 6) Save ----------
        //const std::string outPath = "../Output/output.ppm";
        img.save(outPath);
        std::cout << "Rendered: " << outPath << "\n";

    }catch(const std::exception& e){
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}




