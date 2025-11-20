#include "camera.h"
#include <algorithm>
#include <iostream>

// ---------- small file & string utils ----------

std::string Camera::readFile(const std::string& path){
    std::ifstream f(path);
    if(!f) throw std::runtime_error("Cannot open file: " + path);
    std::ostringstream ss; ss << f.rdbuf(); 
    return ss.str();
}

bool Camera::startsWith(const std::string& s, const std::string& pfx){
    return s.size() >= pfx.size() && std::equal(pfx.begin(), pfx.end(), s.begin());
}

static inline void skipWS(const std::string& s, size_t& i){
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
}

static double parseNumberAt(const std::string& s, size_t& i){
    skipWS(s, i);
    size_t start = i;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    bool hasDigits=false;
    while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; hasDigits=true; }
    if (i < s.size() && s[i] == '.') {
        ++i; 
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; hasDigits=true; }
    }
    if (i < s.size() && (s[i]=='e' || s[i]=='E')) {
        ++i;
        if (i < s.size() && (s[i]=='+' || s[i]=='-')) ++i;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
    }
    if (!hasDigits) throw std::runtime_error("Expected number at: " + s.substr(start, 16));
    return std::stod(s.substr(start, i-start));
}

static bool findKeyOpenParen(const std::string& line, const std::string& key, size_t& k){
    // find "key=("
    std::string needle = key + "=(";
    k = line.find(needle);
    if (k == std::string::npos) return false;
    k += needle.size();
    return true;
}

// Build camera basis from Blender-style Euler XYZ rotation.
// Uses the same convention as your eulerXYZ_to_R: R = Rz * Ry * Rx.
// In Blender camera local space we assume:
//   forward_local = (0, 1, 0)
//   up_local      = (0, 0, 1)
//   right_local   = (1, 0, 0)
static void basisFromEulerXYZ(const Vec3& rot,
                              Vec3& forward,
                              Vec3& up,
                              Vec3& right)
{
    double rx = rot.x;
    double ry = rot.y;
    double rz = rot.z;

    double cx = std::cos(rx), sx = std::sin(rx);
    double cy = std::cos(ry), sy = std::sin(ry);
    double cz = std::cos(rz), sz = std::sin(rz);

    // Rotation matrices
    // Rx
    double Rx00 = 1,  Rx01 = 0,   Rx02 = 0;
    double Rx10 = 0,  Rx11 = cx,  Rx12 = -sx;
    double Rx20 = 0,  Rx21 = sx,  Rx22 = cx;

    // Ry
    double Ry00 = cy,  Ry01 = 0,  Ry02 = sy;
    double Ry10 = 0,   Ry11 = 1,  Ry12 = 0;
    double Ry20 = -sy, Ry21 = 0,  Ry22 = cy;

    // Rz
    double Rz00 = cz,  Rz01 = -sz, Rz02 = 0;
    double Rz10 = sz,  Rz11 = cz,  Rz12 = 0;
    double Rz20 = 0,   Rz21 = 0,   Rz22 = 1;

    // R = Rz * Ry * Rx
    auto mul3 = [&](double A00,double A01,double A02,
                    double A10,double A11,double A12,
                    double A20,double A21,double A22,
                    double B00,double B01,double B02,
                    double B10,double B11,double B12,
                    double B20,double B21,double B22,
                    double& C00,double& C01,double& C02,
                    double& C10,double& C11,double& C12,
                    double& C20,double& C21,double& C22)
    {
        C00 = A00*B00 + A01*B10 + A02*B20;
        C01 = A00*B01 + A01*B11 + A02*B21;
        C02 = A00*B02 + A01*B12 + A02*B22;

        C10 = A10*B00 + A11*B10 + A12*B20;
        C11 = A10*B01 + A11*B11 + A12*B21;
        C12 = A10*B02 + A11*B12 + A12*B22;

        C20 = A20*B00 + A21*B10 + A22*B20;
        C21 = A20*B01 + A21*B11 + A22*B21;
        C22 = A20*B02 + A21*B12 + A22*B22;
    };

    // Ry * Rx
    double T00,T01,T02,T10,T11,T12,T20,T21,T22;
    mul3(Ry00,Ry01,Ry02,
         Ry10,Ry11,Ry12,
         Ry20,Ry21,Ry22,
         Rx00,Rx01,Rx02,
         Rx10,Rx11,Rx12,
         Rx20,Rx21,Rx22,
         T00,T01,T02,
         T10,T11,T12,
         T20,T21,T22);

    // R = Rz * (Ry * Rx)
    double R00,R01,R02,R10,R11,R12,R20,R21,R22;
    mul3(Rz00,Rz01,Rz02,
         Rz10,Rz11,Rz12,
         Rz20,Rz21,Rz22,
         T00,T01,T02,
         T10,T11,T12,
         T20,T21,T22,
         R00,R01,R02,
         R10,R11,R12,
         R20,R21,R22);

    // Local basis in Blender camera space
    Vec3 fL(0, 1, 0);  // forward
    Vec3 uL(0, 0, 1);  // up
    Vec3 rL(1, 0, 0);  // right

    auto applyR = [&](const Vec3& v)->Vec3 {
        return {
            R00*v.x + R01*v.y + R02*v.z,
            R10*v.x + R11*v.y + R12*v.z,
            R20*v.x + R21*v.y + R22*v.z
        };
    };

    forward = normalize(applyR(fL));
    up      = normalize(applyR(uL));
    right   = normalize(applyR(rL));
}


bool Camera::extractVec3(const std::string& line, const std::string& key, Vec3& out){
    size_t k;
    if (!findKeyOpenParen(line, key, k)) return false;
    size_t i = k;
    double a = parseNumberAt(line, i);
    double b = parseNumberAt(line, i);
    double c = parseNumberAt(line, i);
    skipWS(line, i);
    if (i >= line.size() || line[i] != ')') throw std::runtime_error("Expected ')' after " + key);
    out = {a,b,c};
    return true;
}

bool Camera::extractVec2(const std::string& line, const std::string& key, double& a, double& b){
    size_t k;
    if (!findKeyOpenParen(line, key, k)) return false;
    size_t i = k;
    a = parseNumberAt(line, i);
    b = parseNumberAt(line, i);
    skipWS(line, i);
    if (i >= line.size() || line[i] != ')') throw std::runtime_error("Expected ')' after " + key);
    return true;
}

bool Camera::extractVec2i(const std::string& line, const std::string& key, int& a, int& b){
    double da=0, db=0;
    if (!extractVec2(line, key, da, db)) return false;
    a = static_cast<int>(std::llround(da));
    b = static_cast<int>(std::llround(db));
    return true;
}

bool Camera::extractNumber(const std::string& line, const std::string& key, double& out){
    // find "key="
    std::string needle = key + "=";
    size_t k = line.find(needle);
    if (k == std::string::npos) return false;
    k += needle.size();
    size_t i = k;
    out = parseNumberAt(line, i);
    return true;
}

std::string Camera::extractName(const std::string& line){
    // name=Camera or name=Some_Name (no spaces in your exporter; safe fallback)
    std::string key = "name=";
    size_t k = line.find(key);
    if (k == std::string::npos) return {};
    k += key.size();
    // read until whitespace or next key
    size_t i = k;
    while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) ++i;
    return line.substr(k, i - k);
}

// ---------- Camera basis ----------

void Camera::buildBasis(){
    // 'forward' is already world-space gaze from exporter (Camera looks along it)
    forward = normalize(forward);

    // Use a world-up hint; robust if nearly parallel
    Vec3 upWorld{0, 0, 1};
    if (std::fabs(dot(forward, upWorld)) > 0.999) upWorld = {0, 1, 0};

    // Right-handed basis: right = forward × upWorld, up = right × forward
    // (This makes +X to camera's right, +Y roughly up)
    right = normalize(cross(forward, upWorld));
    if (length(right) == 0.0) right = {1,0,0}; // extreme fallback
    up = normalize(cross(right, forward));
}

// ---------- Load from ASCII scene.txt ----------

void Camera::loadFromSceneTxt(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Camera: cannot open " + path);

    std::string line;
    while (std::getline(f, line)) {
        if (!startsWith(line, "CAM ")) continue;

        name = extractName(line);

        Vec3 loc, gaze;
        if (!extractVec3(line, "loc", loc))
            throw std::runtime_error("CAM: loc missing");
        if (!extractVec3(line, "gaze", gaze))
            throw std::runtime_error("CAM: gaze missing");

        // --- HERE is the important part ---
        // convert from Blender coords to renderer coords
        position = loc;
        forward  = normalize(gaze);

        // ----------------------------------

        if (!extractNumber(line, "focal", focal_mm))
            focal_mm = 50.0;

        if (!extractVec2(line, "sensor", sensor_w_mm, sensor_h_mm)) {
            sensor_w_mm = 36.0;
            sensor_h_mm = 24.0;
        }

        if (!extractVec2i(line, "film", film_w_px, film_h_px)) {
            film_w_px = 800;
            film_h_px = 600;
        }

        // build the camera basis AFTER we've fixed axes
        buildBasis();

        // DEBUG: print camera basis
        std::cerr << "[CAM] pos   = (" << position.x << ", " << position.y << ", " << position.z << ")\n";
        std::cerr << "[CAM] gaze  = (" << forward.x  << ", " << forward.y  << ", " << forward.z  << ")\n";
        std::cerr << "[CAM] right = (" << right.x    << ", " << right.y    << ", " << right.z    << ")\n";
        std::cerr << "[CAM] up    = (" << up.x       << ", " << up.y       << ", " << up.z       << ")\n";

        return;
    }

    throw std::runtime_error("Camera: no CAM line in " + path);
}


// ---------- Pixel -> world ray ----------

Ray Camera::rayFromPixel(float px, float py) const {

    // Normalized device coords in [-0.5, 0.5]
    double ndcX = (static_cast<double>(px) + 0.5) / static_cast<double>(film_w_px) - 0.5;
    double ndcY = (static_cast<double>(py) + 0.5) / static_cast<double>(film_h_px) - 0.5;

    // Map to sensor mm. Flip Y so image isn't vertically inverted.
    double sensorX = ndcX * sensor_w_mm;
    double sensorY = -ndcY * sensor_h_mm;

    // P on image plane at distance focal_mm along 'forward'
    Vec3 pixelPoint = position 
                    + forward * focal_mm 
                    + right   * sensorX 
                    + up      * sensorY;

    Vec3 dir_world = normalize(pixelPoint - position);
    return Ray{ position, dir_world };
}


