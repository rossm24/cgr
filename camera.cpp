#include "camera.h"
#include <algorithm>

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


/*
void Camera::loadFromSceneTxt(const std::string& path){
    std::istringstream ss(readFile(path));
    std::string line;
    bool foundCam = false;

    while (std::getline(ss, line)) {
        if (!startsWith(line, "CAM ")) continue;

        // name
        name = extractName(line);

        // loc, rot (optional), gaze, focal, sensor, film
        if (!extractVec3(line, "loc", position))
            throw std::runtime_error("CAM: loc=(...) missing");
        // rot is exported but not needed; ignore if absent

        if (!extractVec3(line, "gaze", forward))
            throw std::runtime_error("CAM: gaze=(...) missing");

        if (!extractNumber(line, "focal", focal_mm))
            throw std::runtime_error("CAM: focal= missing");

        if (!extractVec2(line, "sensor", sensor_w_mm, sensor_h_mm))
            throw std::runtime_error("CAM: sensor=(w h) missing");

        if (!extractVec2i(line, "film", film_w_px, film_h_px))
            throw std::runtime_error("CAM: film=(W H) missing");

        buildBasis();
        foundCam = true;
        break;
    }

    if (!foundCam) {
        throw std::runtime_error("No CAM line found in scene.txt");
    }

    // Basic sanity checks
    if (film_w_px <= 0 || film_h_px <= 0)
        throw std::runtime_error("Invalid film resolution");
    if (sensor_w_mm <= 0 || sensor_h_mm <= 0 || focal_mm <= 0)
        throw std::runtime_error("Invalid camera intrinsics");
}
*/