#pragma once
#include <map>
#include <memory>
#include <string>
#include "texture.h"

class TextureManager {
public:
    Texture* get(const std::string& path) {
        auto it = texs.find(path);
        if (it != texs.end()) return it->second.get();

        auto tex = std::make_unique<Texture>();
        if (!tex->load(path)) {
            // failed to load; you might want a debug print
            return nullptr;
        }

        Texture* ptr = tex.get();
        texs[path] = std::move(tex);
        return ptr;
    }

private:
    std::map<std::string, std::unique_ptr<Texture>> texs;
};

