#pragma once
#include <vector>
#include <memory>
#include <algorithm>
#include "aabb.h"
#include "shape.h"

// bounding volume hierarchy 

class BVH {
public:
    BVH() = default;

    // Build over the subset of shapes that are finite
    void build(const std::vector<Shape*>& shapes);

    // Traverse BVH; returns true if it found a closer hit (updates 'hit')
    bool intersect(const Ray& r, double tMin, double tMax, Hit& hit) const;

    bool empty() const { return nodes_.empty(); }

private:
    struct Item {
        Shape* shape;
        AABB   bounds;
        Vec3   centroid;
    };
    struct Node {
        AABB box;
        int  left  = -1;
        int  right = -1;
        int  start = 0;   // index into 'indices' for leaf
        int  count = 0;   // number of items in leaf
        bool isLeaf() const { return count > 0; }
    };

    std::vector<Item> items_;       // sorted during build
    std::vector<int>  indices_;     // permutation of items_
    std::vector<Node> nodes_;

    int buildRange(int begin, int end); // returns node index
};
