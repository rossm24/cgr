#include "bvh.h"

static inline Vec3 centroidOf(const AABB& b){
    return { (b.min.x + b.max.x)*0.5,
             (b.min.y + b.max.y)*0.5,
             (b.min.z + b.max.z)*0.5 };
}

void BVH::build(const std::vector<Shape*>& shapes){
    // collect finite items
    items_.clear(); indices_.clear(); nodes_.clear();
    items_.reserve(shapes.size());
    for(auto* s : shapes){
        if(!s->isFinite()) continue;
        AABB wb = s->worldBounds();
        Item it; it.shape = s; it.bounds = wb; it.centroid = centroidOf(wb);
        items_.push_back(it);
    }
    const int N = (int)items_.size();
    indices_.resize(N);
    for(int i=0;i<N;++i) indices_[i]=i;
    if(N==0) return;

    nodes_.reserve(2*N);
    buildRange(0, N);
}

int BVH::buildRange(int begin, int end){
    Node node{};
    // compute parent box
    for(int i=begin;i<end;++i) node.box.expand(items_[indices_[i]].bounds);

    const int count = end - begin;
    const int nodeIndex = (int)nodes_.size();
    nodes_.push_back(node);

    // small leaf
    const int LEAF_SIZE = 4;
    if(count <= LEAF_SIZE){
        nodes_[nodeIndex].start = begin;
        nodes_[nodeIndex].count = count;
        return nodeIndex;
    }

    // split by longest-axis centroid
    AABB centBounds;
    for(int i=begin;i<end;++i) centBounds.expand(items_[indices_[i]].centroid);
    int axis = centBounds.longestAxis();
    double mid = (axis==0 ? (centBounds.min.x+centBounds.max.x)*0.5
               : axis==1 ? (centBounds.min.y+centBounds.max.y)*0.5
                         : (centBounds.min.z+centBounds.max.z)*0.5);

    auto it = std::partition(indices_.begin()+begin, indices_.begin()+end,
        [&](int idx){
            const Vec3& c = items_[idx].centroid;
            return (axis==0? c.x : axis==1? c.y : c.z) < mid;
        });

    int midIdx = (int)std::distance(indices_.begin(), it);
    if(midIdx==begin || midIdx==end){
        // fallback: median split by index
        midIdx = begin + count/2;
        std::nth_element(indices_.begin()+begin, indices_.begin()+midIdx, indices_.begin()+end,
            [&](int a, int b){
                const Vec3& ca = items_[a].centroid;
                const Vec3& cb = items_[b].centroid;
                double va = (axis==0?ca.x:axis==1?ca.y:ca.z);
                double vb = (axis==0?cb.x:axis==1?cb.y:cb.z);
                return va < vb;
            });
    }

    int left  = buildRange(begin, midIdx);
    int right = buildRange(midIdx, end);

    nodes_[nodeIndex].left  = left;
    nodes_[nodeIndex].right = right;
    return nodeIndex;
}

bool BVH::intersect(const Ray& r, double tMin, double tMax, Hit& hit) const{
    if(nodes_.empty()) return false;
    bool any = false;
    int stack[64]; int sp=0;
    stack[sp++] = 0;

    while(sp){
        int ni = stack[--sp];
        const Node& n = nodes_[ni];
        if(!n.box.intersect(r, tMin, std::min(tMax, hit.t))) continue;

        if(n.isLeaf()){
            for(int i=0;i<n.count;++i){
                Shape* s = items_[indices_[n.start+i]].shape;
                any |= s->intersect(r, tMin, tMax, hit);
            }
        }else{
            stack[sp++] = n.left;
            stack[sp++] = n.right;
        }
    }
    return any;
}
