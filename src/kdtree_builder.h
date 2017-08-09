#pragma once

#include "kdtree.h"
#include <cstdint>

struct TriangleMesh;

struct KdTree_Build_Params {
    float intersectionCost = 80;
    float traversalCost = 1;
    float emptyBonus = 0.3f;
    int maxDepth = -1;
    bool splitAlongTheLongestAxis = false;
    // the actual amout of leaf triangles can be larger
    int leafTrianglesLimit = 2;
};

struct KdTree_Build_Stats {
    void NewLeaf(int leafTriangles, int depth);
    void FinalizeStats();
    void Print() const;

    int32_t leafCount = 0;
    int32_t emptyLeafCount = 0;
    int32_t single_triangle_leaf_count = 0;
    double trianglesPerLeaf = 0.0;
    int perfectDepth = 0;
    double averageDepth = 0.0;
    double depthStandardDeviation = 0.0;

private:
    int64_t trianglesPerLeafAccumulated = 0;
    std::vector<uint8_t> leafDepthValues;
};

KdTree build_kdtree(const TriangleMesh& mesh, const KdTree_Build_Params& build_params, KdTree_Build_Stats* stats = nullptr);
