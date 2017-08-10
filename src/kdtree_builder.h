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

KdTree build_kdtree(const TriangleMesh& mesh, const KdTree_Build_Params& build_params);
