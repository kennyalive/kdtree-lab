#pragma once

#include "bounding_box.h"
#include <embree2/rtcore.h>

class KdTree;

enum { benchmarkRaysCount = 10000000 };

enum { debug_rays = false };
enum { debug_ray_count = 4 };

int BenchmarkKdTree(const KdTree& kdTree);
int BenchmarkEmbree(RTCScene scene, const BoundingBox& bounds);
void ValidateKdTree(const KdTree& kdTree, int raysCount);
