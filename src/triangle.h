#pragma once

#include "ray.h"
#include "vector.h"
#include <array>
#include <limits>

struct Triangle {
  std::array<Vector, 3> points;

  struct Intersection {
    float t = std::numeric_limits<float>::infinity();
    float epsilon = 0.0;
    float b1 = 0.0;
    float b2 = 0.0;
  };
};

bool IntersectTriangle(const Ray& ray, const Triangle& triangle,
                       Triangle::Intersection& intersection);
