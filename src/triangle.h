#pragma once

#include <limits>

class Ray;
struct Vector;

struct Triangle_Intersection {
    float t = std::numeric_limits<float>::infinity();
    float b1 = 0.0;
    float b2 = 0.0;
};

bool intersect_triangle(const Ray& ray, const Vector& p0, const Vector& p1, const Vector& p2, Triangle_Intersection& intersection);
