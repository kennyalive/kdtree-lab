#include "ray.h"
#include "triangle.h"

//
// M�ller-Trumbore triangle intersection algorithm.
// http://www.graphics.cornell.edu/pubs/1997/MT97.pdf
//
bool intersect_triangle(const Ray& ray, const Triangle& triangle, Triangle_Intersection& intersection) {
    Vector edge1 = triangle[1] - triangle[0];
    Vector edge2 = triangle[2] - triangle[0];

    Vector p = cross(ray.GetDirection(), edge2);
    float divisor = dot(edge1, p);

    // todo: do we need to check against epsilon for better numeric stability?
    if (divisor == 0.0)
        return false;

    const float invDivisor = 1.0f / divisor;

    // compute barycentric coordinate b1
    Vector t = ray.GetOrigin() - triangle[0];
    float b1 = invDivisor * dot(t, p);
    if (b1 < 0.0 || b1 > 1.0)
        return false;

    // compute barycentric coordnate b2
    Vector q = cross(t, edge1);
    float b2 = invDivisor * dot(ray.GetDirection(), q);
    if (b2 < 0.0 || b1 + b2 > 1.0)
        return false;

    // compute distance from ray origin to intersection point
    float distance = invDivisor * dot(edge2, q);
    if (distance < 0.0)
        return false;

    intersection.t = distance;
    intersection.b1 = b1;
    intersection.b2 = b2;
    return true;
}
