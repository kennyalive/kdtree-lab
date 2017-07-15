#include "benchmark.h"
#include "common.h"
#include "kdtree.h"
#include "random.h"
#include "triangle.h"
#include "vector.h"

#include <xmmintrin.h> 

#include <embree2/rtcore_ray.h>

namespace {
// x range: [-PI,PI]
float fast_sine(float x) {
    constexpr float PI = 3.14159265358f;
    constexpr float B = 4.0f / PI;
    constexpr float C = -4.0f / (PI * PI);
    constexpr float P = 0.225f;

    float y = B * x + C * x * (x < 0 ? -x : x);
    return P * (y * (y < 0 ? -y : y) - y) + y;
}

// x range: [-PI, PI]
float fast_cosine(float x) {
    constexpr float PI = 3.14159265358f;
    constexpr float B = 4.0f / PI;
    constexpr float C = -4.0f / (PI * PI);
    constexpr float P = 0.225f;

    x = (x > 0) ? -x : x;
    x += PI/2;

    return fast_sine(x);
}

float fast_sqrt(float v) {
    float out;
    __m128 in = _mm_load_ss(&v);
    _mm_store_ss(&out, _mm_mul_ss(in, _mm_rsqrt_ss(in)));
    return out;
}

Vector UniformSampleSphere() {
    constexpr float PI = 3.14159265358f;

    auto u1 = random_float();
    auto u2 = random_float();

    float z = 1.0f - 2.0f * u1;
    float r = fast_sqrt(1.0f - z*z);
    float phi = PI * (-1.0f + 2.0f * u2);
    float x = r * fast_cosine(phi);
    float y = r * fast_sine(phi);
    return Vector(x, y, z);
}

class RayGenerator {
public:
  RayGenerator(const BoundingBox& meshBounds)
  {
    auto diagonal = meshBounds.maxPoint - meshBounds.minPoint;
    double delta = 2.0 * diagonal.Length();

    raysBounds = BoundingBox(meshBounds.minPoint - Vector(delta),
                             meshBounds.maxPoint + Vector(delta));
  }

  Ray GenerateRay(const Vector& lastHit, double lastHitEpsilon) const
  {
    // generate ray origin
    Vector origin;
    origin.x = random_from_range(raysBounds.minPoint.x, raysBounds.maxPoint.x);
    origin.y = random_from_range(raysBounds.minPoint.y, raysBounds.maxPoint.y);
    origin.z = random_from_range(raysBounds.minPoint.z, raysBounds.maxPoint.z);

    const bool useLastHit = random_float() < 0.25f;
    if (useLastHit)
      origin = lastHit;

    // generate ray direction;
    auto direction = UniformSampleSphere();

    auto len = direction.Length();

    if (random_float() < 1.0f / 32.0f && direction.z != 0.0)
      direction.x = direction.y = 0.0;
    else if (random_float() < 1.0f / 32.0f && direction.y != 0.0)
      direction.x = direction.z = 0.0;
    else if (random_float() < 1.0f / 32.0f && direction.x != 0.0)
      direction.y = direction.z = 0.0;
    direction = direction.GetNormalized();

    // initialize ray
    auto ray = Ray(origin, direction);

    if (useLastHit) {
      ray.Advance(lastHitEpsilon);
    }
    else {
      ray.Advance(1e-3);
    }
    return ray;
  }

private:
  BoundingBox raysBounds;
};
} // namespace

int BenchmarkKdTree(const KdTree& kdTree)
{
  Timer timer;

  Vector lastHit =
      (kdTree.GetMeshBounds().minPoint + kdTree.GetMeshBounds().maxPoint) * 0.5;
  double lastHitEpsilon = 0.0;
  auto rayGenerator = RayGenerator(kdTree.GetMeshBounds());

  for (int raysTested = 0; raysTested < benchmarkRaysCount; raysTested++) {
    const Ray ray = rayGenerator.GenerateRay(lastHit, lastHitEpsilon);

    KdTree::Intersection intersection;
    bool hitFound = kdTree.Intersect(ray, intersection);

    if (hitFound) {
      lastHit = ray.GetPoint(intersection.t);
      lastHitEpsilon = intersection.epsilon;
    }

    if (debug_rays && raysTested < debug_ray_count) {
      if (hitFound)
        printf("%d: found: %s, lastHit: %.14f %.14f %.14f\n", raysTested,
               hitFound ? "true" : "false", lastHit.x, lastHit.y, lastHit.z);
      else
        printf("%d: found: %s\n", raysTested, hitFound ? "true" : "false");
    }
  }
  return timer.ElapsedMilliseconds();
}

int BenchmarkEmbree(RTCScene scene, const BoundingBox_f& bounds)
{
    Timer timer;

    Vector lastHit = (bounds.minPoint + bounds.maxPoint) * 0.5;
        
    double lastHitEpsilon = 0.0;
    auto rayGenerator = RayGenerator(BoundingBox(bounds));

    RTCRay rtc_init_ray;
    rtc_init_ray.tnear = 0.0f;
    rtc_init_ray.tfar = std::numeric_limits<float>::infinity();
    rtc_init_ray.geomID = RTC_INVALID_GEOMETRY_ID ;
    rtc_init_ray.primID = RTC_INVALID_GEOMETRY_ID;
    rtc_init_ray.instID = RTC_INVALID_GEOMETRY_ID;
    rtc_init_ray.time = 0.0f;
    rtc_init_ray.mask = -1;

    for (int raysTested = 0; raysTested < benchmarkRaysCount; raysTested++) {
        const Ray ray = rayGenerator.GenerateRay(lastHit, lastHitEpsilon);

        RTCRay rtc_ray;
        memcpy(&rtc_ray, &rtc_init_ray, sizeof(RTCRay));

        rtc_ray.org[0] = static_cast<float>(ray.GetOrigin().x);
        rtc_ray.org[1] = static_cast<float>(ray.GetOrigin().y);
        rtc_ray.org[2] = static_cast<float>(ray.GetOrigin().z);
        rtc_ray.dir[0] = static_cast<float>(ray.GetDirection().x);
        rtc_ray.dir[1] = static_cast<float>(ray.GetDirection().y);
        rtc_ray.dir[2] = static_cast<float>(ray.GetDirection().z);

        rtcIntersect(scene, rtc_ray);

        if (rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID) {
            lastHit = ray.GetPoint(rtc_ray.tfar);
            lastHitEpsilon = 1e-3 * rtc_ray.tfar;
        }

        if (debug_rays && raysTested < debug_ray_count) {
          if (rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID)
            printf("%d: found: %s, lastHit: %.14f %.14f %.14f\n", raysTested,
                rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID ? "true" : "false", lastHit.x, lastHit.y, lastHit.z);
          else
            printf("%d: found: %s\n", raysTested, rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID ? "true" : "false");
        }
    }
    return timer.ElapsedMilliseconds();
}

void ValidateKdTree(const KdTree& kdTree, int raysCount)
{
  Vector lastHit =
      (kdTree.GetMeshBounds().minPoint + kdTree.GetMeshBounds().maxPoint) * 0.5;
  double lastHitEpsilon = 0.0;
  auto rayGenerator = RayGenerator(kdTree.GetMeshBounds());

  for (int raysTested = 0; raysTested < raysCount; raysTested++) {
    const Ray ray = rayGenerator.GenerateRay(lastHit, lastHitEpsilon);

    KdTree::Intersection kdTreeIntersection;
    bool kdTreeHitFound = kdTree.Intersect(ray, kdTreeIntersection);

    KdTree::Intersection bruteForceIntersection;
    bool bruteForceHitFound = false;

    for (int32_t i = 0; i < kdTree.GetMesh().GetTriangleCount(); i++) {
      const auto& p = kdTree.GetMesh().triangles[i].points;

      Triangle triangle = {
          {Vector(kdTree.GetMesh().vertices[p[0].vertexIndex]),
           Vector(kdTree.GetMesh().vertices[p[1].vertexIndex]),
           Vector(kdTree.GetMesh().vertices[p[2].vertexIndex])}};

      Triangle::Intersection intersection;
      bool hitFound = IntersectTriangle(ray, triangle, intersection);

      if (hitFound && intersection.t < bruteForceIntersection.t) {
        bruteForceIntersection.t = intersection.t;
        bruteForceHitFound = true;
      }
    }

    if (kdTreeHitFound != bruteForceHitFound ||
        kdTreeIntersection.t != bruteForceIntersection.t) {
      const auto& o = ray.GetOrigin();
      const auto& d = ray.GetDirection();
      printf("KdTree accelerator test failure:\n"
             "KdTree hit: %s\n"
             "actual hit: %s\n"
             "KdTree T %.16g [%a]\n"
             "actual T %.16g [%a]\n"
             "ray origin: (%a, %a, %a)\n"
             "ray direction: (%a, %a, %a)\n",
             kdTreeHitFound ? "true" : "false",
             bruteForceHitFound ? "true" : "false", kdTreeIntersection.t,
             kdTreeIntersection.t, bruteForceIntersection.t,
             bruteForceIntersection.t, o.x, o.y, o.z, d.x, d.y, d.z);
      ValidationError("KdTree traversal error detected");
    }

    if (bruteForceHitFound) {
      lastHit = ray.GetPoint(bruteForceIntersection.t);
      lastHitEpsilon = bruteForceIntersection.epsilon;
    }
  }
}
