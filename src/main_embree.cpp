#include "common.h"
#include "random.h"
#include "ray_generator.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "vector.h"

#include <embree2/rtcore.h> 
#include <embree2/rtcore_ray.h>
#include <pmmintrin.h>

#include <string>
#include <vector>

const std::string model_path = "data/soccer_ball.stl";
const std::string kdtree_path = "data/soccer_ball.kdtree";
const int validation_ray_count = 32768;

//const std::string model_path = "data/teapot.stl";
//const std::string kdtree_path = "data/teapot.kdtree";
//const int validation_ray_count = 32768;

//const std::string model_path = "data/bunny.stl";
//const std::string kdtree_path = "data/bunny.kdtree";
//const int validation_ray_count = 64;

//const std::string model_path = "data/dragon.stl";
//const std::string kdtree_path = "data/dragon.kdtree";
//const int validation_ray_count = 32;

int benchmark_embree(RTCScene scene, const Bounding_Box& bounds) {
    Timestamp t;

    Vector lastHit = (bounds.min_point + bounds.max_point) * 0.5;

    float lastHitEpsilon = 0.0;
    auto rayGenerator = Ray_Generator(Bounding_Box(bounds));

    RTCRay rtc_init_ray;
    rtc_init_ray.tnear = 0.0f;
    rtc_init_ray.tfar = std::numeric_limits<float>::infinity();
    rtc_init_ray.geomID = RTC_INVALID_GEOMETRY_ID;
    rtc_init_ray.primID = RTC_INVALID_GEOMETRY_ID;
    rtc_init_ray.instID = RTC_INVALID_GEOMETRY_ID;
    rtc_init_ray.time = 0.0f;
    rtc_init_ray.mask = -1;

    int64_t time_ns = 0;

    for (int raysTested = 0; raysTested < benchmark_ray_count; raysTested++) {
        const Ray ray = rayGenerator.generate_ray(lastHit, lastHitEpsilon);

        Timestamp t2;

        RTCRay rtc_ray;
        memcpy(&rtc_ray, &rtc_init_ray, sizeof(RTCRay));

        rtc_ray.org[0] = static_cast<float>(ray.GetOrigin().x);
        rtc_ray.org[1] = static_cast<float>(ray.GetOrigin().y);
        rtc_ray.org[2] = static_cast<float>(ray.GetOrigin().z);
        rtc_ray.dir[0] = static_cast<float>(ray.GetDirection().x);
        rtc_ray.dir[1] = static_cast<float>(ray.GetDirection().y);
        rtc_ray.dir[2] = static_cast<float>(ray.GetDirection().z);

        rtcIntersect(scene, rtc_ray);

        time_ns += elapsed_nanoseconds(t2);

        if (rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID) {
            lastHit = ray.GetPoint(rtc_ray.tfar);
            lastHitEpsilon = 1e-3f * rtc_ray.tfar;
        }

        if (debug_rays && raysTested < debug_ray_count) {
            if (rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID)
                printf("%d: found: %s, lastHit: %.14f %.14f %.14f\n", raysTested,
                    rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID ? "true" : "false", lastHit.x, lastHit.y, lastHit.z);
            else
                printf("%d: found: %s\n", raysTested, rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID ? "true" : "false");
        }
    }

    double cpu_ghz = get_base_cpu_frequency_ghz();
    double nanoseconds_per_raycast = time_ns / double(benchmark_ray_count);
    int clocks = static_cast<int>(nanoseconds_per_raycast * cpu_ghz);
    printf("Single raycast time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_raycast, clocks);

    //return int(elapsed_milliseconds(t));
    return (int)(time_ns / 1'000'000);
}

int main() {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    std::unique_ptr<Indexed_Triangle_Mesh> mesh = LoadTriangleMesh(model_path);
    
    RTCDevice device = rtcNewDevice(NULL);

    struct Vertex { float x, y, z, a; };
    struct Triangle { int32_t v0, v1, v2; };

    auto scene = rtcDeviceNewScene(device, RTC_SCENE_STATIC, RTC_INTERSECT1);
    unsigned geom_id = rtcNewTriangleMesh2(scene, RTC_GEOMETRY_STATIC, mesh->get_triangle_count(), mesh->get_vertex_count(), 1);

    auto vertices = static_cast<Vertex*>(rtcMapBuffer(scene, geom_id, RTC_VERTEX_BUFFER));
    for (int32_t k = 0; k < mesh->get_vertex_count(); k++, vertices++) {
        vertices->x = mesh->vertices[k].x;
        vertices->y = mesh->vertices[k].y;
        vertices->z = mesh->vertices[k].z;
    }
    rtcUnmapBuffer(scene, geom_id, RTC_VERTEX_BUFFER);

    auto triangles = static_cast<Triangle*>(rtcMapBuffer(scene, geom_id, RTC_INDEX_BUFFER));
    for (int k = 0; k < mesh->get_triangle_count(); k++, triangles++) {
        triangles->v0 = mesh->face_indices[k][0];
        triangles->v1 = mesh->face_indices[k][1];
        triangles->v2 = mesh->face_indices[k][2];
    }
    rtcUnmapBuffer(scene, geom_id, RTC_INDEX_BUFFER);
    rtcCommit(scene);

    printf("shooting rays (embree)...\n");
    random_init();

    int timeMsec = benchmark_embree(scene, mesh->get_bounds());
    double speed = (benchmark_ray_count / 1000000.0) / (timeMsec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", model_path.c_str(), speed, random_uint32());

    rtcDeleteScene(scene);
    rtcDeleteDevice(device);
}
