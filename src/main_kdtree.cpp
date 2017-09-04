#include "common.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "random.h"
#include "ray_generator.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "vector.h"

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

const bool build_tree = false;

int benchmark_kd_tree(const KdTree& kdtree) {
    Timestamp tx;

    Vector last_hit = (kdtree.GetMeshBounds().min_point + kdtree.GetMeshBounds().max_point) * 0.5f;
    float last_hit_epsilon = 0.0;
    auto ray_generator = Ray_Generator(kdtree.GetMeshBounds());

    int64_t time_ns = 0;

    for (int i = 0; i < benchmark_ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit, last_hit_epsilon);

        Timestamp t2;

        KdTree::Intersection intersection;
        bool hitFound = kdtree.Intersect(ray, intersection);

        time_ns += elapsed_nanoseconds(t2);

        if (hitFound) {
            last_hit = ray.GetPoint(intersection.t);
            last_hit_epsilon = intersection.epsilon;
        }

        if (debug_rays && i < debug_ray_count) {
            if (hitFound)
                printf("%d: found: %s, lastHit: %.14f %.14f %.14f\n", i,
                    hitFound ? "true" : "false", last_hit.x, last_hit.y, last_hit.z);
            else
                printf("%d: found: %s\n", i, hitFound ? "true" : "false");
        }
    }

    double cpu_ghz = get_base_cpu_frequency_ghz();
    double nanoseconds_per_raycast = time_ns / double(benchmark_ray_count);
    int clocks = static_cast<int>(nanoseconds_per_raycast * cpu_ghz);
    printf("Single raycast time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_raycast, clocks);

    //return static_cast<int>(elapsed_milliseconds(t));
    return (int)(time_ns / 1'000'000);
}



void ValidateKdTree(const KdTree& kdTree, int raysCount)
{
    Vector lastHit =
        (kdTree.GetMeshBounds().min_point + kdTree.GetMeshBounds().max_point) * 0.5;
    float lastHitEpsilon = 0.0;
    auto rayGenerator = Ray_Generator(kdTree.GetMeshBounds());

    for (int raysTested = 0; raysTested < raysCount; raysTested++) {
        const Ray ray = rayGenerator.generate_ray(lastHit, lastHitEpsilon);

        KdTree::Intersection kdTreeIntersection;
        bool kdTreeHitFound = kdTree.Intersect(ray, kdTreeIntersection);

        KdTree::Intersection bruteForceIntersection;
        bool bruteForceHitFound = false;

        for (int32_t i = 0; i < kdTree.GetMesh().get_triangle_count(); i++) {
            Triangle triangle = kdTree.GetMesh().get_triangle(i);

            Triangle_Intersection intersection;
            bool hitFound = intersect_triangle(ray, triangle, intersection);

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
            RuntimeError("KdTree traversal error detected");
        }

        if (bruteForceHitFound) {
            lastHit = ray.GetPoint(bruteForceIntersection.t);
            lastHitEpsilon = bruteForceIntersection.epsilon;
        }
    }
}

template <typename T> struct Triangle_Mesh_Selector;

template <>
struct Triangle_Mesh_Selector<Indexed_Triangle_Mesh> {
    Triangle_Mesh_Selector(Indexed_Triangle_Mesh& indexed_mesh) : mesh(indexed_mesh) {}
    Indexed_Triangle_Mesh& mesh;
};

template <>
struct Triangle_Mesh_Selector<Simple_Triangle_Mesh> {
    Triangle_Mesh_Selector(Indexed_Triangle_Mesh& indexed_mesh) : mesh(Simple_Triangle_Mesh::from_indexed_mesh(indexed_mesh)) {}
    Simple_Triangle_Mesh mesh;
};

int main() {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    std::unique_ptr<Indexed_Triangle_Mesh> indexed_mesh = LoadTriangleMesh(model_path);

    Triangle_Mesh_Selector<Triangle_Mesh> mesh_selector(*indexed_mesh);
    Triangle_Mesh& mesh = mesh_selector.mesh;

    if (build_tree) {
        KdTree_Build_Params params;
        Timestamp t;
        KdTree kdtree = build_kdtree(mesh, params);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree build time = %dms\n", time);

        kdtree.SaveToFile("test.kdtree");
        printf("\n");
        return 0;
    }
    auto kdtree = std::unique_ptr<KdTree>(new KdTree(kdtree_path, mesh));

    mesh.print_info();
    kdtree->calculate_stats().Print();
    printf("\n");
    printf("=========================\n");
    printf("shooting rays (kdtree)...\n");
    random_init();

    int timeMsec = benchmark_kd_tree(*kdtree);
    double speed = (benchmark_ray_count / 1000000.0) / (timeMsec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", model_path.c_str(), speed, random_uint32());

    //ValidateKdTree(*kdtree, validation_ray_count);
}
