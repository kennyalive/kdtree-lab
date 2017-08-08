#include "benchmark.h"
#include "common.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "random.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "vector.h"
#include <embree2/rtcore.h>
#include <pmmintrin.h>
#include <string>
#include <vector>

const std::string model_path = "data/teapot.stl";
const std::string kdtree_path = "data/teapot.kdtree";
const int validation_ray_count = 32768;

//const std::string model_path = "data/bunny.stl";
//const std::string kdtree_path = "data/bunny.kdtree";
//const int validation_ray_count = 64;

//const std::string model_path = "data/dragon.stl";
//const std::string kdtree_path = "data/dragon.kdtree";
//const int validation_ray_count = 32;

std::unique_ptr<TriangleMesh> mesh = LoadTriangleMesh(model_path);

void main_kdtree() {
    auto kdtree = std::unique_ptr<KdTree>(new KdTree(kdtree_path, *mesh));

    printf("shooting rays (kdtree)...\n");
    random_init();

    int timeMsec = BenchmarkKdTree(*kdtree);
    double speed = (benchmarkRaysCount / 1000000.0) / (timeMsec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", StripExtension(GetFileName(model_path)).c_str(), speed, random_uint32());

    //ValidateKdTree(*kdtree, validation_ray_count);
}

void main_embree() {
    RTCDevice device = rtcNewDevice(NULL);

    struct Vertex { float x, y, z, a; };
    struct Triangle { int32_t v0, v1, v2; };

    auto scene = rtcDeviceNewScene(device, RTC_SCENE_STATIC, RTC_INTERSECT1);
    unsigned geom_id = rtcNewTriangleMesh2(scene, RTC_GEOMETRY_STATIC, mesh->GetTriangleCount(), mesh->GetVertexCount(), 1);

    auto vertices = static_cast<Vertex*>(rtcMapBuffer(scene, geom_id, RTC_VERTEX_BUFFER));
    for (int32_t k = 0; k < mesh->GetVertexCount(); k++, vertices++) {
        vertices->x = mesh->vertices[k].x;
        vertices->y = mesh->vertices[k].y;
        vertices->z = mesh->vertices[k].z;
    }
    rtcUnmapBuffer(scene, geom_id, RTC_VERTEX_BUFFER);

    auto triangles = static_cast<Triangle*>(rtcMapBuffer(scene, geom_id, RTC_INDEX_BUFFER));
    for (int k = 0; k < mesh->GetTriangleCount(); k++, triangles++) {
        triangles->v0 = mesh->triangles[k].points[0].vertexIndex;
        triangles->v1 = mesh->triangles[k].points[1].vertexIndex;
        triangles->v2 = mesh->triangles[k].points[2].vertexIndex;
    }
    rtcUnmapBuffer(scene, geom_id, RTC_INDEX_BUFFER);
    rtcCommit(scene);

    printf("shooting rays (embree)...\n");
    random_init();

    int timeMsec = BenchmarkEmbree(scene, mesh->GetBounds());
    double speed = (benchmarkRaysCount / 1000000.0) / (timeMsec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", StripExtension(GetFileName(model_path)).c_str(), speed, random_uint32());

    rtcDeleteScene(scene);
    rtcDeleteDevice(device);
}

const bool build_tree = false;

int main() {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    if (build_tree) {
        KdTreeBuilder::BuildParams params;
        KdTreeBuilder builder(*mesh, params);

        Timer timer;
        KdTree kdtree = builder.BuildTree();
        int time = timer.ElapsedMilliseconds();
        printf("KdTree build time = %dms\n", time);

        builder.GetBuildStats().Print();
        kdtree.PrintInfo();
        kdtree.SaveToFile("test.kdtree");
        return 0;
    }

    main_kdtree();
//    main_embree();
}
