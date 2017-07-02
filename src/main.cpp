#include "benchmark.h"
#include "common.h"
#include "kdtree.h"
#include "random.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "vector.h"
#include <embree2/rtcore.h>
#include <pmmintrin.h>
#include <string>
#include <vector>

const std::string all_models[] = { "data/teapot.stl", "data/bunny.stl", "data/dragon.stl"};
const std::string all_kd_trees[] = { "data/teapot.kdtree", "data/bunny.kdtree", "data/dragon.kdtree" };

// Selects subset from all_models array to work with.
const std::array<int, 3> model_indices { 0, 1, 2 };

const std::vector<std::string> models = []()
{
    std::vector<std::string> result;
    for (auto i : model_indices)
        result.push_back(all_models[i]);
    return result;
} ();

const std::vector<std::unique_ptr<TriangleMesh>> meshes = []()
{
    std::vector<std::unique_ptr<TriangleMesh>> result;
    for (int i = 0; i < models.size(); i++)
        result.push_back(LoadTriangleMesh(models[i]));
    return result;
} ();

void main_kdtree() {
    std::vector<std::unique_ptr<KdTree>> kdTrees;
    for (size_t i = 0; i < model_indices.size(); i++) {
        kdTrees.push_back(std::unique_ptr<KdTree>(new KdTree(all_kd_trees[model_indices[i]], *meshes[i])));
    }

    printf("shooting rays (kdtree)...\n");
    InitRNG(5489UL);
    for (size_t i = 0; i < models.size(); i++) {
        int timeMsec = BenchmarkKdTree(*kdTrees[i]);
        double speed = (benchmarkRaysCount / 1000000.0) / (timeMsec / 1000.0);
        printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", StripExtension(GetFileName(models[i])).c_str(), speed, RandUint32());

    }

    const int raysCount[3] = {32768, 64, 32};
    for (int i = 0; i < models.size(); i++) {
      ValidateKdTree(*kdTrees[i], raysCount[model_indices[i]]);
    }
}

void main_embree() {
    RTCDevice device = rtcNewDevice(NULL);

    struct Vertex { float x, y, z, a; };
    struct Triangle { int32_t v0, v1, v2; };
    RTCScene scenes[3];

    for (int i = 0; i < models.size(); i++) {
        scenes[i] = rtcDeviceNewScene(device, RTC_SCENE_STATIC, RTC_INTERSECT1);
        unsigned geom_id = rtcNewTriangleMesh2(scenes[i], RTC_GEOMETRY_STATIC, meshes[i]->GetTriangleCount(), meshes[i]->GetVertexCount(), 1);

        auto vertices = static_cast<Vertex*>(rtcMapBuffer(scenes[i], geom_id, RTC_VERTEX_BUFFER));
        for (int32_t k = 0; k < meshes[i]->GetVertexCount(); k++, vertices++) {
            vertices->x = meshes[i]->vertices[k].x;
            vertices->y = meshes[i]->vertices[k].y;
            vertices->z = meshes[i]->vertices[k].z;
        }
        rtcUnmapBuffer(scenes[i], geom_id, RTC_VERTEX_BUFFER);

        auto triangles = static_cast<Triangle*>(rtcMapBuffer(scenes[i], geom_id, RTC_INDEX_BUFFER));
        for (int k = 0; k < meshes[i]->GetTriangleCount(); k++, triangles++) {
            triangles->v0 = meshes[i]->triangles[k].points[0].vertexIndex;
            triangles->v1 = meshes[i]->triangles[k].points[1].vertexIndex;
            triangles->v2 = meshes[i]->triangles[k].points[2].vertexIndex;
        }
        rtcUnmapBuffer(scenes[i], geom_id, RTC_INDEX_BUFFER);
        rtcCommit(scenes[i]);
    }

    printf("shooting rays (embree)...\n");
    InitRNG(5489UL);
    for (size_t i = 0; i < models.size(); i++) {
        int timeMsec = BenchmarkEmbree(scenes[i], meshes[i]->GetBounds());
        double speed = (benchmarkRaysCount / 1000000.0) / (timeMsec / 1000.0);
        printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", StripExtension(GetFileName(models[i])).c_str(), speed, RandUint32());
    }

    for (size_t i = 0; i < models.size(); i++) {
        rtcDeleteScene(scenes[i]);
    }
    rtcDeleteDevice(device);
}

int main() {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    main_kdtree();
    main_embree();
}
