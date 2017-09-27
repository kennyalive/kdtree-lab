#pragma once

#include "bounding_box.h"
#include "ray.h"
#include "triangle.h"
#include "triangle_mesh.h"
#include "vector.h"
#include <cassert>
#include <cstdint>
#include <vector>

struct KdTree_Stats {
    int64_t nodes_size = 0;
    int64_t triangle_indices_size = 0;

    int32_t node_count = 0;
    int32_t leaf_count = 0;
    int32_t empty_leaf_count = 0;
    int32_t single_triangle_leaf_count = 0;
    int perfect_depth = 0;

    struct Leaf_Stats {
        float average_depth = 0.0f;
        float depth_standard_deviation = 0.0f;
        float average_triangle_count = 0.0f;
    };
    Leaf_Stats not_empty_leaf_stats;
    Leaf_Stats empty_leaf_stats; // empty_leaf_stats.average_triangle_count == 0

    void Print();
};

struct KdNode {
    uint32_t word0;
    uint32_t word1;

    enum : int32_t { maxNodesCount = 0x40000000 }; // max ~ 1 billion nodes
    enum : uint32_t { leafNodeFlags = 3 };

    void InitInteriorNode(int axis, int32_t aboveChild, float split) {
        // 0 - x axis, 1 - y axis, 2 - z axis
        assert(axis >= 0 && axis < 3);
        assert(aboveChild < maxNodesCount);

        word0 = axis | (static_cast<uint32_t>(aboveChild) << 2);
        word1 = *reinterpret_cast<uint32_t*>(&split);
    }

    void InitEmptyLeaf() {
        word0 = leafNodeFlags; // word0 == 3
        word1 = 0;             // not used for empty leaf, just set default value
    }

    void InitLeafWithSingleTriangle(int32_t triangleIndex) {
        word0 = leafNodeFlags | (1 << 2); // word0 == 7
        word1 = static_cast<uint32_t>(triangleIndex);
    }

    void InitLeafWithMultipleTriangles(int32_t numTriangles, int32_t triangleIndicesOffset) {
        assert(numTriangles > 1);
        // word0 == 11, 15, 19, ... (for numTriangles = 2, 3, 4, ...)
        word0 = leafNodeFlags | (static_cast<uint32_t>(numTriangles) << 2);
        word1 = static_cast<uint32_t>(triangleIndicesOffset);
    }

    bool IsLeaf() const {
        return (word0 & leafNodeFlags) == leafNodeFlags;
    }

    bool IsInteriorNode() const {
        return !IsLeaf();
    }

    int32_t GetTrianglesCount() const {
        assert(IsLeaf());
        return static_cast<int32_t>(word0 >> 2);
    }

    int32_t GetIndex() const {
        assert(IsLeaf());
        return static_cast<int32_t>(word1);
    }

    int get_split_axis() const {
        assert(IsInteriorNode());
        return static_cast<int>(word0 & leafNodeFlags);
    }

    float get_split_position() const {
        assert(IsInteriorNode());
        return *reinterpret_cast<const float*>(&word1);
    }

    int32_t get_above_child() const {
        assert(IsInteriorNode());
        return static_cast<int32_t>(word0 >> 2);
    }
};

class KdTree {
public:
    struct Intersection {
        float t = std::numeric_limits<float>::infinity();
        float epsilon = 0.0;
    };

public:
    KdTree(std::vector<KdNode>&& nodes, std::vector<int32_t>&& triangle_indices, const Triangle_Mesh& mesh);
    KdTree(const std::string& fileName, const Triangle_Mesh& mesh);

    void save_to_file(const std::string& fileName) const;

    bool intersect(const Ray& ray, Intersection& intersection) const;

    const Triangle_Mesh& get_mesh() const { return mesh; }
    KdTree_Stats calculate_stats() const;
    std::vector<int32_t> calculate_path_to_node(int32_t node_index) const;

private:
    void intersect_leaf_triangles(const Ray& ray, KdNode leaf, Triangle_Intersection& closestIntersection) const;

private:
    friend class KdTree_Builder;

    enum { max_traversal_depth = 64 };

private:
    const std::vector<KdNode> nodes;
    const std::vector<int32_t> triangle_indices;
    const Triangle_Mesh& mesh;
    const Bounding_Box mesh_bounds;
};
