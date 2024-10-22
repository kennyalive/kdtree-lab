#include "common.h"
#include "kdtree.h"
#include "triangle.h"

#include <cassert>
#include <fstream>
#include <map>
#include <numeric>

KdTree::KdTree(std::vector<KdNode>&& nodes, std::vector<int32_t>&& triangle_indices, const Triangle_Mesh& mesh)
: nodes(std::move(nodes))
, triangle_indices(std::move(triangle_indices))
, mesh(mesh)
, mesh_bounds(mesh.get_bounds())
{
}

KdTree::KdTree(const std::string& file_name, const Triangle_Mesh& mesh)
: mesh(mesh)
, mesh_bounds(mesh.get_bounds())
{
    std::ifstream file(file_name, std::ios_base::in | std::ios_base::binary);
    if (!file)
        error("failed to open kdTree file: " + file_name);

    // read nodes
    int32_t nodesCount;
    file.read(reinterpret_cast<char*>(&nodesCount), 4);
    if (!file)
        error("failed to read nodes count: " + file_name);

    auto& mutableNodes = const_cast<std::vector<KdNode>&>(nodes);
    mutableNodes.resize(nodesCount);

    auto nodesBytesCount = nodesCount * sizeof(KdNode);
    file.read(reinterpret_cast<char*>(mutableNodes.data()), nodesBytesCount);
    if (!file)
        error("failed to read kdTree nodes: " + file_name);

    // read triangle indices
    int32_t indicesCount;
    file.read(reinterpret_cast<char*>(&indicesCount), 4);
    if (!file)
        error("failed to read triangle indices count: " + file_name);

    auto& mutableIndices = const_cast<std::vector<int32_t>&>(triangle_indices);
    mutableIndices.resize(indicesCount);

    auto indicesBytesCount = indicesCount * 4;
    file.read(reinterpret_cast<char*>(mutableIndices.data()), indicesBytesCount);
    if (!file)
        error("failed to read kdTree triangle indices: " + file_name);
}

void KdTree::save_to_file(const std::string& file_name) const
{
    std::ofstream file(file_name, std::ios_base::out | std::ios_base::binary);
    if (!file)
        error("failed to open kdTree file for writing: " + file_name);

    // write nodes
    int32_t nodesCount = static_cast<int32_t>(nodes.size());
    file.write(reinterpret_cast<const char*>(&nodesCount), 4);

    auto nodesBytesCount = nodesCount * sizeof(KdNode);
    file.write(reinterpret_cast<const char*>(nodes.data()), nodesBytesCount);
    if (!file)
        error("failed to write kdTree nodes: " + file_name);

    // write triangle indices
    int32_t indicesCount = static_cast<int32_t>(triangle_indices.size());
    file.write(reinterpret_cast<const char*>(&indicesCount), 4);

    auto indicesBytesCount = indicesCount * 4;
    file.write(reinterpret_cast<const char*>(triangle_indices.data()),
        indicesBytesCount);
    if (!file)
        error("failed to write kdTree triangle indices: " + file_name);
}

bool KdTree::intersect(const Ray& ray, Intersection& intersection) const
{
    auto bounds_intersection = mesh_bounds.intersect(ray);
    if (!bounds_intersection.found)
        return false;

    struct Traversal_Info {
        const KdNode* node;
        float tMin;
        float tMax;
    };
    Traversal_Info traversal_stack[max_traversal_depth];
    int traversal_stack_size = 0;

    float t_min = bounds_intersection.t0;
    float t_max = bounds_intersection.t1;

    Triangle_Intersection closest_intersection;
    auto node = &nodes[0];

    while (t_min < closest_intersection.t) {
        if (node->is_interior_node()) {
            int axis = node->get_split_axis();

            float distance_to_split_plane = node->get_split_position() - ray.GetOrigin()[axis];

            auto belowChild = node + 1;
            auto aboveChild = &nodes[node->get_above_child()];

            if (distance_to_split_plane != 0.0) { // general case
                const KdNode *firstChild, *secondChild;

                if (distance_to_split_plane > 0.0) {
                    firstChild = belowChild;
                    secondChild = aboveChild;
                }
                else {
                    firstChild = aboveChild;
                    secondChild = belowChild;
                }

                // t_split != 0 (since distance_to_split_plane != 0)
                float t_split = distance_to_split_plane * ray.GetInvDirection()[axis];

                if (t_split >= t_max || t_split < 0.0)
                    node = firstChild;
                else if (t_split <= t_min)
                    node = secondChild;
                else { // t_min < t_split < t_max
                    assert(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size++] = {secondChild, t_split, t_max};
                    node = firstChild;
                    t_max = t_split;
                }
            }
            else { // special case, distanceToSplitPlane == 0.0
                if (ray.GetDirection()[axis] > 0.0) {
                    if (t_min > 0.0)
                        node = aboveChild;
                    else { // t_min == 0.0
                        assert(traversal_stack_size < max_traversal_depth);
                        traversal_stack[traversal_stack_size++] = {aboveChild, 0.0, t_max};
                        // check single point [0.0, 0.0]
                        node = belowChild;
                        t_max = 0.0;
                    }
                }
                else if (ray.GetDirection()[axis] < 0.0) {
                    if (t_min > 0.0)
                        node = belowChild;
                    else { // t_min == 0.0
                        assert(traversal_stack_size < max_traversal_depth);
                        traversal_stack[traversal_stack_size++] = {belowChild, 0.0, t_max};
                        // check single point [0.0, 0.0]
                        node = aboveChild;
                        t_max = 0.0;
                    }
                }
                else { // ray.direction[axis] == 0.0
                    // for both nodes check [t_min, t_max] range
                    assert(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size++] = {aboveChild, t_min, t_max};
                    node = belowChild;
                }
            }
        }
        else { // leaf node
            intersect_leaf_triangles(ray, *node, closest_intersection);

            if (traversal_stack_size == 0)
                break;

            // Almost correct implementation is just: --traversal_stack_size.
            // We need to scan the entire stack to handle the case when distance_to_split_plane == 0.0 && ray.direction[axis] == 0.0.
            do {
                --traversal_stack_size;
            } while (traversal_stack_size > 0 && traversal_stack[traversal_stack_size].tMin >= closest_intersection.t);

            node = traversal_stack[traversal_stack_size].node;
            t_min = traversal_stack[traversal_stack_size].tMin;
            t_max = traversal_stack[traversal_stack_size].tMax;
        }
    } // while (t_min < closest_intersection.t)

    if (closest_intersection.t == std::numeric_limits<float>::infinity())
        return false;

    intersection.t = closest_intersection.t;
    intersection.epsilon = closest_intersection.t * 1e-3f;
    return true;
}

void KdTree::intersect_leaf_triangles(const Ray& ray, KdNode leaf, Triangle_Intersection& closestIntersection) const
{
    if (leaf.get_triangle_count() == 1) {
        Triangle t = mesh.get_triangle(leaf.get_index());
        Triangle_Intersection intersection;
        bool hitFound = intersect_triangle(ray, t, intersection);
        if (hitFound && intersection.t < closestIntersection.t) {
            closestIntersection = intersection;
        }
    }
    else {
        for (int32_t i = 0; i < leaf.get_triangle_count(); i++) {
            int32_t triangleIndex = triangle_indices[leaf.get_index() + i];
            Triangle triangle = mesh.get_triangle(triangleIndex);
            Triangle_Intersection intersection;
            bool hitFound = intersect_triangle(ray, triangle, intersection);
            if (hitFound && intersection.t < closestIntersection.t) {
                closestIntersection = intersection;
            }
        }
    }
}

KdTree_Stats KdTree::calculate_stats() const
{
    KdTree_Stats stats;

    stats.nodes_size = nodes.size() * sizeof(KdNode);
    stats.triangle_indices_size = triangle_indices.size() * sizeof(triangle_indices[0]);
    stats.node_count = static_cast<int32_t>(nodes.size());

    int64_t triangle_per_leaf_accumulated = 0;

    for (auto node : nodes) {
        if (node.is_leaf()) {
            stats.leaf_count++;
            triangle_per_leaf_accumulated += node.get_triangle_count();

            if (node.get_triangle_count() == 0)
                stats.empty_leaf_count++;
            else if (node.get_triangle_count() == 1)
                stats.single_triangle_leaf_count++;
        }
    }

    auto not_empty_leaf_count = stats.leaf_count - stats.empty_leaf_count;

    stats.perfect_depth = static_cast<int>(std::ceil(std::log2(stats.leaf_count)));
    stats.not_empty_leaf_stats.average_triangle_count = float(double(triangle_per_leaf_accumulated) / not_empty_leaf_count);

    // Compute depth of each leaf node.
    std::vector<uint8_t> not_empty_leaf_depth_values;
    std::vector<uint8_t> empty_leaf_depth_values;

    struct Depth_Info {
        int32_t node_index = -1;
        uint8_t depth = -1;
    };
    std::vector<Depth_Info> depth_info{ Depth_Info{0, 0} };

    size_t i = 0;
    while (i < depth_info.size()) {
        int32_t node_index = depth_info[i].node_index;
        uint8_t depth = depth_info[i].depth;

        if (nodes[node_index].is_leaf()) {
            if (nodes[node_index].get_triangle_count() > 0)
                not_empty_leaf_depth_values.push_back(depth);
            else
                empty_leaf_depth_values.push_back(depth);
        }
        else {
            int32_t below_child_index = node_index + 1;
            depth_info.push_back({ below_child_index, uint8_t(depth + 1) });

            int32_t above_child_index = nodes[node_index].get_above_child();
            depth_info.push_back({ above_child_index, uint8_t(depth + 1) });
        }
        i++;
    }

    int64_t not_empty_leaf_depth_accumulated = std::accumulate(not_empty_leaf_depth_values.cbegin(), not_empty_leaf_depth_values.cend(), int64_t(0));
    stats.not_empty_leaf_stats.average_depth = float(double(not_empty_leaf_depth_accumulated) / not_empty_leaf_count);

    double accum = 0.0;
    for (auto depth : not_empty_leaf_depth_values) {
        auto diff = depth - stats.not_empty_leaf_stats.average_depth;
        accum += diff * diff;
    }
    stats.not_empty_leaf_stats.depth_standard_deviation = float(std::sqrt(accum / not_empty_leaf_count));

    int64_t empty_leaf_depth_accumulated = std::accumulate(empty_leaf_depth_values.cbegin(), empty_leaf_depth_values.cend(), int64_t(0));
    stats.empty_leaf_stats.average_depth = float(double(empty_leaf_depth_accumulated) / stats.empty_leaf_count);

    accum = 0.0f;
    for (auto depth : empty_leaf_depth_values) {
        auto diff = depth - stats.empty_leaf_stats.average_depth;
        accum += diff * diff;
    }
    stats.empty_leaf_stats.depth_standard_deviation = float(std::sqrt(accum / stats.empty_leaf_count));

    return stats;
}

std::vector<int32_t> KdTree::calculate_path_to_node(int32_t node_index) const
{
    assert(node_index >= 0 && node_index < nodes.size());

    std::map<int32_t, int32_t> parent_map;

    for (int32_t i = 0; i < int32_t(nodes.size()); i++) {
        auto node = nodes[i];
        if (node.is_interior_node()) {
            int32_t below_child = i + 1;
            int32_t above_child = node.get_above_child();
            parent_map[below_child] = i;
            parent_map[above_child] = i;
        }
    }

    std::vector<int32_t> path { node_index };
    auto it = parent_map.find(node_index);
    while (it != parent_map.cend()) {
        path.push_back(it->second);
        it = parent_map.find(it->second);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void KdTree_Stats::print()
{
    printf("[memory consumption]\n");
    printf("nodes_size = %zdK\n", nodes_size / 1024);
    printf("triangle_indices_size = %zdK\n", triangle_indices_size / 1024);
    printf("\n");

    printf("[kdtree general]\n");
    printf("node_count = %d\n", node_count);
    printf("leaf_count = %d\n", leaf_count);
    printf("empty_leaf_count = %d\n", empty_leaf_count);
    printf("single_triangle_leaf_count = %d\n", single_triangle_leaf_count);
    printf("perfect_depth = %d\n", perfect_depth);
    printf("\n");

    printf("[kdtree not empty leaves]\n");
    printf("average_depth = %.2f\n", not_empty_leaf_stats.average_depth);
    printf("depth_standard_deviation = %.2f\n", not_empty_leaf_stats.depth_standard_deviation);
    printf("average_triangle_count = %.2f\n", not_empty_leaf_stats.average_triangle_count);
    printf("\n");

    printf("[kdtree empty leaves]\n");
    printf("average_depth = %.2f\n", empty_leaf_stats.average_depth);
    printf("depth_standard_deviation = %.2f\n", empty_leaf_stats.depth_standard_deviation);
}
