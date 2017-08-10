#include "common.h"
#include "kdtree.h"
#include "triangle.h"

#include <cassert>
#include <fstream>
#include <numeric>

KdTree::KdTree(std::vector<Node>&& nodes,
               std::vector<int32_t>&& triangleIndices, const TriangleMesh& mesh)
: nodes(std::move(nodes))
, triangleIndices(std::move(triangleIndices))
, mesh(mesh)
, meshBounds(mesh.GetBounds())
{
}

KdTree::KdTree(const std::string& fileName, const TriangleMesh& mesh)
: mesh(mesh)
, meshBounds(mesh.GetBounds())
{
  std::ifstream file(fileName, std::ios_base::in | std::ios_base::binary);
  if (!file)
    RuntimeError("failed to open kdTree file: " + fileName);

  // read nodes
  int32_t nodesCount;
  file.read(reinterpret_cast<char*>(&nodesCount), 4);
  if (!file)
    RuntimeError("failed to read nodes count: " + fileName);

  auto& mutableNodes = const_cast<std::vector<Node>&>(nodes);
  mutableNodes.resize(nodesCount);

  auto nodesBytesCount = nodesCount * sizeof(Node);
  file.read(reinterpret_cast<char*>(mutableNodes.data()), nodesBytesCount);
  if (!file)
    RuntimeError("failed to read kdTree nodes: " + fileName);

  // read triangle indices
  int32_t indicesCount;
  file.read(reinterpret_cast<char*>(&indicesCount), 4);
  if (!file)
    RuntimeError("failed to read triangle indices count: " + fileName);

  auto& mutableIndices = const_cast<std::vector<int32_t>&>(triangleIndices);
  mutableIndices.resize(indicesCount);

  auto indicesBytesCount = indicesCount * 4;
  file.read(reinterpret_cast<char*>(mutableIndices.data()), indicesBytesCount);
  if (!file)
    RuntimeError("failed to read kdTree triangle indices: " + fileName);
}

void KdTree::SaveToFile(const std::string& fileName) const
{
  std::ofstream file(fileName, std::ios_base::out | std::ios_base::binary);
  if (!file)
    RuntimeError("failed to open kdTree file for writing: " + fileName);

  // write nodes
  int32_t nodesCount = static_cast<int32_t>(nodes.size());
  file.write(reinterpret_cast<const char*>(&nodesCount), 4);

  auto nodesBytesCount = nodesCount * sizeof(Node);
  file.write(reinterpret_cast<const char*>(nodes.data()), nodesBytesCount);
  if (!file)
    RuntimeError("failed to write kdTree nodes: " + fileName);

  // write triangle indices
  int32_t indicesCount = static_cast<int32_t>(triangleIndices.size());
  file.write(reinterpret_cast<const char*>(&indicesCount), 4);

  auto indicesBytesCount = indicesCount * 4;
  file.write(reinterpret_cast<const char*>(triangleIndices.data()),
             indicesBytesCount);
  if (!file)
    RuntimeError("failed to write kdTree triangle indices: " + fileName);
}

void KdTree::PrintInfo() {
    auto nodes_size = nodes.size() * sizeof(Node);
    printf("Nodes size: %zdK\n", nodes_size / 1024);
    auto indices_size = triangleIndices.size() * sizeof(triangleIndices[0]);
    printf("Triangle indices size: %zdK\n", indices_size / 1024);

    printf("Total size = %zdK\n", (nodes_size + indices_size) / 1024);

}

bool KdTree::Intersect(const Ray& ray, Intersection& intersection) const
{
  auto boundsIntersection = meshBounds.intersect(ray);
  if (!boundsIntersection.found)
    return false;

  struct TraversalInfo {
    const Node* node;
    float tMin;
    float tMax;
  };
  TraversalInfo traversalStack[maxTraversalDepth];
  int traversalStackSize = 0;

  float tMin = boundsIntersection.t0;
  float tMax = boundsIntersection.t1;

  Triangle_Intersection closest_intersection;
  auto node = &nodes[0];

  while (closest_intersection.t > tMin) {
    if (node->IsInteriorNode()) {
      int axis = node->GetSplitAxis();

      float distanceToSplitPlane =
          node->GetSplitPosition() - ray.GetOrigin()[axis];

      auto belowChild = node + 1;
      auto aboveChild = &nodes[node->GetAboveChild()];

      if (distanceToSplitPlane != 0.0) { // general case
        const Node *firstChild, *secondChild;

        if (distanceToSplitPlane > 0.0) {
          firstChild = belowChild;
          secondChild = aboveChild;
        }
        else {
          firstChild = aboveChild;
          secondChild = belowChild;
        }

        // tSplit != 0 (since distanceToSplitPlane != 0)
        float tSplit = distanceToSplitPlane * ray.GetInvDirection()[axis];
        if (tSplit >= tMax || tSplit < 0.0)
          node = firstChild;
        else if (tSplit <= tMin)
          node = secondChild;
        else { // tMin < tSplit < tMax
          assert(traversalStackSize < maxTraversalDepth);
          traversalStack[traversalStackSize++] = {secondChild, tSplit, tMax};
          node = firstChild;
          tMax = tSplit;
        }
      }
      else { // special case, distanceToSplitPlane == 0.0
        if (ray.GetDirection()[axis] > 0.0) {
          if (tMin > 0.0)
            node = aboveChild;
          else { // tMin == 0.0
            assert(traversalStackSize < maxTraversalDepth);
            traversalStack[traversalStackSize++] = {aboveChild, 0.0, tMax};
            // check single point [0.0, 0.0]
            node = belowChild;
            tMax = 0.0;
          }
        }
        else if (ray.GetDirection()[axis] < 0.0) {
          if (tMin > 0.0)
            node = belowChild;
          else { // tMin == 0.0
            assert(traversalStackSize < maxTraversalDepth);
            traversalStack[traversalStackSize++] = {belowChild, 0.0, tMax};
            // check single point [0.0, 0.0]
            node = aboveChild;
            tMax = 0.0;
          }
        }
        else { // ray.direction[axis] == 0.0
          // for both nodes check [tMin, tMax] range
          assert(traversalStackSize < maxTraversalDepth);
          traversalStack[traversalStackSize++] = {aboveChild, tMin, tMax};
          node = belowChild;
        }
      }
    }
    else { // leaf node
      IntersectLeafTriangles(ray, *node, closest_intersection);

      if (traversalStackSize == 0)
        break;

      --traversalStackSize;
      node = traversalStack[traversalStackSize].node;
      tMin = traversalStack[traversalStackSize].tMin;
      tMax = traversalStack[traversalStackSize].tMax;
    }
  } // while (closestIntersection.t > tMin)

  if (closest_intersection.t == std::numeric_limits<float>::infinity())
    return false;

  intersection.t = closest_intersection.t;
  intersection.epsilon = closest_intersection.t * 1e-3f;
  return true;
}

void KdTree::IntersectLeafTriangles(const Ray& ray, Node leaf, Triangle_Intersection& closestIntersection) const
{
  if (leaf.GetTrianglesCount() == 1) {
    const auto& p = mesh.triangles[leaf.GetIndex()].points;

    auto& p0 = mesh.vertices[p[0].vertexIndex];
    auto& p1 = mesh.vertices[p[1].vertexIndex];
    auto& p2 = mesh.vertices[p[2].vertexIndex];

    Triangle_Intersection intersection;
    bool hitFound = intersect_triangle(ray, p0, p1, p2, intersection);
    if (hitFound && intersection.t < closestIntersection.t) {
      closestIntersection = intersection;
    }
  }
  else {
    for (int32_t i = 0; i < leaf.GetTrianglesCount(); i++) {
      int32_t triangleIndex = triangleIndices[leaf.GetIndex() + i];
      const auto& p = mesh.triangles[triangleIndex].points;

      auto& p0 = mesh.vertices[p[0].vertexIndex];
      auto& p1 = mesh.vertices[p[1].vertexIndex];
      auto& p2 = mesh.vertices[p[2].vertexIndex];

      Triangle_Intersection intersection;
      bool hitFound = intersect_triangle(ray, p0, p1, p2, intersection);
      if (hitFound && intersection.t < closestIntersection.t) {
        closestIntersection = intersection;
      }
    }
  }
}

const TriangleMesh& KdTree::GetMesh() const {
  return mesh;
}

const Bounding_Box& KdTree::GetMeshBounds() const {
  return meshBounds;
}

KdTree_Stats KdTree::calculate_stats() const {
    KdTree_Stats stats;

    int64_t triangle_per_leaf_accumulated = 0;

    for (auto node  : nodes) {
        if (node.IsLeaf()) {
            stats.leaf_count++;
            triangle_per_leaf_accumulated += node.GetTrianglesCount();

            if (node.GetTrianglesCount() == 0)
                stats.empty_leaf_count++;
            else if (node.GetTrianglesCount() == 1)
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
    std::vector<Depth_Info> depth_info { Depth_Info{0, 0} };

    size_t i = 0;
    while (i < depth_info.size()) {
        int32_t node_index = depth_info[i].node_index;
        uint8_t depth = depth_info[i].depth;

        if (nodes[node_index].IsLeaf()) {
            if (nodes[node_index].GetTrianglesCount() > 0)
                not_empty_leaf_depth_values.push_back(depth);
            else
                empty_leaf_depth_values.push_back(depth);
        } else {
            int32_t below_child_index = node_index + 1;
            depth_info.push_back({below_child_index, uint8_t(depth + 1)});

            int32_t above_child_index = nodes[node_index].GetAboveChild();
            depth_info.push_back({above_child_index, uint8_t(depth + 1)});
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

void KdTree_Stats::Print() {
    printf("leaf_count = %d\n", leaf_count);
    printf("empty_leaf_count = %d\n", empty_leaf_count);
    printf("single_triangle_leaf_count = %d\n", single_triangle_leaf_count);
    printf("perfect_depth = %d\n", perfect_depth);
    printf("[not empty leaves]\n");
    printf("average_triangle_count = %.2f\n", not_empty_leaf_stats.average_triangle_count);
    printf("average_depth = %.2f\n", not_empty_leaf_stats.average_depth);
    printf("depth_standard_deviation = %.2f\n", not_empty_leaf_stats.depth_standard_deviation);
    printf("[empty leaves]\n");
    printf("average_depth = %.2f\n", empty_leaf_stats.average_depth);
    printf("depth_standard_deviation = %.2f\n", empty_leaf_stats.depth_standard_deviation);
}
