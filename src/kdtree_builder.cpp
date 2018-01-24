#include "common.h"
#include "kdtree_builder.h"
#include "triangle_mesh.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

struct Edge {
    float positionOnAxis;
    uint32_t triangleAndFlag;

    enum : uint32_t { endMask = 0x80000000 };
    enum : uint32_t { triangleMask = 0x7fffffff };

    bool IsStart() const
    {
        return (triangleAndFlag & endMask) == 0;
    }

    bool IsEnd() const
    {
        return !IsStart();
    }

    int32_t GetTriangleIndex() const
    {
        return static_cast<int32_t>(triangleAndFlag & triangleMask);
    }

    static bool Less(Edge edge1, Edge edge2)
    {
        if (edge1.positionOnAxis == edge2.positionOnAxis)
            return edge1.IsEnd() && edge2.IsStart();
        else
            return edge1.positionOnAxis < edge2.positionOnAxis;
    }
};

struct Split {
    int32_t edge;
    int axis;
    float cost;
};

struct Triangle_Info {
	int32_t triangle;
	Bounding_Box bounds;
};

class KdTree_Builder {
public:
    KdTree_Builder(const Triangle_Mesh& mesh, const KdTree_Build_Params& buildParams);
    KdTree build();

private:
    void build_node(const Bounding_Box& node_bounds, int32_t triangles_offset, int32_t triangle_count,
        int depth, int32_t above_triangles_offset);

    void CreateLeaf(const Triangle_Info* nodeTriangles, int32_t nodeTrianglesCount);
    Split SelectSplit(const Bounding_Box& nodeBounds, const Triangle_Info* nodeTriangles, int32_t nodeTrianglesCount);
    Split SelectSplitForAxis(const Bounding_Box& nodeBounds, int32_t nodeTrianglesCount, int axis) const;

private:
    const Triangle_Mesh& mesh;
    KdTree_Build_Params buildParams;

    std::vector<Edge> edges[3]; // edges for each axis

	std::size_t triangle_buffer_max_size = 0;
    std::vector<Triangle_Info> trianglesBuffer;
    std::vector<Triangle_Info> triangle_buffer2;
    std::vector<KdNode> nodes;
    std::vector<int32_t> triangleIndices;
};

KdTree build_kdtree(const Triangle_Mesh& mesh, const KdTree_Build_Params& build_params) {
    KdTree_Builder builder(mesh, build_params);
    return builder.build();
}

enum {
  // max count is chosen such that maxTrianglesCount * 2 is still an int32_t,
  // this simplifies implementation.
  maxTrianglesCount = 0x3fffffff // max ~ 1 billion triangles
};

KdTree_Builder::KdTree_Builder(const Triangle_Mesh& mesh, const KdTree_Build_Params& buildParams)
: mesh(mesh)
, buildParams(buildParams)
{
  if (mesh.get_triangle_count() > maxTrianglesCount) {
    RuntimeError("exceeded the maximum number of mesh triangles: " +
                 std::to_string(maxTrianglesCount));
  }

  if (this->buildParams.max_depth <= 0) {
    this->buildParams.max_depth = std::lround(8.0 + 1.3 * std::floor(std::log2(mesh.get_triangle_count())));
  }
  this->buildParams.max_depth = std::min(this->buildParams.max_depth, static_cast<int>(KdTree::max_traversal_depth));
}

KdTree KdTree_Builder::build() {
    const int32_t triangle_count = mesh.get_triangle_count();

    // Prepare working structures.
    for (int i = 0; i < 3; i++)
        edges[i].resize(2 * triangle_count);

    triangle_buffer_max_size = triangle_count * (buildParams.max_depth + 1);
    trianglesBuffer.resize(static_cast<size_t>(triangle_count * 2.5));
    triangle_buffer2.resize(triangle_count);

    Bounding_Box mesh_bounds;
    for (int32_t i = 0; i < triangle_count; i++) {
        Bounding_Box bounds = mesh.get_triangle_bounds(i);
        trianglesBuffer[i] = { i, bounds };
        mesh_bounds = Bounding_Box::get_union(mesh_bounds, bounds);
    }
     
    // Recursively build all nodes.
    build_node(mesh_bounds, 0, triangle_count, buildParams.max_depth, triangle_count);
    return KdTree(std::move(nodes), std::move(triangleIndices), mesh);
}

void KdTree_Builder::build_node(const Bounding_Box& node_bounds, int32_t triangles_offset, int32_t triangle_count, int depth, int32_t above_triangles_offset)
{
  if (nodes.size() >= KdNode::maxNodesCount)
    RuntimeError("maximum number of KdTree nodes has been reached: " + std::to_string(KdNode::maxNodesCount));

  // check if leaf node should be created
  if (triangle_count <= buildParams.leaf_triangles_limit || depth == 0) {
    CreateLeaf(&trianglesBuffer[triangles_offset], triangle_count);
    return;
  }

  // select split position
  auto split = SelectSplit(node_bounds, &trianglesBuffer[triangles_offset], triangle_count);
  if (split.edge == -1) {
      CreateLeaf(&trianglesBuffer[triangles_offset], triangle_count);
      return;
  }
  float splitPosition = edges[split.axis][split.edge].positionOnAxis;

  memcpy(triangle_buffer2.data(), &trianglesBuffer[triangles_offset], triangle_count * sizeof(Triangle_Info));

  if (trianglesBuffer.size() < above_triangles_offset + triangle_count)
      trianglesBuffer.resize(trianglesBuffer.size() + mesh.get_triangle_count());

  // classify triangles with respect to split
  int32_t n0 = 0;
  for (int32_t i = 0; i < split.edge; i++) {
	  if (edges[split.axis][i].IsStart()) {
		  int32_t index = edges[split.axis][i].GetTriangleIndex();
		  Triangle_Info triangle_info = triangle_buffer2[index];

		  // TODO: clip bounds here
		  
		  trianglesBuffer[n0++] = triangle_info;
	  }
  }

  int32_t n1 = 0;
    for (int32_t i = split.edge + 1; i < 2 * triangle_count; i++) {
	  if (edges[split.axis][i].IsEnd()) {
		  int32_t index = edges[split.axis][i].GetTriangleIndex();
		  Triangle_Info triangle_info = triangle_buffer2[index];

		  // TODO: clip bounds here

		  trianglesBuffer[above_triangles_offset + n1++] = triangle_info;
	  }
  }

  // add interior node and recursively create children nodes
  auto thisNodeIndex = static_cast<int32_t>(nodes.size());
  nodes.push_back(KdNode());

  Bounding_Box bounds0 = node_bounds;
  bounds0.max_point[split.axis] = splitPosition;
  build_node(bounds0, 0, n0, depth - 1, above_triangles_offset + n1);

  auto aboveChild = static_cast<int32_t>(nodes.size());
  nodes[thisNodeIndex].InitInteriorNode(split.axis, aboveChild, splitPosition);

  Bounding_Box bounds1 = node_bounds;
  bounds1.min_point[split.axis] = splitPosition;
  build_node(bounds1, above_triangles_offset, n1, depth - 1, above_triangles_offset);
}

void KdTree_Builder::CreateLeaf(const Triangle_Info* nodeTriangles, int32_t nodeTrianglesCount)
{
  KdNode node;
  if (nodeTrianglesCount == 0) {
    node.InitEmptyLeaf();
  }
  else if (nodeTrianglesCount == 1) {
    node.InitLeafWithSingleTriangle(nodeTriangles[0].triangle);
  }
  else {
    node.InitLeafWithMultipleTriangles(nodeTrianglesCount, static_cast<int32_t>(triangleIndices.size()));

	for (int32_t i = 0; i < nodeTrianglesCount; i++)
		triangleIndices.push_back(nodeTriangles[i].triangle);
  }
  nodes.push_back(node);
}

Split KdTree_Builder::SelectSplit(const Bounding_Box& nodeBounds, const Triangle_Info* nodeTriangles, int32_t nodeTrianglesCount)
{
  // Determine axes iteration order.
  int axes[3];
  if (buildParams.split_along_the_longest_axis) {
    Vector diag = nodeBounds.max_point - nodeBounds.min_point;
    if (diag.x >= diag.y && diag.x >= diag.z) {
      axes[0] = 0;
      axes[1] = diag.y >= diag.z ? 1 : 2;
    }
    else if (diag.y >= diag.x && diag.y >= diag.z) {
      axes[0] = 1;
      axes[1] = diag.x >= diag.z ? 0 : 2;
    }
    else {
      axes[0] = 2;
      axes[1] = diag.x >= diag.y ? 0 : 1;
    }
    axes[2] = 3 - axes[0] - axes[1]; // since 0 + 1 + 2 == 3
  }
  else {
    axes[0] = 0;
    axes[1] = 1;
    axes[2] = 2;
  }

  // Select spliting axis and position. If buildParams.splitAlongTheLongestAxis
  // is true then we stop at the first axis that gives a valid split.
  Split bestSplit = {-1, -1, std::numeric_limits<float>::infinity()};

  for (int axis : axes) {
    // initialize edges
    for (int32_t i = 0; i < nodeTrianglesCount; i++) {
	    auto& bounds = nodeTriangles[i].bounds;
        edges[axis][2 * i + 0] = {bounds.min_point[axis], static_cast<uint32_t>(i)};
        edges[axis][2 * i + 1] = {bounds.max_point[axis], static_cast<uint32_t>(i) | Edge::endMask};
    }

    std::stable_sort(edges[axis].data(), edges[axis].data() + 2 * nodeTrianglesCount, Edge::Less);

    // select split position
    auto split = SelectSplitForAxis(nodeBounds, nodeTrianglesCount, axis);
    if (split.edge != -1) {
        if (buildParams.split_along_the_longest_axis)
        return split;
        if (split.cost < bestSplit.cost)
        bestSplit = split;
    }
  }

  return bestSplit;
}

Split KdTree_Builder::SelectSplitForAxis(const Bounding_Box& nodeBounds, int32_t nodeTrianglesCount, int axis) const
{
  static const int otherAxis[3][2] = {{1, 2}, {0, 2}, {0, 1}};
  const int otherAxis0 = otherAxis[axis][0];
  const int otherAxis1 = otherAxis[axis][1];
  const Vector diag = nodeBounds.max_point - nodeBounds.min_point;

  const float s0 = 2.0f * (diag[otherAxis0] * diag[otherAxis1]);
  const float d0 = 2.0f * (diag[otherAxis0] + diag[otherAxis1]);

  const float invTotalS =
      1.0f / (2.0f * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z));

  const int32_t numEdges = 2 * nodeTrianglesCount;

  Split bestSplit = {-1, axis,
                     buildParams.intersection_cost * nodeTrianglesCount};

  int32_t numBelow = 0;
  int32_t numAbove = nodeTrianglesCount;

  int32_t i = 0;
  while (i < numEdges) {
    Edge edge = edges[axis][i];

    // find group of edges with the same axis position: [i, groupEnd)
    int groupEnd = i + 1;
    while (groupEnd < numEdges &&
           edge.positionOnAxis == edges[axis][groupEnd].positionOnAxis)
      groupEnd++;

    // [i, middleEdge) - edges End points.
    // [middleEdge, groupEnd) - edges Start points.
    int middleEdge = i;
    while (middleEdge != groupEnd && edges[axis][middleEdge].IsEnd())
      middleEdge++;

    numAbove -= middleEdge - i;

    float t = edge.positionOnAxis;
    if (t > nodeBounds.min_point[axis] && t < nodeBounds.max_point[axis]) {
      auto belowS = s0 + d0 * (t - nodeBounds.min_point[axis]);
      auto aboveS = s0 + d0 * (nodeBounds.max_point[axis] - t);

      auto pBelow = belowS * invTotalS;
      auto pAbove = aboveS * invTotalS;

      auto empty_bonus =
          (numBelow == 0 || numAbove == 0) ? buildParams.empty_bonus : 0.0f;

      auto cost = buildParams.traversal_cost +
                  (1.0f - empty_bonus) * buildParams.intersection_cost *
                      (pBelow * numBelow + pAbove * numAbove);

      if (cost < bestSplit.cost) {
        bestSplit.edge = (middleEdge == groupEnd) ? middleEdge - 1 : middleEdge;
        bestSplit.cost = cost;
      }
    }

    numBelow += groupEnd - middleEdge;
    i = groupEnd;
  }
  return bestSplit;
}
