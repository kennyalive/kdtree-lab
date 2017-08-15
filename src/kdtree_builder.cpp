#include "common.h"
#include "kdtree_builder.h"
#include "triangle_mesh.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

struct BoundEdge {
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

    static bool Less(BoundEdge edge1, BoundEdge edge2)
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

class KdTree_Builder {
public:
    KdTree_Builder(const TriangleMesh& mesh, const KdTree_Build_Params& buildParams);
    KdTree build();

private:
    void BuildNode(const Bounding_Box& nodeBounds, const int32_t* nodeTriangles, int32_t nodeTrianglesCount, int depth, int32_t* triangles0, int32_t* triangles1);
    void CreateLeaf(const int32_t* nodeTriangles, int32_t nodeTrianglesCount);
    Split SelectSplit(const Bounding_Box& nodeBounds, const int32_t* nodeTriangles, int32_t nodeTrianglesCount);
    Split SelectSplitForAxis(const Bounding_Box& nodeBounds, int32_t nodeTrianglesCount, int axis) const;

private:
    const TriangleMesh& mesh;
    KdTree_Build_Params buildParams;

    std::vector<Bounding_Box> triangleBounds;
    std::vector<BoundEdge> edgesBuffer;
    std::vector<int32_t> trianglesBuffer;
    std::vector<KdNode> nodes;
    std::vector<int32_t> triangleIndices;
};

KdTree build_kdtree(const TriangleMesh& mesh, const KdTree_Build_Params& build_params) {
    KdTree_Builder builder(mesh, build_params);
    return builder.build();
}

enum {
  // max count is chosen such that maxTrianglesCount * 2 is still an int32_t,
  // this simplifies implementation.
  maxTrianglesCount = 0x3fffffff // max ~ 1 billion triangles
};

KdTree_Builder::KdTree_Builder(const TriangleMesh& mesh, const KdTree_Build_Params& buildParams)
: mesh(mesh)
, buildParams(buildParams)
{
  if (mesh.GetTriangleCount() > maxTrianglesCount) {
    RuntimeError("exceeded the maximum number of mesh triangles: " +
                 std::to_string(maxTrianglesCount));
  }

  if (this->buildParams.maxDepth <= 0) {
    this->buildParams.maxDepth = std::lround(8.0 + 1.3 * std::floor(std::log2(mesh.GetTriangleCount())));
  }
  this->buildParams.maxDepth = std::min(this->buildParams.maxDepth, static_cast<int>(KdTree::maxTraversalDepth));
}

KdTree KdTree_Builder::build()
{
  const auto trianglesCount = mesh.GetTriangleCount();

  // initialize bounding boxes
  triangleBounds.resize(trianglesCount);
  Bounding_Box meshBounds;

  for (auto i = 0; i < trianglesCount; i++) {
    triangleBounds[i] = mesh.GetTriangleBounds(i);
    meshBounds = Bounding_Box::get_union(meshBounds, triangleBounds[i]);
  }

  // initialize working memory
  edgesBuffer.resize(2 * trianglesCount);
  trianglesBuffer.resize(trianglesCount * (buildParams.maxDepth + 1));

  // fill triangle indices for root node
  for (auto i = 0; i < trianglesCount; i++)
    trianglesBuffer[i] = i;

  // recursively build all nodes
  BuildNode(meshBounds, trianglesBuffer.data(), trianglesCount,
            buildParams.maxDepth, trianglesBuffer.data(),
            trianglesBuffer.data() + trianglesCount);

  return KdTree(std::move(nodes), std::move(triangleIndices), mesh);
}

void KdTree_Builder::BuildNode(const Bounding_Box& nodeBounds,
                              const int32_t* nodeTriangles,
                              int32_t nodeTrianglesCount, int depth,
                              int32_t* triangles0, int32_t* triangles1)
{
  if (nodes.size() >= KdNode::maxNodesCount)
    RuntimeError("maximum number of KdTree nodes has been reached: " +
                 std::to_string(KdNode::maxNodesCount));

  // check if leaf node should be created
  if (nodeTrianglesCount <= buildParams.leafTrianglesLimit || depth == 0) {
    CreateLeaf(nodeTriangles, nodeTrianglesCount);
    return;
  }

  // select split position
  auto split = SelectSplit(nodeBounds, nodeTriangles, nodeTrianglesCount);
  if (split.edge == -1) {
    CreateLeaf(nodeTriangles, nodeTrianglesCount);
    return;
  }
  float splitPosition = edgesBuffer[split.edge].positionOnAxis;

  // classify triangles with respect to split
  int32_t n0 = 0;
  for (int32_t i = 0; i < split.edge; i++) {
    if (edgesBuffer[i].IsStart())
      triangles0[n0++] = edgesBuffer[i].GetTriangleIndex();
  }

  int32_t n1 = 0;
  for (int32_t i = split.edge + 1; i < 2 * nodeTrianglesCount; i++) {
    if (edgesBuffer[i].IsEnd())
      triangles1[n1++] = edgesBuffer[i].GetTriangleIndex();
  }

  // add interior node and recursively create children nodes
  auto thisNodeIndex = static_cast<int32_t>(nodes.size());
  nodes.push_back(KdNode());

  Bounding_Box bounds0 = nodeBounds;
  bounds0.max_point[split.axis] = splitPosition;
  BuildNode(bounds0, triangles0, n0, depth - 1, triangles0, triangles1 + n1);

  auto aboveChild = static_cast<int32_t>(nodes.size());
  nodes[thisNodeIndex].InitInteriorNode(split.axis, aboveChild, splitPosition);

  Bounding_Box bounds1 = nodeBounds;
  bounds1.min_point[split.axis] = splitPosition;
  BuildNode(bounds1, triangles1, n1, depth - 1, triangles0, triangles1);
}

void KdTree_Builder::CreateLeaf(const int32_t* nodeTriangles,
                               int32_t nodeTrianglesCount)
{
  KdNode node;
  if (nodeTrianglesCount == 0) {
    node.InitEmptyLeaf();
  }
  else if (nodeTrianglesCount == 1) {
    node.InitLeafWithSingleTriangle(nodeTriangles[0]);
  }
  else {
    node.InitLeafWithMultipleTriangles(nodeTrianglesCount, static_cast<int32_t>(triangleIndices.size()));
    triangleIndices.insert(triangleIndices.end(), nodeTriangles, nodeTriangles + nodeTrianglesCount);
  }
  nodes.push_back(node);
}

Split KdTree_Builder::SelectSplit(const Bounding_Box& nodeBounds,
                                                const int32_t* nodeTriangles,
                                                int32_t nodeTrianglesCount)
{
  // Determine axes iteration order.
  int axes[3];
  if (buildParams.splitAlongTheLongestAxis) {
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
      auto triangle = static_cast<uint32_t>(nodeTriangles[i]);
      edgesBuffer[2 * i + 0] = {triangleBounds[triangle].min_point[axis],
                                triangle | 0};

      edgesBuffer[2 * i + 1] = {triangleBounds[triangle].max_point[axis],
                                triangle | BoundEdge::endMask};
    }
    std::stable_sort(edgesBuffer.data(),
                     edgesBuffer.data() + 2 * nodeTrianglesCount,
                     BoundEdge::Less);

    // select split position
    auto split = SelectSplitForAxis(nodeBounds, nodeTrianglesCount, axis);
    if (split.edge != -1) {
      if (buildParams.splitAlongTheLongestAxis)
        return split;
      if (split.cost < bestSplit.cost)
        bestSplit = split;
    }
  }

  // If split axis is not the last axis (2) then we should reinitialize
  // edgesBuffer to
  // contain data for split axis since edgesBuffer will be used later.
  if (bestSplit.axis == 0 || bestSplit.axis == 1) {
    for (int32_t i = 0; i < nodeTrianglesCount; i++) {
      auto triangle = static_cast<uint32_t>(nodeTriangles[i]);

      edgesBuffer[2 * i + 0] = {
          triangleBounds[triangle].min_point[bestSplit.axis], triangle | 0};

      edgesBuffer[2 * i + 1] = {
          triangleBounds[triangle].max_point[bestSplit.axis],
          triangle | BoundEdge::endMask};
    }
    std::stable_sort(edgesBuffer.data(),
                     edgesBuffer.data() + 2 * nodeTrianglesCount,
                     BoundEdge::Less);
  }
  return bestSplit;
}

Split KdTree_Builder::SelectSplitForAxis(
    const Bounding_Box& nodeBounds, int32_t nodeTrianglesCount, int axis) const
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
                     buildParams.intersectionCost * nodeTrianglesCount};

  int32_t numBelow = 0;
  int32_t numAbove = nodeTrianglesCount;

  int32_t i = 0;
  while (i < numEdges) {
    BoundEdge edge = edgesBuffer[i];

    // find group of edges with the same axis position: [i, groupEnd)
    int groupEnd = i + 1;
    while (groupEnd < numEdges &&
           edge.positionOnAxis == edgesBuffer[groupEnd].positionOnAxis)
      groupEnd++;

    // [i, middleEdge) - edges End points.
    // [middleEdge, groupEnd) - edges Start points.
    int middleEdge = i;
    while (middleEdge != groupEnd && edgesBuffer[middleEdge].IsEnd())
      middleEdge++;

    numAbove -= middleEdge - i;

    float t = edge.positionOnAxis;
    if (t > nodeBounds.min_point[axis] && t < nodeBounds.max_point[axis]) {
      auto belowS = s0 + d0 * (t - nodeBounds.min_point[axis]);
      auto aboveS = s0 + d0 * (nodeBounds.max_point[axis] - t);

      auto pBelow = belowS * invTotalS;
      auto pAbove = aboveS * invTotalS;

      auto emptyBonus =
          (numBelow == 0 || numAbove == 0) ? buildParams.emptyBonus : 0.0f;

      auto cost = buildParams.traversalCost +
                  (1.0f - emptyBonus) * buildParams.intersectionCost *
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
