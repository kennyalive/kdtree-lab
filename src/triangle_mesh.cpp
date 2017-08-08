#include "triangle_mesh.h"

int32_t TriangleMesh::GetTriangleCount() const {
  return static_cast<int32_t>(triangles.size());
}

int32_t TriangleMesh::GetVertexCount() const {
    return static_cast<int32_t>(vertices.size());
}

Bounding_Box TriangleMesh::GetTriangleBounds(int32_t triangleIndex) const {
  const auto& p = triangles[triangleIndex].points;
  auto bounds = Bounding_Box(vertices[p[0].vertexIndex]);
  bounds.extend(vertices[p[1].vertexIndex]);
  bounds.extend(vertices[p[2].vertexIndex]);
  return bounds;
}

Bounding_Box TriangleMesh::GetBounds() const {
  Bounding_Box bounds;
  for (int32_t i = 0; i < GetTriangleCount(); i++) {
    bounds = Bounding_Box::get_union(bounds, GetTriangleBounds(i));
  }
  return bounds;
}
