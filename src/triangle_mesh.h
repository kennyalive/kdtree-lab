#pragma once

#include "bounding_box.h"
#include "vector.h"
#include <array>
#include <cstdint>
#include <vector>

struct TriangleMesh {
  struct TrianglePoint {
    int32_t vertexIndex;
  };
  struct Triangle {
    std::array<TrianglePoint, 3> points;
  };

  std::vector<Vector_f> vertices;
  std::vector<Vector_f> normals;
  std::vector<Triangle> triangles;

  int32_t GetTriangleCount() const;
  int32_t GetVertexCount() const;
  BoundingBox_f GetTriangleBounds(int32_t triangleIndex) const;
  BoundingBox_f GetBounds() const;
};
