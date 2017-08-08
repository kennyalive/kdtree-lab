#pragma once

#include "bounding_box.h"

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

  std::vector<Vector> vertices;
  std::vector<Vector> normals;
  std::vector<Triangle> triangles;

  int32_t GetTriangleCount() const;
  int32_t GetVertexCount() const;
  Bounding_Box GetTriangleBounds(int32_t triangleIndex) const;
  Bounding_Box GetBounds() const;
};
