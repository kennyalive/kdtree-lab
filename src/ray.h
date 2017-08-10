#pragma once

#include "vector.h"
#include <cassert>
#include <cmath>

class Ray {
public:
  Ray(const Vector& origin, const Vector& direction)
  : origin(origin)
  , direction(direction)
  , invDirection(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z)
  {
    assert(std::abs(direction.length() - 1.0f) < 1e-6f);
  }

  const Vector& GetOrigin() const
  {
    return origin;
  }

  const Vector& GetDirection() const
  {
    return direction;
  }

  const Vector& GetInvDirection() const
  {
    return invDirection;
  }

  void Advance(float t)
  {
    origin = GetPoint(t);
  }

  Vector GetPoint(float t) const
  {
    return origin + direction * t;
  }

private:
  Vector origin;
  Vector direction;
  Vector invDirection;
};
