#pragma once

#include <cassert>
#include <cmath>

struct Vector {
  float x;
  float y;
  float z;

  Vector()
  : x(0.f)
  , y(0.f)
  , z(0.f)
  {}

  Vector(float value)
  : x(value)
  , y(value)
  , z(value)
  {}

  Vector(float x, float y, float z)
  : x(x)
  , y(y)
  , z(z)
  {}

  Vector& operator+=(const Vector& v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  Vector operator+(const Vector& v) const {
    return Vector(x + v.x, y + v.y, z + v.z);
  }

  Vector operator-(const Vector& v) const {
    return Vector(x - v.x, y - v.y, z - v.z);
  }

  Vector operator*(float value) const {
    return Vector(x * value, y * value, z * value);
  }

  Vector operator/(float value) const {
    const float inv_value = 1.0f / value;
    return Vector(x * inv_value, y * inv_value, z * inv_value);
  }

  bool operator==(const Vector& v) const {
    return x == v.x && y == v.y && z == v.z;
  }

  bool operator!=(const Vector& v) const {
    return !(*this == v);
  }

  float operator[](int index) const {
    assert(index >= 0 && index < 3);
    return (&x)[index];
  }

  float& operator[](int index) {
    assert(index >= 0 && index < 3);
    return (&x)[index];
  }

  float length_squared() const {
    return x*x + y*y + z*z;
  }

  float length() const {
    return std::sqrt(length_squared());
  }

  Vector normalized() const {
    return *this / length();
  }
};

inline Vector operator*(float value, const Vector& v) {
    return v * value;
}

inline float dot(const Vector& a, const Vector& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Vector cross(const Vector& a, const Vector& b) {
    return Vector(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    );
}
