#pragma once

#include <cmath>

template <typename FLOAT_TYPE> struct v3 {
  FLOAT_TYPE x = 0.0F;
  FLOAT_TYPE y = 0.0F;
  FLOAT_TYPE z = 0.0F;

  v3() : x(0.0), y(0.0), z(0.0) {}
  v3(FLOAT_TYPE x, FLOAT_TYPE y, FLOAT_TYPE z) : x(x), y(y), z(z) {}

  FLOAT_TYPE length() const { return std::sqrt(x * x + y * y + z * z); }

  FLOAT_TYPE distance(v3<FLOAT_TYPE> &other) const {
    // TODO remove this method
    FLOAT_TYPE dx = x - other.x;
    FLOAT_TYPE dy = y - other.y;
    FLOAT_TYPE dz = z - other.z;
    return v3<FLOAT_TYPE>(dx, dy, dz).length();
  }

  v3<FLOAT_TYPE> &operator+=(const v3<FLOAT_TYPE> &v) {
    this->x += v.x;
    this->y += v.y;
    this->z += v.z;
    return *this;
  }

  v3<FLOAT_TYPE> &operator-=(const v3<FLOAT_TYPE> &v) {
    this->x -= v.x;
    this->y -= v.y;
    this->z -= v.z;
    return *this;
  }

  v3<FLOAT_TYPE> &operator*=(const v3<FLOAT_TYPE> &v) {
    this->x *= v.x;
    this->y *= v.y;
    this->z *= v.z;
    return *this;
  }

  v3<FLOAT_TYPE> &operator/=(const v3<FLOAT_TYPE> &v) {
    this->x /= v.x;
    this->y /= v.y;
    this->z /= v.z;
    return *this;
  }

  v3<FLOAT_TYPE> operator+(const v3<FLOAT_TYPE> &v) const {
    return v3(x + v.x, y + v.y, z + v.z);
  }

  v3<FLOAT_TYPE>operator-(const v3<FLOAT_TYPE> &v) const {
    return v3(x - v.x, y - v.y, z - v.z);
  }

  v3<FLOAT_TYPE> operator*(const v3<FLOAT_TYPE> &v) const {
    return v3(x * v.x, y * v.y, z * v.z);
  }

  v3<FLOAT_TYPE> operator/(const v3<FLOAT_TYPE> &v) const {
    return v3(x / v.x, y / v.y, z / v.z);
  }

  v3<FLOAT_TYPE> operator+(const FLOAT_TYPE scalar) const {
    return v3(x + scalar, y + scalar, z + scalar);
  }

  v3<FLOAT_TYPE> operator-(const FLOAT_TYPE scalar) const {
    return v3(x - scalar, y - scalar, z - scalar);
  }

  v3<FLOAT_TYPE> operator*(const FLOAT_TYPE scalar) const {
    return v3(x * scalar, y * scalar, z * scalar);
  }

  template <typename F2> v3<FLOAT_TYPE> operator/(const F2 scalar) const {
    return v3(x / static_cast<FLOAT_TYPE>(scalar),
                y / static_cast<FLOAT_TYPE>(scalar),
                z / static_cast<FLOAT_TYPE>(scalar));
  }
};

template <typename F1, typename F2>
v3<F1> operator+(const F2 scalar, const v3<F1> &v) {
  return v3<F1>(v.x + static_cast<F1>(scalar), v.y + static_cast<F1>(scalar),
                  v.z + static_cast<F1>(scalar));
}

template <typename F1, typename F2>
v3<F1> operator-(const F2 scalar, const v3<F1> &v) {
  return v3<F1>(v.x - static_cast<F1>(scalar), v.y - static_cast<F1>(scalar),
                  v.z - static_cast<F1>(scalar));
}

template <typename F1, typename F2>
v3<F1> operator*(const F2 scalar, const v3<F1> &v) {
  return v3<F1>(v.x * static_cast<F1>(scalar), v.y * static_cast<F1>(scalar),
                  v.z * static_cast<F1>(scalar));
}

template <typename F1, typename F2>
v3<F1> operator/(const F2 scalar, const v3<F1> &v) {
  return v3<F1>(v.x / static_cast<F1>(scalar), v.y / static_cast<F1>(scalar),
                  v.z / static_cast<F1>(scalar));
}
