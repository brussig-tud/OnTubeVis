#include "hermite.h"

#include <fstream>

#include "bezier.h"

template <typename FLOAT_TYPE>
v3<FLOAT_TYPE> Hermite<FLOAT_TYPE>::evaluate(FLOAT_TYPE t) const {
  auto a = 2.0 * t * t * t - 3.0 * t * t + 1;
  auto b = t * t * t - 2.0 * t * t + t;
  auto c = -2.0 * t * t * t + 3 * t * t;
  auto d = t * t * t - t * t;
  return a * p0 + b * t0 + c * p1 + d * t1;
}

template <typename FLOAT_TYPE>
void Hermite<FLOAT_TYPE>::to_csv(const std::string &fileName,
                                 const int numTestPoints) const {
  std::ofstream f(fileName);

  for (int i = 0; i < numTestPoints; i++) {
    FLOAT_TYPE t =
        static_cast<FLOAT_TYPE>(i) / static_cast<FLOAT_TYPE>(numTestPoints);
    auto v = evaluate(t);
    f << v.x << " " << v.y << std::endl;
  }
}

template <typename FLOAT_TYPE>
Bezier<FLOAT_TYPE> Hermite<FLOAT_TYPE>::to_bezier() const {
  auto result = Bezier<FLOAT_TYPE>();
  result.points[0] = p0;
  result.points[3] = p1;
  result.points[1] = p0 + (t0 / 3.0);
  result.points[2] = p1 - (t1 / 3.0);
  return result;
}

template struct Hermite<float>;
template struct Hermite<double>;
