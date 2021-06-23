#pragma once

#include <string>

#include "v3.h"

template <typename FLOAT_TYPE> struct Bezier;

template <typename FLOAT_TYPE> struct Hermite {
  v3<FLOAT_TYPE> p0;
  v3<FLOAT_TYPE> p1;
  v3<FLOAT_TYPE> t0;
  v3<FLOAT_TYPE> t1;

  Hermite(const v3<FLOAT_TYPE> &p0, const v3<FLOAT_TYPE> &p1,
          const v3<FLOAT_TYPE> &t0, const v3<FLOAT_TYPE> &t1)
      : p0(p0), p1(p1), t0(t0), t1(t1) {}

  v3<FLOAT_TYPE> evaluate(FLOAT_TYPE t) const;

  Bezier<FLOAT_TYPE> to_bezier() const;
  void to_csv(const std::string &fileName, int numTestPoints) const;
};
