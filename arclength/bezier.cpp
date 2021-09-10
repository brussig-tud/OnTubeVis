#include "bezier.h"

#include <fstream>
#include <iostream>
#include <limits>

#include <fastgl.h>

#include "PolynomialRegression.h"
#include "hermite.h"

template <typename FLOAT_TYPE> v3<FLOAT_TYPE> Bezier<FLOAT_TYPE>::evaluate(const FLOAT_TYPE t) const {
    auto t0 = (1.0 - t) * (1.0 - t) * (1.0 - t) * points[0];
    auto t1 = 3.0 * (1.0 - t) * (1.0 - t) * t * points[1];
    auto t2 = 3.0 * (1.0 - t) * t * t * points[2];
    auto t3 = t * t * t * points[3];
    return t0 + t1 + t2 + t3;
}

template <typename FLOAT_TYPE> v3<FLOAT_TYPE> Bezier<FLOAT_TYPE>::derivative(const FLOAT_TYPE t) const {
    auto t0 = 3.0 * (1.0 - t) * (1.0 - t) * (points[1] - points[0]);
    auto t1 = 6.0 * (1.0 - t) * t * (points[2] - points[1]);
    auto t2 = 3.0 * t * t * (points[3] - points[2]);
    return t0 + t1 + t2;
}

template <typename FLOAT_TYPE>
std::vector<FLOAT_TYPE> extrema(FLOAT_TYPE p0, FLOAT_TYPE p1, FLOAT_TYPE p2, FLOAT_TYPE p3) {
    std::vector<FLOAT_TYPE> result = {};

    FLOAT_TYPE a = 3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3);
    FLOAT_TYPE b = 6.0 * (p0 - 2.0 * p1 + p2);
    FLOAT_TYPE c = 3.0 * (p1 - p0);

    if (a == 0.0) {
        if (b == 0.0) {
            return result;
        }
        FLOAT_TYPE t = -c / b;
        result.push_back(t);
        return result;
    }

    FLOAT_TYPE t0 = (-b + sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    if (t0 >= 0.0 && t0 <= 1.0) {
        result.push_back(t0);
    }

    FLOAT_TYPE t1 = (-b - sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    if (t1 >= 0.0 && t1 <= 1.0) {
        result.push_back(t1);
    }

    return result;
}

template <typename FLOAT_TYPE> aabb<FLOAT_TYPE> Bezier<FLOAT_TYPE>::bounding_box() const {
    auto extremaX = extrema(points[0].x, points[1].x, points[2].x, points[3].x);
    extremaX.push_back(0.0F);
    extremaX.push_back(1.0F);

    auto extremaY = extrema(points[0].y, points[1].y, points[2].y, points[3].y);
    extremaY.push_back(0.0F);
    extremaY.push_back(1.0F);

    auto min = v3<FLOAT_TYPE>(std::numeric_limits<FLOAT_TYPE>::max(), std::numeric_limits<FLOAT_TYPE>::max(),
                              std::numeric_limits<FLOAT_TYPE>::max());
    auto max = v3<FLOAT_TYPE>(std::numeric_limits<FLOAT_TYPE>::min(), std::numeric_limits<FLOAT_TYPE>::min(),
                              std::numeric_limits<FLOAT_TYPE>::min());

    for (const auto &tx : extremaX) {
        auto x = evaluate(tx).x;
        if (x < min.x) {
            min.x = x;
        }
        if (x > max.x) {
            max.x = x;
        }
    }

    for (const auto &ty : extremaY) {
        auto y = evaluate(ty).y;
        if (y < min.y) {
            min.y = y;
        }
        if (y > max.y) {
            max.y = y;
        }
    }

    return {min, max};
}

template <typename FLOAT_TYPE> Bezier<FLOAT_TYPE> Bezier<FLOAT_TYPE>::axis_align() const {
    Bezier<FLOAT_TYPE> result = {points};
    for (int i = 0; i < 4; i++) {
        result.points[i] -= points[0];
    }

    return result;
}

template <typename FLOAT_TYPE>
std::pair<Bezier<FLOAT_TYPE>, Bezier<FLOAT_TYPE>> Bezier<FLOAT_TYPE>::split(FLOAT_TYPE t) const {
    auto splitPoint = evaluate(t);
    Bezier left = {};
    Bezier right = {};
    left.points[0] = points[0];
    left.points[3] = splitPoint;
    right.points[0] = splitPoint;
    right.points[3] = points[3];

    auto helperMidpoint = (1.0 - t) * points[1] + t * points[2];

    auto left1 = (1.0 - t) * points[0] + t * points[1];
    auto left2 = (1.0 - t) * left1 + t * helperMidpoint;
    left.points[1] = left1;
    left.points[2] = left2;

    auto right2 = (1.0 - t) * points[2] + t * points[3];
    auto right1 = (1.0 - t) * helperMidpoint + t * right2;
    right.points[1] = right1;
    right.points[2] = right2;

    return std::make_pair(left, right);
}

template <typename FLOAT_TYPE> FLOAT_TYPE Bezier<FLOAT_TYPE>::length_chord() const {
    return points[0].distance(points[3]);
}

template <typename FLOAT_TYPE> FLOAT_TYPE Bezier<FLOAT_TYPE>::length_control_polygon() const {
    FLOAT_TYPE length = 0.0;
    for (int i = 1; i < points.size(); i++) {
        length += points[i - 1].distance(points[i]);
    }
    return length;
}

template <typename FLOAT_TYPE>
FLOAT_TYPE Bezier<FLOAT_TYPE>::arc_length_legendre_gauss(const FLOAT_TYPE t, const int numSamples) const {
    if (numSamples < 2) {
        return 0.0;
    }

    FLOAT_TYPE zHalf = t * 0.5;
    FLOAT_TYPE result = 0.0;
    for (int i = 0; i < numSamples; i++) {
        const fastgl::QuadPair pair = fastgl::GLPair(numSamples, i + 1);
        FLOAT_TYPE T = pair.x();
        FLOAT_TYPE t_ = zHalf * T + zHalf;
        auto value = derivative(t_);
        FLOAT_TYPE C = pair.weight;
        result += C * value.length();
    }

    return zHalf * result;
}

template <typename FLOAT_TYPE>
FLOAT_TYPE Bezier<FLOAT_TYPE>::arc_length_even_subdivision(FLOAT_TYPE t, int numSegments) const {
    FLOAT_TYPE step = 1.0 / static_cast<FLOAT_TYPE>(numSegments);
    FLOAT_TYPE result = 0.0;

    v3<FLOAT_TYPE> lastPoint = points[0];
    for (int i = 0; i < numSegments; i++) {
        FLOAT_TYPE t_ = static_cast<FLOAT_TYPE>(i + 1) * step;
        if (t_ >= t) {
            v3<FLOAT_TYPE> nextPoint = evaluate(t);
            result += lastPoint.distance(nextPoint);
            break;
        }

        v3<FLOAT_TYPE> nextPoint = evaluate(t_);
        result += lastPoint.distance(nextPoint);
        lastPoint = nextPoint;
    }

    return result;
}

template <typename FLOAT_TYPE>
FLOAT_TYPE arc_length_adaptive_subdivision(const Bezier<FLOAT_TYPE> &b, FLOAT_TYPE epsilon) {
    auto lengthControlPolygon = b.length_control_polygon();
    auto lengthChord = b.length_chord();
    auto degree = 3.0;

    // TODO use better error metric
    auto error = lengthControlPolygon - lengthChord;
    if (error < epsilon) {
        return (2.0 * lengthChord + (degree - 1.0) * lengthControlPolygon) / (degree + 1.0);
    }

    auto new_split = b.split(0.5F);
    auto b1 = new_split.first;
    auto b2 = new_split.second;
    FLOAT_TYPE newEpsilon = epsilon / 2.0;
    return arc_length_adaptive_subdivision(b1, newEpsilon) + arc_length_adaptive_subdivision(b2, newEpsilon);
}

template <typename FLOAT_TYPE>
FLOAT_TYPE Bezier<FLOAT_TYPE>::arc_length_adaptive_subdivision(FLOAT_TYPE t, FLOAT_TYPE epsilon) const {
    auto parts = split(t);
    return ::arc_length_adaptive_subdivision(parts.first, epsilon);
}

template <typename FLOAT_TYPE>
ParameterizationRegression<FLOAT_TYPE> *
Bezier<FLOAT_TYPE>::parameterization_regression(const int order, int numSamples, const int numLegendreSamples) const {
    if (numSamples == -1) {
        numSamples = order * 10;
    }

    std::vector<FLOAT_TYPE> lengths = {};
    std::vector<FLOAT_TYPE> ts = {};
    for (int i = 0; i < numSamples; i++) {
        FLOAT_TYPE t = static_cast<FLOAT_TYPE>(i) / static_cast<FLOAT_TYPE>(numSamples);
        ts.emplace_back(t);

        FLOAT_TYPE length = arc_length_legendre_gauss(t, numLegendreSamples);
        lengths.emplace_back(length);
    }

    std::vector<FLOAT_TYPE> coeffs = {};
    bool success = PolynomialRegression<FLOAT_TYPE>().fitIt(lengths, ts, order, coeffs);
    if (!success) {
        // TODO decide what to do here
        return {};
    }

    auto result = new ParameterizationRegression<FLOAT_TYPE>();
    result->_length = arc_length_legendre_gauss();
    result->coefficients.resize(coeffs.size());
    for (int i = 0; i < coeffs.size(); i++) {
        result->coefficients[i] = coeffs[i];
    }

    return result;
}

template <typename F1, typename F2> bool is_equal(F1 a, F2 b, F1 epsilon = 1.0e-5) {
    return ((a - static_cast<F1>(b)) < epsilon) && ((static_cast<F1>(b) - a) < epsilon);
}

template <typename FLOAT_TYPE> v3<bool> is_equal(v3<FLOAT_TYPE> a, v3<FLOAT_TYPE> b, FLOAT_TYPE epsilon = 1.0e-5) {
    return {
          is_equal(a.x, b.x, epsilon),
          is_equal(a.y, b.y, epsilon),
          is_equal(a.z, b.z, epsilon),
    };
}

template <typename FLOAT_TYPE>
void Bezier<FLOAT_TYPE>::to_csv(const std::string &fileName, const int numTestPoints,
                                const bool evenPointDistribution) const {
    std::ofstream f(fileName);

    if (!evenPointDistribution) {
        for (int i = 0; i < numTestPoints; i++) {
            FLOAT_TYPE t = static_cast<FLOAT_TYPE>(i) / static_cast<FLOAT_TYPE>(numTestPoints);
            auto v = evaluate(t);
            f << v.x << " " << v.y << std::endl;
        }
        return;
    }

    auto approx = parameterization_subdivision_bezier_approximation();
    for (int i = 0; i < numTestPoints; i++) {
        FLOAT_TYPE d = static_cast<FLOAT_TYPE>(i) / static_cast<FLOAT_TYPE>(numTestPoints);
        d *= approx->length();
        FLOAT_TYPE t = approx->evaluate(d);
        if (t > 1.0) {
            continue;
        }
        auto v = evaluate(t);
        f << v.x << " " << v.y << std::endl;
    }
}

template <typename FLOAT_TYPE>
FLOAT_TYPE ParameterizationSubdivisionLegendreGauss<FLOAT_TYPE>::evaluate(const FLOAT_TYPE d) const {
    FLOAT_TYPE t0 = 0.0;
    FLOAT_TYPE t1 = 1.0;

    for (int i = 0; i < depth; i++) {
        FLOAT_TYPE t = (t0 + t1) / 2.0;
        auto l = b.arc_length_legendre_gauss(t, 100);
        if (d < l) {
            t1 = t;
        } else if (d > l) {
            t0 = t;
        } else {
            break;
        }
    }

    return (t0 + t1) / 2.0;
}
template <typename FLOAT_TYPE> FLOAT_TYPE ParameterizationSubdivisionLegendreGauss<FLOAT_TYPE>::length() const {
    return b.arc_length_legendre_gauss();
}

namespace std {
template <typename FLOAT_TYPE> v3<FLOAT_TYPE> sqrt(v3<FLOAT_TYPE> v) {
    return v3<FLOAT_TYPE>(std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z));
}
} // namespace std

template <typename FLOAT_TYPE>
std::vector<FLOAT_TYPE> intersection(const std::vector<FLOAT_TYPE> &v1, const std::vector<FLOAT_TYPE> &v2) {
    std::vector<FLOAT_TYPE> roots = {};
    for (const auto &r1 : v1) {
        bool found = false;
        for (const auto &r2 : v2) {
            if (is_equal(r1, r2)) {
                found = true;
                break;
            }
        }
        if (found) {
            roots.push_back(r1);
        }
    }
    return roots;
}

template <typename FLOAT_TYPE> void unique(std::vector<FLOAT_TYPE> &v) {
    for (int i = v.size() - 1; i >= 0; i--) {
        bool should_remove = false;
        for (int j = 0; j < v.size(); j++) {
            if (i == j) {
                continue;
            }
            if (is_equal(v[i], v[j])) {
                should_remove = true;
                break;
            }
        }
        if (should_remove) {
            v.erase(v.begin() + i);
        }
    }
}

template <typename FLOAT_TYPE>
ArcLengthBezierApproximation<FLOAT_TYPE> Bezier<FLOAT_TYPE>::arc_length_bezier_approximation(int numSegments,
                                                                                             int numSamples) const {
    if (numSegments < 1) {
        return {};
    }

    auto result = ArcLengthBezierApproximation<FLOAT_TYPE>();
    result.totalLength = arc_length_legendre_gauss(1.0, numSamples);
    result.lengths.push_back(0.0);

    auto dStep = 1.0 / static_cast<FLOAT_TYPE>(numSegments);
    for (int i = 0; i < numSegments; i++) {
        auto tPrev = static_cast<FLOAT_TYPE>(i) * dStep;
        auto tCur = static_cast<FLOAT_TYPE>(i + 1) * dStep;
        auto tDiff = tCur - tPrev;

        auto dPrev = result.lengths.back();
        auto dCur = arc_length_legendre_gauss(tCur, numSamples);
        auto dDiff = dCur - dPrev;

        FLOAT_TYPE sample1 = (tPrev + tDiff * (1.0 / 3.0));
        auto s1over3 = arc_length_legendre_gauss(sample1);
        auto s1over3Scaled = (s1over3 - dPrev) / dDiff;

        FLOAT_TYPE sample2 = (tPrev + tDiff * (2.0 / 3.0));
        auto s2over3 = arc_length_legendre_gauss(sample2);
        auto s2over3Scaled = (s2over3 - dPrev) / dDiff;

        auto y1 = (18.0 * s1over3Scaled - 9.0 * s2over3Scaled + 2.0) / 6.0;
        auto y2 = (-9.0 * s1over3Scaled + 18.0 * s2over3Scaled - 5.0) / 6.0;

        result.y1.push_back(y1);
        result.y2.push_back(y2);
        result.lengths.push_back(dCur);
    }

    return result;
}

template <typename FLOAT_TYPE> FLOAT_TYPE ArcLengthBezierApproximation<FLOAT_TYPE>::evaluate(FLOAT_TYPE t) const {
    if (t <= 0.0) {
        return 0.0;
    }
    if (t >= 1.0) {
        return totalLength;
    }

    size_t segmentCount = y1.size();
    size_t index = segmentCount * t;
    auto tStep = 1.0 / static_cast<FLOAT_TYPE>(segmentCount);
    auto tPrev = static_cast<FLOAT_TYPE>(index) * tStep;
    auto tCur = static_cast<FLOAT_TYPE>(index + 1) * tStep;
    auto tDiff = tCur - tPrev;

    auto x = (t - tPrev) / tDiff;
    auto t1 = 3.0 * (1.0 - x) * (1.0 - x) * x * y1[index];
    auto t2 = 3.0 * (1.0 - x) * x * x * y2[index];
    auto t3 = x * x * x;
    auto y = t1 + t2 + t3;

    auto dPrev = lengths[index];
    auto dCur = lengths[index + 1];
    auto dDiff = dCur - dPrev;
    return (y * dDiff) + dPrev;
}

template <typename FLOAT_TYPE>
FLOAT_TYPE ParameterizationBezierApproximation<FLOAT_TYPE>::evaluate(FLOAT_TYPE d) const {
    if (d <= 0.0) {
        return 0.0;
    }
    if (d >= totalLength) {
        return 1.0;
    }

    auto dNormalized = d / totalLength;
    size_t segmentCount = y1.size();
    size_t index = segmentCount * dNormalized;
    auto dStep = 1.0 / static_cast<FLOAT_TYPE>(segmentCount);
    auto dPrev = static_cast<FLOAT_TYPE>(index) * dStep;
    auto dCur = static_cast<FLOAT_TYPE>(index + 1) * dStep;
    auto dDiff = dCur - dPrev;

    auto x = (dNormalized - dPrev) / dDiff;
    auto t1 = 3.0 * (1.0 - x) * (1.0 - x) * x * y1[index];
    auto t2 = 3.0 * (1.0 - x) * x * x * y2[index];
    auto t3 = x * x * x;
    auto y = t1 + t2 + t3;

    auto tPrev = t[index];
    auto tCur = t[index + 1];
    auto tDiff = tCur - tPrev;
    return (y * tDiff) + tPrev;
}

template <typename FLOAT_TYPE> FLOAT_TYPE ParameterizationBezierApproximation<FLOAT_TYPE>::length() const {
    return totalLength;
}

template <typename FLOAT_TYPE>
ParameterizationBezierApproximation<FLOAT_TYPE> *
Bezier<FLOAT_TYPE>::parameterization_bezier_approximation(int numSegments, int numSamples) const {
    if (numSegments < 1) {
        return nullptr;
    }

    auto result = new ParameterizationBezierApproximation<FLOAT_TYPE>();
    result->totalLength = arc_length_legendre_gauss(1.0, numSamples);
    result->t.push_back(0.0);

    auto *legendreGauss = parameterization_subdivision_legendre_gauss(numSamples);

    auto dStep = 1.0 / static_cast<FLOAT_TYPE>(numSegments);
    for (int i = 0; i < numSegments; i++) {
        auto dPrev = static_cast<FLOAT_TYPE>(i) * dStep;
        auto dCur = static_cast<FLOAT_TYPE>(i + 1) * dStep;
        auto dDiff = dCur - dPrev;

        auto tPrev = result->t.back();
        auto tCur = legendreGauss->evaluate(dCur * result->totalLength);
        auto tDiff = tCur - tPrev;

        FLOAT_TYPE sample1 = (dPrev + dDiff * (1.0 / 3.0)) * result->totalLength;
        auto s1over3 = legendreGauss->evaluate(sample1);
        auto s1over3Scaled = (s1over3 - tPrev) / tDiff;

        FLOAT_TYPE sample2 = (dPrev + dDiff * (2.0 / 3.0)) * result->totalLength;
        auto s2over3 = legendreGauss->evaluate(sample2);
        auto s2over3Scaled = (s2over3 - tPrev) / tDiff;

        auto y1 = (18.0 * s1over3Scaled - 9.0 * s2over3Scaled + 2.0) / 6.0;
        auto y2 = (-9.0 * s1over3Scaled + 18.0 * s2over3Scaled - 5.0) / 6.0;

        result->y1.push_back(y1);
        result->y2.push_back(y2);
        result->t.push_back(tCur);
    }

    return result;
}

template <typename FLOAT_TYPE>
FLOAT_TYPE ParameterizationSubdivisionBezierApproximation<FLOAT_TYPE>::evaluate(FLOAT_TYPE d) const {
    FLOAT_TYPE t0 = 0.0;
    FLOAT_TYPE t1 = 1.0;
    for (int i = 0; i < depth; i++) {
        FLOAT_TYPE t = (t0 + t1) / 2.0;
        auto l = arcLength.evaluate(t);
        if (d < l) {
            t1 = t;
        } else if (d > l) {
            t0 = t;
        } else {
            break;
        }
    }
    return (t0 + t1) / 2.0;
}

template <typename FLOAT_TYPE> FLOAT_TYPE ParameterizationSubdivisionBezierApproximation<FLOAT_TYPE>::length() const {
    return arcLength.totalLength;
}

template <typename FLOAT_TYPE> FLOAT_TYPE ParameterizationAdaptive<FLOAT_TYPE>::evaluate(FLOAT_TYPE d) const {
    FLOAT_TYPE t0 = 0.0;
    FLOAT_TYPE t1 = 1.0;
    for (int i = 0; i < depth; i++) {
        FLOAT_TYPE t = (t0 + t1) / 2.0;
        auto l = b.arc_length_adaptive_subdivision(t);
        if (d < l) {
            t1 = t;
        } else if (d > l) {
            t0 = t;
        } else {
            break;
        }
    }
    return (t0 + t1) / 2.0;
}

template <typename FLOAT_TYPE> FLOAT_TYPE ParameterizationAdaptive<FLOAT_TYPE>::length() const {
    return b.arc_length_adaptive_subdivision();
}

template <typename FLOAT_TYPE> Hermite<FLOAT_TYPE> Bezier<FLOAT_TYPE>::to_hermite() const {
    auto result = Hermite<FLOAT_TYPE>(   //
          points[0],                     //
          points[3],                     //
          3.0 * (points[1] - points[0]), //
          3.0 * (points[3] - points[2])  //
    );
    return result;
}

template <typename FLOAT_TYPE>
Bezier<FLOAT_TYPE> Bezier<FLOAT_TYPE>::fit_to_points(v3<FLOAT_TYPE> p0, v3<FLOAT_TYPE> p1, v3<FLOAT_TYPE> p2,
                                                     v3<FLOAT_TYPE> p3, FLOAT_TYPE t1, FLOAT_TYPE t2) {
    Bezier<FLOAT_TYPE> result = {};
    result.points[0] = p0;
    result.points[3] = p3;

    // auxiliary values
    FLOAT_TYPE tt1 = 1 - t1;
    FLOAT_TYPE tt2 = 1 - t2;

    // Solution of linear equation system
    FLOAT_TYPE a11 = 3 * tt1 * tt1 * t1;
    FLOAT_TYPE a12 = 3 * tt1 * t1 * t1;
    FLOAT_TYPE a21 = 3 * tt2 * tt2 * t2;
    FLOAT_TYPE a22 = 3 * tt2 * t2 * t2;
    FLOAT_TYPE Det = a11 * a22 - a12 * a21;

    v3<FLOAT_TYPE> b1 = p1 - p0 * tt1 * tt1 * tt1 - p3 * t1 * t1 * t1;
    v3<FLOAT_TYPE> b2 = p2 - p0 * tt2 * tt2 * tt2 - p3 * t2 * t2 * t2;
    result.points[1] = (b1 * a22 - b2 * a12) / Det;
    result.points[2] = (-1 * b1 * a21 + b2 * a11) / Det;

    return result;
}

template struct v3<float>;
template struct aabb<float>;
template struct Bezier<float>;
template struct ArcLengthBezierApproximation<float>;
template struct ParameterizationRegression<float>;
template struct ParameterizationAdaptive<float>;
template struct ParameterizationBezierApproximation<float>;
template struct ParameterizationSubdivisionLegendreGauss<float>;
template struct ParameterizationSubdivisionBezierApproximation<float>;

template struct v3<double>;
template struct aabb<double>;
template struct Bezier<double>;
template struct ArcLengthBezierApproximation<double>;
template struct ParameterizationRegression<double>;
template struct ParameterizationAdaptive<double>;
template struct ParameterizationBezierApproximation<double>;
template struct ParameterizationSubdivisionLegendreGauss<double>;
template struct ParameterizationSubdivisionBezierApproximation<double>;
