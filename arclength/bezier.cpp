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

template <typename FLOAT_TYPE> FLOAT_TYPE arc_length_adaptive_subdivision(Bezier<FLOAT_TYPE> b, FLOAT_TYPE epsilon) {
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
Bezier<FLOAT_TYPE>::parameterization_regression(const int order, const int numSamples,
                                                const int numLegendreSamples) const {
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

template <typename FLOAT_TYPE> std::vector<FLOAT_TYPE> Bezier<FLOAT_TYPE>::get_inflection_points() const {
    auto P1 = points[0];
    auto C1 = points[1];
    auto C2 = points[2];
    auto P2 = points[3];
    v3<FLOAT_TYPE> a = C1 - P1;
    v3<FLOAT_TYPE> b = C2 - C1 - a;
    v3<FLOAT_TYPE> c = P2 - C2 - a - (2.0 * b);

    v3<FLOAT_TYPE> i = {};
    i.x = b.y * c.z - b.z * c.y;
    i.y = b.z * c.x - b.x * c.z;
    i.z = b.x * c.y - b.y * c.x;

    v3<FLOAT_TYPE> j = {};
    j.x = a.y * c.z - a.z * c.y;
    j.y = a.z * c.x - a.x * c.z;
    j.z = a.x * c.y - a.y * c.x;

    v3<FLOAT_TYPE> k = {};
    k.x = a.y * b.z - a.z * b.y;
    k.y = a.z * b.x - a.x * b.z;
    k.z = a.x * b.y - a.y * b.x;

    auto result = std::vector<FLOAT_TYPE>();
    FLOAT_TYPE t0;
    FLOAT_TYPE t1;
    bool useT0T1 = true;
    if (is_equal(i.x, static_cast<FLOAT_TYPE>(0.0))) {
        if (is_equal(j.x, static_cast<FLOAT_TYPE>(0.0)) && !is_equal(k.x, static_cast<FLOAT_TYPE>(0.0))) {
            return result;
        } else if (is_equal(j.x, static_cast<FLOAT_TYPE>(0.0)) && is_equal(k.x, static_cast<FLOAT_TYPE>(0.0))) {
            useT0T1 = false;
        } else {
            t0 = -k.x / j.x;
            t1 = t0;
        }
    } else {
        t0 = (-j.x + std::sqrt(j.x * j.x - 4.0 * i.x * k.x)) / (2.0 * i.x);
        t1 = (-j.x - std::sqrt(j.x * j.x - 4.0 * i.x * k.x)) / (2.0 * i.x);
    }

    FLOAT_TYPE t2;
    FLOAT_TYPE t3;
    bool useT2T3 = true;
    if (is_equal(i.y, static_cast<FLOAT_TYPE>(0.0))) {
        if (is_equal(j.y, static_cast<FLOAT_TYPE>(0.0)) && !is_equal(k.y, static_cast<FLOAT_TYPE>(0.0))) {
            return result;
        } else if (is_equal(j.y, static_cast<FLOAT_TYPE>(0.0)) && is_equal(k.y, static_cast<FLOAT_TYPE>(0.0))) {
            useT2T3 = false;
        } else {
            t2 = -k.y / j.y;
            t3 = t2;
        }
    } else {
        t2 = (-j.y + std::sqrt(j.y * j.y - 4.0 * i.y * k.y)) / (2.0 * i.y);
        t3 = (-j.y - std::sqrt(j.y * j.y - 4.0 * i.y * k.y)) / (2.0 * i.y);
    }

    FLOAT_TYPE t4;
    FLOAT_TYPE t5;
    bool useT4T5 = true;
    if (is_equal(i.z, 0.0)) {
        if (is_equal(j.z, 0.0) && !is_equal(k.z, 0.0)) {
            return result;
        } else if (is_equal(j.z, 0.0) && is_equal(k.z, 0.0)) {
            useT4T5 = false;
        } else {
            t4 = -k.z / j.z;
            t5 = t4;
        }
    } else {
        t4 = (-j.z + std::sqrt(j.z * j.z - 4.0 * i.z * k.z)) / (2.0 * i.z);
        t5 = (-j.z - std::sqrt(j.z * j.z - 4.0 * i.z * k.z)) / (2.0 * i.z);
    }

    if (!useT0T1 && !useT2T3 && useT4T5) {
        result.push_back(t4);
        result.push_back(t5);
    } else if (!useT0T1 && useT2T3 && !useT4T5) {
        result.push_back(t2);
        result.push_back(t3);
    } else if (useT0T1 && !useT2T3 && !useT4T5) {
        result.push_back(t0);
        result.push_back(t1);
    } else {

        if (is_equal(t0, t2) || !useT0T1 || !useT2T3) {
            if (!useT0T1) {
                t0 = t2;
            }
            if (is_equal(t0, t4) || is_equal(t0, t5) || !useT4T5) {
                if (t0 > 0.0 && t0 < 1.0) {
                    result.push_back(t0);
                }
            }
        }
        if (is_equal(t0, t3) || !useT0T1 || !useT2T3) {
            if (!useT0T1) {
                t0 = t3;
            }
            if (is_equal(t0, t4) || is_equal(t0, t5) || !useT4T5) {
                if (t0 > 0.0 && t0 < 1.0) {
                    result.push_back(t0);
                }
            }
        }

        if (is_equal(t1, t2) || !useT0T1 || !useT2T3) {
            if (!useT0T1) {
                t1 = t2;
            }
            if (is_equal(t1, t4) || is_equal(t1, t5) || !useT4T5) {
                if (t1 > 0.0 && t1 < 1.0) {
                    result.push_back(t1);
                }
            }
        }
        if (is_equal(t1, t3) || !useT0T1 || !useT2T3) {
            if (!useT0T1) {
                t1 = t3;
            }
            if (is_equal(t1, t4) || is_equal(t1, t5) || !useT4T5) {
                if (t1 > 0.0 && t1 < 1.0) {
                    result.push_back(t1);
                }
            }
        }
    }

    // removing duplicates
    for (int i = result.size() - 1; i >= 0; i--) {
        bool should_remove = false;
        for (int j = 0; j < result.size(); j++) {
            if (i != j && is_equal(result[i], result[j])) {
                should_remove = true;
                break;
            }
        }
        if (should_remove) {
            result.erase(result.begin() + i);
        }
    }

    for (const auto &t : result) {
        std::cout << t << std::endl;
    }
    return result;
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
std::vector<FLOAT_TYPE> Bezier<FLOAT_TYPE>::get_inflection_points_of_length_function() const {
    // TODO this function seems correct, but has not been tested extensively

    auto a = 9.0 * points[1] - 3.0 * points[0] - 9.0 * points[2] + 3.0 * points[3];
    auto b = -12.0 * points[1] + 6.0 * points[0] + 6.0 * points[2];
    auto c = 3.0 * points[1] - 3.0 * points[0];

    auto t0 = (b * -1.0 + std::sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    auto t1 = (b * -1.0 - std::sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    auto t2 = (-1.0 * c) / b;

    auto n = 6.0 * points[2] - 12.0 * points[1] + 6.0 * points[0];
    auto m = -18.0 * points[2] + 18.0 * points[1] - 6.0 * points[0] + 6.0 * points[3];
    auto t3 = (-1.0 * n) / m;

    const v3<FLOAT_TYPE> zero = v3<FLOAT_TYPE>(0.0, 0.0, 0.0);
    auto m_equal_zero = is_equal(m, zero);
    auto n_equal_zero = is_equal(n, zero);
    auto a_equal_zero = is_equal(a, zero);
    auto b_equal_zero = is_equal(b, zero);
    auto c_equal_zero = is_equal(c, zero);

    std::vector<FLOAT_TYPE> x_roots = {};
    bool x_has_infinite_roots = false;
    if (!m_equal_zero.x) {
        x_roots.push_back(t3.x);
    } else if (m_equal_zero.x && n_equal_zero.x) {
        x_has_infinite_roots = true;
    }
    if (a_equal_zero.x && b_equal_zero.x && c_equal_zero.x) {
        x_has_infinite_roots = true;
    } else if (a_equal_zero.x && !b_equal_zero.x) {
        x_roots.push_back(t2.x);
    } else if (!a_equal_zero.x) {
        FLOAT_TYPE d = b.x * b.x - 4.0 * a.x * c.x;
        if (d >= 0.0) {
            x_roots.push_back(t0.x);
            x_roots.push_back(t1.x);
        }
    }

    std::vector<FLOAT_TYPE> y_roots = {};
    bool y_has_infinite_roots = false;
    if (!m_equal_zero.y) {
        y_roots.push_back(t3.y);
    } else if (m_equal_zero.y && n_equal_zero.y) {
        y_has_infinite_roots = true;
    }
    if (a_equal_zero.y && b_equal_zero.y && c_equal_zero.y) {
        y_has_infinite_roots = true;
    } else if (a_equal_zero.y && !b_equal_zero.y) {
        y_roots.push_back(t2.y);
    } else if (!a_equal_zero.y && b.y * b.y - 4.0 * a.y * c.y >= 0.0) {
        y_roots.push_back(t0.y);
        y_roots.push_back(t1.y);
    }

    std::vector<FLOAT_TYPE> z_roots = {};
    bool z_has_infinite_roots = false;
    if (!m_equal_zero.z) {
        z_roots.push_back(t3.z);
    } else if (m_equal_zero.z && n_equal_zero.z) {
        z_has_infinite_roots = true;
    }
    if (a_equal_zero.z && b_equal_zero.z && c_equal_zero.z) {
        z_has_infinite_roots = true;
    } else if (a_equal_zero.z && !b_equal_zero.z) {
        z_roots.push_back(t2.z);
    } else if (!a_equal_zero.z && b.z * b.z - 4.0 * a.z * c.z >= 0.0) {
        z_roots.push_back(t0.z);
        z_roots.push_back(t1.z);
    }

    if (x_has_infinite_roots && y_has_infinite_roots && z_has_infinite_roots) {
        // TODO what should we do in this case? Is this even possible?
        return {};
    } else if (x_has_infinite_roots && y_has_infinite_roots) {
        return z_roots;
    } else if (x_has_infinite_roots && z_has_infinite_roots) {
        return y_roots;
    } else if (y_has_infinite_roots && z_has_infinite_roots) {
        return x_roots;
    } else if (x_has_infinite_roots) {
        std::vector<FLOAT_TYPE> result = intersection(y_roots, z_roots);
        unique(result);
        return result;
    } else if (y_has_infinite_roots) {
        std::vector<FLOAT_TYPE> result = intersection(x_roots, z_roots);
        unique(result);
        return result;
    } else if (z_has_infinite_roots) {
        std::vector<FLOAT_TYPE> result = intersection(x_roots, y_roots);
        unique(result);
        return result;
    }

    auto tmp = intersection(x_roots, y_roots);
    auto result = intersection(tmp, z_roots);
    unique(result);
    return result;
}

template <typename FLOAT_TYPE>
ArcLengthBezierApproximation<FLOAT_TYPE> Bezier<FLOAT_TYPE>::arc_length_bezier_approximation(int numSamples) const {
    std::vector<FLOAT_TYPE> inflectionPoints = get_inflection_points_of_length_function();

    auto result = ArcLengthBezierApproximation<FLOAT_TYPE>();

    result.isTwoSpan = false; //inflectionPoints.size() >= 2;
    if (!result.isTwoSpan) {
        result.totalLength0 = arc_length_legendre_gauss(1.0, numSamples);
        const auto s1over3 = arc_length_legendre_gauss(1.0 / 3.0, numSamples) / result.totalLength0;
        const auto s2over3 = arc_length_legendre_gauss(2.0 / 3.0, numSamples) / result.totalLength0;

        result.s0y1 = (18.0 * s1over3 - 9.0 * s2over3 + 2.0) / 6.0;
        result.s0y2 = (-9.0 * s1over3 + 18.0 * s2over3 - 5.0) / 6.0;
        return result;
    }

#define DEBUG_BEZIER_APPROXIMATION 0
#if DEBUG_BEZIER_APPROXIMATION
    std::cout << "We have more than one inflection point on the length curve." << std::endl;
    std::cout << "Curve:" << std::endl;
    for (const auto &p : points) {
        std::cout << p.x << " " << p.y << std::endl;
    }
    std::cout << "Inflection points given as parameter t:" << std::endl;
    for (const auto &t : inflectionPoints) {
        std::cout << t << std::endl;
    }
#endif

    // TODO not sure about the next part. we have not encountered such a case
    //  yet
    FLOAT_TYPE t1 = inflectionPoints[0];
    FLOAT_TYPE t2 = inflectionPoints[1];
    result.tMid = (t1 + t2) / 2.0;

    {
        result.totalLength0 = arc_length_legendre_gauss(result.tMid, numSamples);
        const auto s1over3 = arc_length_legendre_gauss((1.0 / 3.0) * result.tMid, numSamples) / result.totalLength0;
        const auto s2over3 = arc_length_legendre_gauss((2.0 / 3.0) * result.tMid, numSamples) / result.totalLength0;

        result.s0y1 = (18.0 * s1over3 - 9.0 * s2over3 + 2.0) / 6.0;
        result.s0y2 = (-9.0 * s1over3 + 18.0 * s2over3 - 5.0) / 6.0;
    }

    {
        result.totalLength1 = arc_length_legendre_gauss(1.0, numSamples) - result.totalLength0;

        FLOAT_TYPE sample1 = result.tMid + ((1.0 - result.tMid) * (1.0 / 3.0));
        const auto t1over3 =
              (arc_length_legendre_gauss(sample1, numSamples) - result.totalLength0) / result.totalLength1;

        FLOAT_TYPE sample2 = result.tMid + ((1.0 - result.tMid) * (2.0 / 3.0));
        const auto t2over3 =
              (arc_length_legendre_gauss(sample2, numSamples) - result.totalLength0) / result.totalLength1;

        result.s1y1 = (18.0 * t1over3 - 9.0 * t2over3 + 2.0) / 6.0;
        result.s1y2 = (-9.0 * t1over3 + 18.0 * t2over3 - 5.0) / 6.0;
    }

    return result;
}

template <typename FLOAT_TYPE> Bezier<FLOAT_TYPE> ArcLengthBezierApproximation<FLOAT_TYPE>::get_curve() const {
    return {
          v3<FLOAT_TYPE>(0.0, 0.0, 0.0),
          v3<FLOAT_TYPE>(1.0 / 3.0, s0y1, 0.0),
          v3<FLOAT_TYPE>(2.0 / 3.0, s0y2, 0.0),
          v3<FLOAT_TYPE>(1.0, 1.0, 0.0),
    };
}

template <typename FLOAT_TYPE> FLOAT_TYPE ArcLengthBezierApproximation<FLOAT_TYPE>::evaluate(FLOAT_TYPE t) const {
    if (!isTwoSpan) {
        auto b = get_curve();
        const v3<FLOAT_TYPE> v = b.evaluate(t);
        return v.y * totalLength0;
    }

    if (t < tMid) {
        FLOAT_TYPE x1 = tMid * (1.0 / 3.0);
        FLOAT_TYPE x2 = tMid * (2.0 / 3.0);
        Bezier<FLOAT_TYPE> b = {
              v3<FLOAT_TYPE>(0.0, 0.0, 0.0),
              v3<FLOAT_TYPE>(x1, s0y1, 0.0),
              v3<FLOAT_TYPE>(x2, s0y2, 0.0),
              v3<FLOAT_TYPE>(1.0, 1.0, 0.0),
        };
        const v3<FLOAT_TYPE> v = b.evaluate(t);
        return v.y * totalLength0;
    } else {
        FLOAT_TYPE x1 = tMid + ((1.0 - tMid) * (1.0 / 3.0));
        FLOAT_TYPE x2 = tMid + ((1.0 - tMid) * (2.0 / 3.0));
        Bezier<FLOAT_TYPE> b = {
              v3<FLOAT_TYPE>(0.0, 0.0, 0.0),
              v3<FLOAT_TYPE>(x1, s1y1, 0.0),
              v3<FLOAT_TYPE>(x2, s1y2, 0.0),
              v3<FLOAT_TYPE>(1.0, 1.0, 0.0),
        };
        const v3<FLOAT_TYPE> v = b.evaluate(t - tMid);
        return (v.y * totalLength1) + totalLength0;
    }
}

template <typename FLOAT_TYPE> Bezier<FLOAT_TYPE> ParameterizationBezierApproximation<FLOAT_TYPE>::get_curve() const {
    return {
          v3<FLOAT_TYPE>(0.0, 0.0, 0.0),
          v3<FLOAT_TYPE>(1.0 / 3.0, s0y1, 0.0),
          v3<FLOAT_TYPE>(2.0 / 3.0, s0y2, 0.0),
          v3<FLOAT_TYPE>(1.0, 1.0, 0.0),
    };
}

template <typename FLOAT_TYPE>
FLOAT_TYPE ParameterizationBezierApproximation<FLOAT_TYPE>::evaluate(FLOAT_TYPE d) const {
    if (!isTwoSpan) {
        const auto b = get_curve();
        const auto v = b.evaluate(d / totalLength0);
        return v.y;
    }

    if (d < tMid) {
        FLOAT_TYPE x1 = tMid * (1.0 / 3.0);
        FLOAT_TYPE x2 = tMid * (2.0 / 3.0);
        Bezier<FLOAT_TYPE> b = {
              v3<FLOAT_TYPE>(0.0, 0.0, 0.0),
              v3<FLOAT_TYPE>(x1, s0y1, 0.0),
              v3<FLOAT_TYPE>(x2, s0y2, 0.0),
              v3<FLOAT_TYPE>(1.0, 1.0, 0.0),
        };
        const v3<FLOAT_TYPE> v = b.evaluate(d);
        return v.y * totalLength0;
    } else {
        FLOAT_TYPE x1 = tMid + ((1.0 - tMid) * (1.0 / 3.0));
        FLOAT_TYPE x2 = tMid + ((1.0 - tMid) * (2.0 / 3.0));
        Bezier<FLOAT_TYPE> b = {
              v3<FLOAT_TYPE>(0.0, 0.0, 0.0),
              v3<FLOAT_TYPE>(x1, s1y1, 0.0),
              v3<FLOAT_TYPE>(x2, s1y2, 0.0),
              v3<FLOAT_TYPE>(1.0, 1.0, 0.0),
        };
        const v3<FLOAT_TYPE> v = b.evaluate(d - tMid);
        return (v.y * totalLength1) + totalLength0;
    }
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
    return arcLength.totalLength0 + arcLength.totalLength1;
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

template <typename FLOAT_TYPE>
ParameterizationBezierApproximation<FLOAT_TYPE> *
Bezier<FLOAT_TYPE>::parameterization_bezier_approximation(int numSamples) const {
    std::vector<FLOAT_TYPE> inflectionPoints = get_inflection_points_of_length_function();

    auto result = new ParameterizationBezierApproximation<FLOAT_TYPE>();

    result->isTwoSpan = inflectionPoints.size() >= 2;
    if (!result->isTwoSpan) {
        result->totalLength0 = arc_length_legendre_gauss(1.0, numSamples);
        const auto s1over3 =
              parameterization_subdivision_legendre_gauss(numSamples)->evaluate(1.0 / 3.0 * result->totalLength0);
        const auto s2over3 =
              parameterization_subdivision_legendre_gauss(numSamples)->evaluate(2.0 / 3.0 * result->totalLength0);

        result->s0y1 = (18.0 * s1over3 - 9.0 * s2over3 + 2.0) / 6.0;
        result->s0y2 = (-9.0 * s1over3 + 18.0 * s2over3 - 5.0) / 6.0;
        return result;
    }

#define DEBUG_BEZIER_APPROXIMATION 0
#if DEBUG_BEZIER_APPROXIMATION
    std::cout << "We have more than one inflection point on the length curve." << std::endl;
    std::cout << "Curve:" << std::endl;
    for (const auto &p : points) {
        std::cout << p.x << " " << p.y << std::endl;
    }
    std::cout << "Inflection points given as parameter t:" << std::endl;
    for (const auto &t : inflectionPoints) {
        std::cout << t << std::endl;
    }
#endif

    // TODO not sure about the next part. we have not encountered such a case yet
    FLOAT_TYPE t1 = inflectionPoints[0];
    FLOAT_TYPE t2 = inflectionPoints[1];
    result->tMid = (t1 + t2) / 2.0;

    {
        result->totalLength0 = arc_length_legendre_gauss(result->tMid, numSamples);
        const auto s1over3 = arc_length_legendre_gauss((1.0 / 3.0) * result->tMid, numSamples) / result->totalLength0;
        const auto s2over3 = arc_length_legendre_gauss((2.0 / 3.0) * result->tMid, numSamples) / result->totalLength0;

        result->s0y1 = (18.0 * s1over3 - 9.0 * s2over3 + 2.0) / 6.0;
        result->s0y2 = (-9.0 * s1over3 + 18.0 * s2over3 - 5.0) / 6.0;
    }

    {
        result->totalLength1 = arc_length_legendre_gauss(1.0, numSamples) - result->totalLength0;

        FLOAT_TYPE sample1 = result->tMid + ((1.0 - result->tMid) * (1.0 / 3.0));
        const auto t1over3 =
              (arc_length_legendre_gauss(sample1, numSamples) - result->totalLength0) / result->totalLength1;

        FLOAT_TYPE sample2 = result->tMid + ((1.0 - result->tMid) * (2.0 / 3.0));
        const auto t2over3 =
              (arc_length_legendre_gauss(sample2, numSamples) - result->totalLength0) / result->totalLength1;

        result->s1y1 = (18.0 * t1over3 - 9.0 * t2over3 + 2.0) / 6.0;
        result->s1y2 = (-9.0 * t1over3 + 18.0 * t2over3 - 5.0) / 6.0;
    }

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
