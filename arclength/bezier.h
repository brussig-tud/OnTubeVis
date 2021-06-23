#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "v3.h"

template <typename FLOAT_TYPE> struct aabb {
    v3<FLOAT_TYPE> min = {};
    v3<FLOAT_TYPE> max = {};
    FLOAT_TYPE width() const { return std::abs(max.x - min.x); }
    FLOAT_TYPE height() const { return std::abs(max.y - min.y); }
    FLOAT_TYPE depth() const { return std::abs(max.z - min.z); }
};

template <typename FLOAT_TYPE> struct ArcLengthBezierApproximation {
    FLOAT_TYPE s0y1;
    FLOAT_TYPE s0y2;
    FLOAT_TYPE s1y1;
    FLOAT_TYPE s1y2;
    FLOAT_TYPE totalLength0 = 0.0;
    FLOAT_TYPE totalLength1 = 0.0;
    FLOAT_TYPE tMid = 0.0;
    bool isTwoSpan = false;

    FLOAT_TYPE evaluate(FLOAT_TYPE t) const;
};

template <typename FLOAT_TYPE> struct ParameterizationRegression {
    std::array<FLOAT_TYPE, 4> coefficients = {};
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const {
        return coefficients[0] +         //
               coefficients[1] * d +     //
               coefficients[2] * d * d + //
               coefficients[3] * d * d * d;
    }
};

template <typename FLOAT_TYPE> struct Bezier;
template <typename FLOAT_TYPE> struct ParameterizationSubdivisionLegendreGauss {
    Bezier<FLOAT_TYPE> b;
    int depth;
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const;
};

template <typename FLOAT_TYPE> struct ParameterizationSubdivisionBezierApproximation {
    ArcLengthBezierApproximation<FLOAT_TYPE> arcLength;
    int depth;
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const;
};

template <typename FLOAT_TYPE> struct Hermite;

template <typename FLOAT_TYPE> struct Bezier {
    std::array<v3<FLOAT_TYPE>, 4> points = {};

    /**
     * Evaluate the given curve at the given parameter point t.
     */
    v3<FLOAT_TYPE> evaluate(FLOAT_TYPE t) const;

    /**
     * Calculate the derivative of the given curve at the given parameter point t.
     */
    v3<FLOAT_TYPE> derivative(FLOAT_TYPE t) const;

    /**
     * Calculate the axis-aligned bounding box of the give Bezier curve.
     */
    aabb<FLOAT_TYPE> bounding_box() const;

    /**
     * Moves and rotates the curve, so that it starts at (0,0) and ends on the x-axis.
     * @return a new axis-aligned bezier curve
     */
    Bezier<FLOAT_TYPE> axis_align() const;

    /**
     * Split the Bezier curve at the parameter point t.
     */
    std::pair<Bezier<FLOAT_TYPE>, Bezier<FLOAT_TYPE>> split(FLOAT_TYPE t) const;

    /**
     * Calculate the inflection points of this curve. Returns the t values of the
     * inflection points.
     */
    std::vector<FLOAT_TYPE> get_inflection_points() const;

    std::vector<FLOAT_TYPE> get_inflection_points_of_length_function() const;

    /** --------------------------------------------------------------------- */

    /**
     * Create numSamples amount of slices and add them all up to calculate the
     * integral that describes the length of the given bezier curve.
     */
    FLOAT_TYPE arc_length_legendre_gauss(FLOAT_TYPE t = 1.0, int numSamples = 20) const;

    /**
     * Divide the curve into evenly spaced segments and sum up the straight line
     * distance of those segments.
     */
    FLOAT_TYPE arc_length_even_segments(FLOAT_TYPE t = 1.0, int numSegments = 100) const;

    /**
     * Calculates a bezier curve that approximates the arc length of the original
     * bezier curve.
     * https://www.visgraf.impa.br/sibgrapi96/trabs/pdf/a14.pdf
     */
    ArcLengthBezierApproximation<FLOAT_TYPE> arc_length_bezier_approximation(int numSamples = 100) const;

    /** --------------------------------------------------------------------- */

    /**
     * Calculates an arc length parameterization for this curve by sampling the
     * curve and using the samples to do a cubic regression.
     * NOTE: This is very inaccurate, use
     * parameterization_subdivision_bezier_approximation instead.
     */
    ParameterizationRegression<FLOAT_TYPE> parameterization_regression(int order = 3, int numSamples = 100,
                                                                       int numLegendreSamples = 100) const;

    /**
     * Creates an arc length parameterization that does a binary search to find the correct parameter t for a given
     * arc length.
     * Uses the Legendre-Gauss algorithm to calculate the length at each point.
     */
    ParameterizationSubdivisionLegendreGauss<FLOAT_TYPE>
    parameterization_subdivision_legendre_gauss(int depth = 100) const {
        auto result = ParameterizationSubdivisionLegendreGauss<FLOAT_TYPE>();
        result.depth = depth;
        result.b = *this;
        return result;
    }

    /**
     * Creates an arc length parameterization that does a binary search to find the correct parameter t for a given
     * arc length.
     * Uses the Bezier-Approximation algorithm to calculate the length at each point.
     */
    ParameterizationSubdivisionBezierApproximation<FLOAT_TYPE>
    parameterization_subdivision_bezier_approximation(int depth = 100) const {
        auto result = ParameterizationSubdivisionBezierApproximation<FLOAT_TYPE>();
        result.depth = depth;
        result.arcLength = arc_length_bezier_approximation();
        return result;
    }

    void to_csv(const std::string &fileName, int numTestPoints = 100, bool evenPointDistribution = false) const;
    Hermite<FLOAT_TYPE> to_hermite() const;

    /**
     * Fits a bezier curve to the given points.
     * @param p0 start point of the curve
     * @param p1 point on the curve at parameter value t1
     * @param p2 point on the curve at parameter value t2
     * @param p3 end point of the curve
     * @param t1/t2 values of parameter t for points p1 and p2 respectively
     * @return new bezier curve that fits the given points
     */
    static Bezier<FLOAT_TYPE> fit_to_points(v3<FLOAT_TYPE> p0, v3<FLOAT_TYPE> p1, v3<FLOAT_TYPE> p2, v3<FLOAT_TYPE> p3,
                                            FLOAT_TYPE t1, FLOAT_TYPE t2);
};
