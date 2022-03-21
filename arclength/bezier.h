#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "v3.h"

template <typename FLOAT_TYPE> struct Bezier;

template <typename FLOAT_TYPE> struct aabb {
    v3<FLOAT_TYPE> min = {};
    v3<FLOAT_TYPE> max = {};
    FLOAT_TYPE width() const { return std::abs(max.x - min.x); }
    FLOAT_TYPE height() const { return std::abs(max.y - min.y); }
    FLOAT_TYPE depth() const { return std::abs(max.z - min.z); }
};

template <typename FLOAT_TYPE> struct ArcLengthBezierApproximation {
    std::vector<FLOAT_TYPE> y1 = {};
    std::vector<FLOAT_TYPE> y2 = {};
    std::vector<FLOAT_TYPE> lengths = {};
    FLOAT_TYPE totalLength = 0.0;

    FLOAT_TYPE evaluate(FLOAT_TYPE t) const;
};

template <typename FLOAT_TYPE> struct Parameterization {
    virtual FLOAT_TYPE evaluate(FLOAT_TYPE d) const = 0;
    virtual FLOAT_TYPE length() const = 0;
};

template <typename FLOAT_TYPE> struct ParameterizationNone : public Parameterization<FLOAT_TYPE> {
    FLOAT_TYPE totalLength = 0.0;
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const override { return d / totalLength; }
    FLOAT_TYPE length() const override { return totalLength; }
};

template <typename FLOAT_TYPE> struct ParameterizationSubdivisionLegendreGauss : public Parameterization<FLOAT_TYPE> {
    Bezier<FLOAT_TYPE> b;
    int depth;
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const override;
    FLOAT_TYPE length() const override;
};

template <typename FLOAT_TYPE> struct ParameterizationAdaptive : public Parameterization<FLOAT_TYPE> {
    Bezier<FLOAT_TYPE> b;
    int depth;
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const override;
    FLOAT_TYPE length() const override;
};

template <typename FLOAT_TYPE>
struct ParameterizationSubdivisionBezierApproximation : public Parameterization<FLOAT_TYPE> {
    ArcLengthBezierApproximation<FLOAT_TYPE> arcLength;
    int depth;
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const override;
    FLOAT_TYPE length() const override;
};

template <typename FLOAT_TYPE> struct ParameterizationBezierApproximation : public Parameterization<FLOAT_TYPE> {
    std::vector<FLOAT_TYPE> y1 = {};
    std::vector<FLOAT_TYPE> y2 = {};
    std::vector<FLOAT_TYPE> t = {};
    FLOAT_TYPE totalLength = 0.0;

    FLOAT_TYPE evaluate(FLOAT_TYPE d) const override;
    FLOAT_TYPE length() const override;
};

template <typename FLOAT_TYPE> struct ParameterizationRegression : public Parameterization<FLOAT_TYPE> {
    std::vector<FLOAT_TYPE> coefficients = {};
    FLOAT_TYPE _length = 0.0;
    FLOAT_TYPE evaluate(FLOAT_TYPE d) const override {
        FLOAT_TYPE value = 0;
        FLOAT_TYPE x = 1;
        for (int i = 0; i < coefficients.size(); i++) {
            value += coefficients[i] * x;
            x *= d;
        }
        return value;
    }
    FLOAT_TYPE length() const override { return _length; }
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
     * Length of the chord of this curve.
     */
    FLOAT_TYPE length_chord() const;

    /**
     * Length of the control polygon of this curve.
     */
    FLOAT_TYPE length_control_polygon() const;

    /** --------------------------------------------------------------------- */

    /**
     * Create numSamples amount of slices and add them all up to calculate the
     * integral that describes the length of the given bezier curve.
     */
    FLOAT_TYPE arc_length_legendre_gauss(FLOAT_TYPE t = 1, int numSamples = 20) const;

    /**
     * Divide the curve into evenly spaced segments and sum up the straight line
     * distance of those segments.
     */
    FLOAT_TYPE arc_length_even_subdivision(FLOAT_TYPE t = 1, int numSegments = 100) const;

    /**
     * Divide the curve into evenly spaced segments and sum up the straight line
     * distance of those segments.
     */
    FLOAT_TYPE arc_length_adaptive_subdivision(FLOAT_TYPE t = 1, FLOAT_TYPE epsilon = 0.0001F) const;

    /**
     * Calculates a bezier curve that approximates the arc length of the original curve.
     * https://www.visgraf.impa.br/sibgrapi96/trabs/pdf/a14.pdf
     */
    ArcLengthBezierApproximation<FLOAT_TYPE> arc_length_bezier_approximation(int numSegments,
                                                                             int numSamples = 100) const;

    /** --------------------------------------------------------------------- */

    /**
     * Calculates the length of the curve and creates a no-op parameterization.
     */
    ParameterizationNone<FLOAT_TYPE> *parameterization_none() const {
        auto result = new ParameterizationNone<FLOAT_TYPE>();
        result->totalLength = arc_length_legendre_gauss(1.0, 100);
        return result;
    }

    /**
     * Calculates an arc length parameterization for this curve by sampling the
     * curve and using the samples to do a cubic regression.
     * NOTE: This is very inaccurate, use
     * parameterization_subdivision_bezier_approximation instead.
     */
    ParameterizationRegression<FLOAT_TYPE> *parameterization_regression(int order = 3, int numSamples = -1,
                                                                        int numLegendreSamples = 100) const;

    Parameterization<FLOAT_TYPE> *parameterization_subdivision_adaptive(int depth = 100) const {
        auto result = new ParameterizationAdaptive<FLOAT_TYPE>();
        result->depth = depth;
        result->b = *this;
        return result;
    }

    /**
     * Creates an arc length parameterization that does a binary search to find the correct parameter t for a given
     * arc length.
     * Uses the Legendre-Gauss algorithm to calculate the length at each point.
     */
    ParameterizationSubdivisionLegendreGauss<FLOAT_TYPE> *
    parameterization_subdivision_legendre_gauss(int depth = 100) const {
        auto result = new ParameterizationSubdivisionLegendreGauss<FLOAT_TYPE>();
        result->depth = depth;
        result->b = *this;
        return result;
    }

    /**
     * Creates an arc length parameterization that does a binary search to find the correct parameter t for a given
     * arc length.
     * Uses the Bézier-Approximation algorithm to calculate the length at each point.
     */
    ParameterizationSubdivisionBezierApproximation<FLOAT_TYPE>
    parameterization_subdivision_bezier_approximation(int depth = 100, int numSegments = 3) const {
        ParameterizationSubdivisionBezierApproximation<FLOAT_TYPE> result;
        result.depth = depth;
        result.arcLength = arc_length_bezier_approximation(numSegments);
        return result;
    }

    [[nodiscard]] ParameterizationBezierApproximation<FLOAT_TYPE>
    parameterization_bezier_approximation(int numSegments, int numSamples = 100) const;

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
