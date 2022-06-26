
#ifndef __CURVEUTILS_H__
#define __CURVEUTILS_H__


//////
//
// Includes
//

// C++ STL
#include <limits>

// CGV framework
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>


// Vector type defintion
template <unsigned N, class T>
using Vec = cgv::math::fvec<T, N>;

// Matrix type defintion
template <unsigned N, unsigned M, class T>
using Mat = cgv::math::fmat<T, M, N>; // order of dimensions reversed as this code bridges from GLM (which uses the opposite convention of CGV)



//////
//
// Global constants
//

/** @brief 1 over 3. */
template <class Real>
constexpr Real _1o3 = Real(1./3.);
/** @brief 2 over 3. */
template <class Real>
constexpr Real _2o3 = Real(2./3.);
/** @brief Positive infinity. */
template <class Real>
constexpr Real _posInf = std::numeric_limits<Real>::infinity();
/** @brief Negative infinity. */
template <class Real>
constexpr Real _negInf = -std::numeric_limits<Real>::infinity();

/** @brief Zero vector. */
template <class Real, unsigned dimension>
const Vec<dimension, Real> _zero{0};

/** @brief Vector of ones. */
template <class Real, unsigned dimension>
const Vec<dimension, Real> _one{1};

/** @brief Vector of twos. */
template <class Real, unsigned dimension>
const Vec<dimension, Real> _two{2};

/** @brief Boolean true vector. */
template <unsigned dimension>
const Vec<dimension, bool> _true{true};

/** @brief Cubic Hermite to Bezier basis transform. */
template <class Real>
const Mat<4, 4, Real> h2b({
	1,          0,           0,  0,
	1, _1o3<Real>,           0,  0,
	0,          0, -_1o3<Real>,  1,
	0,          0,           0,  1
});
/** @brief Cubic Bezier to Hermite basis transform. */
template <class Real>
const Mat<4, 4, Real> b2h({
	 1,  0,  0,  0,
	-3,  3,  0,  0,
	 0,  0, -3,  3,
	 0,  0,  0,  1
});

/** @brief Cubic Bezier to Monomial basis transform. */
template <class Real>
const Mat<4, 4, Real> b2m({
	-1,  3, -3,  1,
	 3, -6,  3,  0,
	-3,  3,  0,  0,
	 1,  0,  0,  0
});
/** @brief Cubic Monomial to Bezier basis transform. */
template <class Real>
const Mat<4, 4, Real> m2b({
	0,          0,          0, 1,
	0,          0, _1o3<Real>, 1,
	0, _1o3<Real>, _2o3<Real>, 1,
	1,          1,          1, 1
});

/** @brief Quadratic Bezier to Monomial basis transform. */
template <class Real>
const Mat<3, 3, Real> b2m_2({
	 1, -2,  1,
	-2,  2,  0,
	 1,  0,  0
});
/** @brief Quadratic Monomial to Bezier basis transform. */
template <class Real>
const Mat<3, 3, Real> m2b_2({
	0,  0,         1,
	0,  Real(0.5), 1,
	1,  1,         1
});

/** @brief Linear Bezier to Monomial basis transform. */
template <class Real>
const Mat<2, 2, Real> b2m_1({
	-1, 1,
	 1, 0
});
/** @brief Linear Monomial to Bezier basis transform. */
template <class Real>
const Mat<2, 2, Real> m2b_1({
	0, 1,
	1, 1
});



//////
//
// Structs and typedefs
//

/** @brief Vector-valued cubic curve type. */
template<class Real, unsigned dimension>
using CubicCurve = Mat<4, dimension, Real>;

/** @brief 4D vector-valued cubic curve type. */
template<class Real>
using CubicCurve4 = CubicCurve<Real, 4>;

/** @brief 3D vector-valued cubic curve type. */
template<class Real>
using CubicCurve3 = CubicCurve<Real, 3>;

/** @brief 2D vector-valued cubic curve type. */
template<class Real>
using CubicCurve2 = CubicCurve<Real, 2>;

/** @brief Scalar-valued cubic curve type. */
template<class Real>
using CubicCurve1 = Vec<4, Real>;

/** @brief Vector-valued quadratic curve type. */
template<class Real, unsigned dimension>
using QuadraticCurve = Mat<3, dimension, Real>;

/** @brief 4D vector-valued quadratic curve type. */
template<class Real>
using QuadraticCurve4 = QuadraticCurve<Real, 4>;

/** @brief 3D vector-valued quadratic curve type. */
template<class Real>
using QuadraticCurve3 = QuadraticCurve<Real, 3>;

/** @brief 2D vector-valued quadratic curve type. */
template<class Real>
using QuadraticCurve2 = QuadraticCurve<Real, 2>;

/** @brief Scalar-valued quadratic curve type. */
template<class Real>
using QuadraticCurve1 =Vec<3, Real>;

/** @brief Vector-valued linear curve type. */
template<class Real, unsigned dimension>
using LinearCurve = Mat<2, dimension, Real>;

/** @brief 4D vector-valued linear curve type. */
template<class Real>
using LinearCurve4 = LinearCurve<Real, 4>;

/** @brief 3D vector-valued linear curve type. */
template<class Real>
using LinearCurve3 = LinearCurve<Real, 3>;

/** @brief 2D vector-valued linear curve type. */
template<class Real>
using LinearCurve2 = LinearCurve<Real, 2>;

/** @brief Scalar-valued linear curve type. */
template<class Real>
using LinearCurve1 = Vec<2, Real>;

/** @brief Struct representing the result of a linear root solver. */
template<class Real, unsigned dimension>
struct LinearRoots
{
	/** @brief The roots of each linear component curve. */
	Vec<dimension, Real> roots;

	/** @brief number of roots from each component curve. */
	Vec<dimension, unsigned> num;
};

/** @brief Roots of a 4D vector-valued linear curve. */
template<class Real>
using LinearRoots4 = LinearRoots<Real, 4>;

/** @brief Roots of a 3D vector-valued linear curve. */
template<class Real>
using LinearRoots3 = LinearRoots<Real, 3>;

/** @brief Roots of a 2D vector-valued linear curve. */
template<class Real>
using LinearRoots2 = LinearRoots<Real, 2>;

/** @brief The root of a scalar-valued linear curve. */
template<class Real>
using LinearRoots1 = LinearRoots<Real, 1>;



//////
//
// Functions
//

////
// Linear interpolation

#ifndef glm_glm
	/** @brief Drop-in replacements for some GLM functions. */
	namespace glm {

		/** @brief Replacement for the glm::mix function. */
		template<class T>
		inline T mix (const T &v0, const T &v1, float t)
		{
			return (1-t)*v0 + t*v1;
		}
	};
#endif


////
// Component function extraction

/**
 * @brief
 *		Extracts the i-th component function from the given 4D vector-valued cubic curve
 */
template <class Real>
CubicCurve1<Real> getComponentFkt (const CubicCurve4<Real> &curve, int i)
{
	return { curve[0][i], curve[1][i], curve[2][i], curve[3][i] };
}

/**
 * @brief
 *		Extracts the i-th component function from the given 3D vector-valued cubic curve
 */
template <class Real>
CubicCurve1<Real> getComponentFkt (const CubicCurve3<Real> &curve, int i)
{
	return { curve[0][i], curve[1][i], curve[2][i], curve[3][i] };
}

/**
 * @brief
 *		Extracts the i-th component function from the given 2D vector-valued cubic curve
 */
template <class Real>
CubicCurve1<Real> getComponentFkt (const CubicCurve2<Real> &curve, int i)
{
	return { curve[0][i], curve[1][i], curve[2][i], curve[3][i] };
}

/**
 * @brief
 *		Extracts the i-th component function from the given 4D vector-valued quadratic
 *		curve
 */
template <class Real>
QuadraticCurve1<Real> getComponentFkt(const QuadraticCurve4<Real> &curve, int i)
{
	return { curve[0][i], curve[1][i], curve[2][i] };
}

/**
 * @brief
 *		Extracts the i-th component function from the given 3D vector-valued quadratic
 *		curve
 */
template <class Real>
QuadraticCurve1<Real> getComponentFkt(const QuadraticCurve3<Real> &curve, int i)
{
	return { curve[0][i], curve[1][i], curve[2][i] };
}

/**
 * @brief
 *		Extracts the i-th component function from the given 2D vector-valued quadratic
 *		curve
 */
template <class Real>
QuadraticCurve1<Real> getComponentFkt(const QuadraticCurve2<Real> &curve, int i)
{
	return { curve[0][i], curve[1][i], curve[2][i] };
}


////
// Bezier to Hermite conversion

/**
 * @brief
 *		Converts a 4D vector-valued cubic Hermite curve defined by the given control
 *		points to a cubic Bezier curve.
 */
template <class Real>
CubicCurve4<Real> toBezier (
	const Vec<4,Real> &n0, const Vec<4,Real> &t0,
	const Vec<4,Real> &n1, const Vec<4,Real> &t1
)
{
	return { n0, n0 + _1o3<Real>*t0, n1 - _1o3<Real>*t1, n1 };
}

/**
 * @brief
 *		Converts a 3D vector-valued cubic Hermite curve defined by the given control
 *		points to a cubic Bezier curve.
 */
template <class Real>
CubicCurve3<Real> toBezier(
	const Vec<3, Real> &n0, const Vec<3, Real> &t0,
	const Vec<3, Real> &n1, const Vec<3, Real> &t1)
{
	return { n0, n0 + _1o3<Real>*t0, n1 - _1o3<Real>*t1, n1 };
}

/**
 * @brief
 *		Converts a 2D vector-valued cubic Hermite curve defined by the given control
 *		points to a cubic Bezier curve.
 */
template <class Real>
CubicCurve2<Real> toBezier (
	const Vec<2,Real> &n0, const Vec<2,Real> &t0,
	const Vec<2,Real> &n1, const Vec<2,Real> &t1
)
{
	return { n0, n0 + _1o3<Real>*t0, n1 - _1o3<Real>*t1, n1 };
}

/**
 * @brief
 *		Converts a scalar-valued cubic Hermite curve defined by the given control points
 *		to a cubic Bezier curve.
 */
template <class Real>
CubicCurve1<Real> toBezier (Real n0, Real t0, Real n1, Real t1)
{
	return { n0, n0 + _1o3<Real>*t0, n1 - _1o3<Real>*t1, n1 };
}

/*
 * @brief
 *		Converts a vector-valued cubic Hermite curve to a cubic Bezier curve, writing the
 *		control points to the output parameter.
 *
template <class CurveType>
void toBezier (
	CurveType &b,
	const typename CurveType::col_type &n0, const typename CurveType::col_type &t0,
	const typename CurveType::col_type &n1, const typename CurveType::col_type &t1
)
{
	b[0] = n0;
	b[1] = n0 + _1o3<typename CurveType::value_type>*t0;
	b[2] = n1 - _1o3<typename CurveType::value_type>*t1;
	b[3] = n1;
}*/

/**
 * @brief
 *		Converts a scalar-valued cubic Hermite curve to a cubic Bezier curve, writing the
 *		control points to the output parameter
 */
template <class Real>
void toBezier (CubicCurve1<Real> &b, Real n0, Real t0, Real n1, Real t1)
{
	b[0] = n0;
	b[1] = n0 + _1o3<Real>*t0;
	b[2] = n1 - _1o3<Real>*t1;
	b[3] = n1;
}

/** @brief Converts a cubic Hermite curve to a cubic Bezier curve. */
template <class CurveType>
CurveType toBezier (const CurveType &h)
{
	return h * h2b<typename CurveType::value_type>;
}

/**
 * @brief
 *		Converts a cubic Hermite curve to a cubic Bezier curve, writing the control
 *		points to the output parameter.
 */
template <class CurveType>
void toBezier (CurveType &b, const CurveType &h)
{
	b = h * h2b<typename CurveType::value_type>;
}


////
// Bezier to Hermite conversion

/**
 * @brief
 *		Converts a vector-valued cubic Bezier curve defined by the given control points
 *		to a cubic Hermite curve.
 */
template <class Real, unsigned dimension>
CubicCurve<Real, dimension> toHermite (
	const Vec<dimension,Real> &b0, const Vec<dimension,Real> &b1,
	const Vec<dimension,Real> &b2, const Vec<dimension,Real> &b3
)
{
	return { b0, Real(3)*(b1-b0), Real(3)*(b3-b2), b3 };
}

/**
 * @brief
 *		Converts a scalar-valued cubic Bezier curve defined by the given control points
 *		to a cubic Hermite curve.
 */
template <class Real>
CubicCurve1<Real> toHermite (Real b0, Real b1, Real b2, Real b3)
{
	return { b0, Real(3)*(b1 - b0), Real(3)*(b3 - b2), b3 };
}

/*
 * @brief
 *		Converts a vector-valued cubic Bezier curve to a cubic Hermite curve, writing the
 *		control points to the output parameter.
 *
template <class CurveType>
void toHermite (
	CurveType &h,
	const typename CurveType::col_type &b0, const typename CurveType::col_type &b1,
	const typename CurveType::col_type &b2, const typename CurveType::col_type &b3
)
{
	h[0] = b0;
	h[1] = typename CurveType::value_type(3)*(b1-b0);
	h[2] = typename CurveType::value_type(3)*(b3-b2);
	h[3] = b3;
}*/

/**
 * @brief
 *		Converts a scalar-valued cubic Bezier curve to a cubic Hermite curve, writing the
 *		control points to the output parameter
 */
template <class Real>
void toHermite (CubicCurve1<Real> &h, Real b0, Real b1, Real b2, Real b3)
{
	h[0] = b0;
	h[1] = Real(3)*(b1-b0);
	h[2] = Real(3)*(b3-b2);
	h[3] = b3;
}

/** @brief Converts a cubic Bezier curve to a cubic Hermite curve. */
template <class CurveType>
CurveType toHermite (const CurveType &b)
{
	return b * b2h<CurveType::value_type>;
}

/**
 * @brief
 *		Converts a cubic Bezier curve to a cubic Hermite curve, writing the control
 *		points to the output parameter.
 */
template <class CurveType>
void toHermite (CurveType &h, const CurveType &b)
{
	h = b * b2h<CurveType::value_type>;
}


////
// Cubic Bezier to Monomial conversion

/**
 * @brief
 *		Converts a 4D vector-valued cubic Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
CubicCurve4<Real> toMonomial (
	const Vec<4,Real> &b0, const Vec<4,Real> &b1,
	const Vec<4,Real> &b2, const Vec<4,Real> &b3
)
{
	return {         -b0 + Real(3)*b1 - Real(3)*b2 + b3,
	          Real(3)*b0 - Real(6)*b1 + Real(3)*b2,
	         Real(-3)*b0 + Real(3)*b1,
	                  b0 };
}

/**
 * @brief
 *		Converts a 3D vector-valued cubic Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
CubicCurve3<Real> toMonomial (
	const Vec<3,Real> &b0, const Vec<3,Real> &b1,
	const Vec<3,Real> &b2, const Vec<3,Real> &b3
)
{
	return {         -b0 + Real(3)*b1 - Real(3)*b2 + b3,
	          Real(3)*b0 - Real(6)*b1 + Real(3)*b2,
	         Real(-3)*b0 + Real(3)*b1,
	                  b0 };
}

/**
 * @brief
 *		Converts a 2D vector-valued cubic Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
CubicCurve2<Real> toMonomial (
	const Vec<2,Real> &b0, const Vec<2,Real> &b1,
	const Vec<2,Real> &b2, const Vec<2,Real> &b3
)
{
	return {         -b0 + Real(3)*b1 - Real(3)*b2 + b3,
	          Real(3)*b0 - Real(6)*b1 + Real(3)*b2,
	         Real(-3)*b0 + Real(3)*b1,
	                  b0 };
}

/**
 * @brief
 *		Converts a scalar-valued cubic Bezier curve defined by the given control points
 *		into canonical polynomial form.
 */
template <class Real>
CubicCurve1<Real> toMonomial (Real b0, Real b1, Real b2, Real b3)
{
	return {         -b0 + Real(3)*b1 - Real(3)*b2 + b3,
	          Real(3)*b0 - Real(6)*b1 + Real(3)*b2,
	         Real(-3)*b0 + Real(3)*b1,
	                  b0 };
}

/*
 * @brief
 *		Converts a vector-valued cubic Bezier curve into canonical polynomial form,
 *		writing the coefficient points to the output parameter.
 *
template <class CurveType>
void toMonomial (
	CurveType &m,
	const typename CurveType::col_type &b0, const typename CurveType::col_type &b1,
	const typename CurveType::col_type &b2, const typename CurveType::col_type &b3
)
{
	typedef typename CurveType::value_type Real;
	m[0] =         -b0 + Real(3)*b1 - Real(3)*b2 + b3;
	m[1] =  Real(3)*b0 - Real(6)*b1 + Real(3)*b2;
	m[2] = Real(-3)*b0 + Real(3)*b1;
	m[3] =          b0;
}*/

/**
 * @brief
 *		Converts a scalar-valued cubic Bezier curve into canonical polynomial form,
 *		writing the polynomial coefficients to the output parameter.
 */
template <class Real>
void toMonomial (CubicCurve1<Real> &m, Real b0, Real b1, Real b2, Real b3)
{
	m[0] =         -b0 + Real(3)*b1 - Real(3)*b2 + b3;
	m[1] =  Real(3)*b0 - Real(6)*b1 + Real(3)*b2;
	m[2] = Real(-3)*b0 + Real(3)*b1;
	m[3] =          b0;
}

/** @brief Converts a 4D cubic Bezier curve into canonical polynomial form. */
template <class Real>
CubicCurve4<Real> toMonomial (const CubicCurve4<Real> &b)
{
	return b * b2m<Real>;
}

/** @brief Converts a 3D cubic Bezier curve into canonical polynomial form. */
template <class Real>
CubicCurve3<Real> toMonomial (const CubicCurve3<Real> &b)
{
	return b * b2m<Real>;
}

/** @brief Converts a 2D cubic Bezier curve into canonical polynomial form. */
template <class Real>
CubicCurve2<Real> toMonomial (const CubicCurve2<Real> &b)
{
	return b * b2m<Real>;
}

/** @brief Converts a scalar cubic Bezier curve into canonical polynomial form. */
template <class Real>
CubicCurve1<Real> toMonomial (const CubicCurve1<Real> &b)
{
	return b * b2m<Real>;
}

/**
 * @brief
 *		Converts a 4D cubic Bezier curve into canonical polynomial form, writing the
 *		control points to the output parameter.
 */
template <class Real>
void toMonomial (CubicCurve4<Real> &m, const CubicCurve4<Real> &b)
{
	m = b * b2m<Real>;
}

/**
 * @brief
 *		Converts a 3D cubic Bezier curve into canonical polynomial form, writing the
 *		control points to the output parameter.
 */
template <class Real>
void toMonomial (CubicCurve3<Real> &m, const CubicCurve3<Real> &b)
{
	m = b * b2m<Real>;
}

/**
 * @brief
 *		Converts a 2D cubic Bezier curve into canonical polynomial form, writing the
 *		control points to the output parameter.
 */
template <class Real>
void toMonomial (CubicCurve2<Real> &m, const CubicCurve2<Real> &b)
{
	m = b * b2m<Real>;
}

/**
 * @brief
 *		Converts a scalar cubic Bezier curve into canonical polynomial form, writing the
 *		control points to the output parameter.
 */
template <class Real>
void toMonomial (CubicCurve1<Real> &m, const CubicCurve1<Real> &b)
{
	m = b * b2m<Real>;
}


////
// Quadratic Bezier to Monomial conversion

/**
 * @brief
 *		Converts a 4D vector-valued quadratic Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
QuadraticCurve4<Real> toMonomial (
	const Vec<4,Real> &b0, const Vec<4,Real> &b1, const Vec<4,Real> &b2
)
{
	return {          b0 - Real(2)*b1 + b2,
	         Real(-2)*b0 + Real(2)*b1,
	                  b0 };
}

/**
 * @brief
 *		Converts a 3D vector-valued quadratic Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
QuadraticCurve3<Real> toMonomial (
	const Vec<3,Real> &b0, const Vec<3,Real> &b1, const Vec<3,Real> &b2
)
{
	return {          b0 - Real(2)*b1 + b2,
	         Real(-2)*b0 + Real(2)*b1,
	                  b0 };
}

/**
 * @brief
 *		Converts a 2D vector-valued quadratic Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
QuadraticCurve2<Real> toMonomial (
	const Vec<2,Real> &b0, const Vec<2,Real> &b1, const Vec<2,Real> &b2
)
{
	return {          b0 - Real(2)*b1 + b2,
	         Real(-2)*b0 + Real(2)*b1,
	                  b0 };
}

/**
 * @brief
 *		Converts a scalar-valued quadratic Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
QuadraticCurve1<Real> toMonomial (Real b0, Real b1, Real b2)
{
	return {          b0 - Real(2)*b1 + b2,
	         Real(-2)*b0 + Real(2)*b1,
	                  b0 };
}

/*
 * @brief
 *		Converts a vector-valued quadratic Bezier curve into canonical polynomial form,
 *		writing the coefficient points to the output parameter.
 *
template <class CurveType>
void toMonomial (
	CurveType &m, const typename CurveType::col_type &b0,
	const typename CurveType::col_type &b1, const typename CurveType::col_type &b2
)
{
	typedef typename CurveType::value_type Real;
	m[0] =          b0 - Real(2)*b1 + b2;
	m[1] = Real(-2)*b0 + Real(2)*b1;
	m[2] =          b0;
}*/

/**
 * @brief
 *		Converts a scalar-valued quadratic Bezier curve into canonical polynomial form,
 *		writing the polynomial coefficients to the output parameter.
 */
template <class Real>
void toMonomial (QuadraticCurve1<Real> &m, Real b0, Real b1, Real b2)
{
	m[0] =          b0 - Real(2)*b1 + b2;
	m[1] = Real(-2)*b0 + Real(2)*b1;
	m[2] =          b0;
}

/**
 * @brief
 *		Converts a 4D vector-valued quadratic Bezier curve into canonical polynomial
 *		form.
 */
template <class Real>
QuadraticCurve4<Real> toMonomial (const QuadraticCurve4<Real> &b)
{
	return b * b2m_2<Real>;
}

/**
 * @brief
 *		Converts a 3D vector-valued quadratic Bezier curve into canonical polynomial
 *		form.
 */
template <class Real>
QuadraticCurve3<Real> toMonomial (const QuadraticCurve3<Real> &b)
{
	return b * b2m_2<Real>;
}

/**
 * @brief
 *		Converts a 2D vector-valued quadratic Bezier curve into canonical polynomial
 *		form.
 */
template <class Real>
QuadraticCurve2<Real> toMonomial (const QuadraticCurve2<Real> &b)
{
	return b * b2m_2<Real>;
}

/**
 * @brief Converts a scalar-valued quadratic Bezier curve into canonical polynomial form.
 */
template <class Real>
QuadraticCurve1<Real> toMonomial (const QuadraticCurve1<Real> &b)
{
	return b * b2m_2<Real>;
}

/**
 * @brief
 *		Converts a 4D vector-valued quadratic Bezier curve into canonical polynomial
 *		form, writing the coefficient points to the output parameter.
 */
template <class Real>
void toMonomial (QuadraticCurve4<Real> &m, const QuadraticCurve4<Real> &b)
{
	m = b * b2m_2<Real>;
}

/**
 * @brief
 *		Converts a 3D vector-valued quadratic Bezier curve into canonical polynomial
 *		form, writing the coefficient points to the output parameter.
 */
template <class Real>
void toMonomial (QuadraticCurve3<Real> &m, const QuadraticCurve3<Real> &b)
{
	m = b * b2m_2<Real>;
}

/**
 * @brief
 *		Converts a 2D vector-valued quadratic Bezier curve into canonical polynomial
 *		form, writing the coefficient points to the output parameter.
 */
template <class Real>
void toMonomial (QuadraticCurve2<Real> &m, const QuadraticCurve2<Real> &b)
{
	m = b * b2m_2<Real>;
}

/**
 * @brief
 *		Converts a scalar-valued quadratic Bezier curve into canonical polynomial form,
 *		writing the polynomial coefficients to the output parameter.
 */
template <class Real>
void toMonomial (QuadraticCurve1<Real> &m, const QuadraticCurve1<Real> &b)
{
	m = b * b2m_2<Real>;
}


////
// Linear Bezier to Monomial conversion

/**
 * @brief
 *		Converts a 4D vector-valued linear Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
LinearCurve4<Real> toMonomial (const Vec<4,Real> &b0, const Vec<4,Real> &b1) {
	return {/* slope: */ b1 - b0,  /* offset: */ b0 };
}

/**
 * @brief
 *		Converts a 3D vector-valued linear Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
LinearCurve3<Real> toMonomial (const Vec<3,Real> &b0, const Vec<3,Real> &b1) {
	return {/* slope: */ b1 - b0,  /* offset: */ b0 };
}

/**
 * @brief
 *		Converts a 2D vector-valued linear Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
LinearCurve2<Real> toMonomial (const Vec<2,Real> &b0, const Vec<2,Real> &b1) {
	return {/* slope: */ b1 - b0,  /* offset: */ b0 };
}

/**
 * @brief
 *		Converts a scalar-valued linear Bezier curve defined by the given control
 *		points into canonical polynomial form.
 */
template <class Real>
LinearCurve1<Real> toMonomial (const Real b0, const Real b1) {
	return {/* slope: */ b1 - b0,  /* offset: */ b0 };
}

/**
 * @brief
 *		Converts a scalar-valued linear Bezier curve into canonical polynomial form,
 *		writing the polynomial coefficients to the output parameter.
 */
template <class Real>
void toMonomial (LinearCurve1<Real> &m, Real b0, Real b1)
{
	m[0] = b1 - b0; // slope
	m[1] = b0;      // offset
}

/**
 * @brief
 *		Converts a 4D vector-valued quadratic Bezier curve into canonical polynomial
 *		form.
 */
template <class Real>
LinearCurve4<Real> toMonomial (const LinearCurve4<Real> &b)
{
	return b * b2m_1<Real>;
}

/**
 * @brief
 *		Converts a 3D vector-valued quadratic Bezier curve into canonical polynomial
 *		form.
 */
template <class Real>
LinearCurve3<Real> toMonomial (const LinearCurve3<Real> &b)
{
	return b * b2m_1<Real>;
}

/**
 * @brief
 *		Converts a 2D vector-valued quadratic Bezier curve into canonical polynomial
 *		form.
 */
template <class Real>
LinearCurve2<Real> toMonomial (const LinearCurve2<Real> &b)
{
	return b * b2m_1<Real>;
}

/**
 * @brief Converts a scalar-valued quadratic Bezier curve into canonical polynomial form.
 */
template <class Real>
LinearCurve1<Real> toMonomial (const LinearCurve1<Real> &b)
{
	return b * b2m_1<Real>;
}

/**
 * @brief
 *		Converts a 4D vector-valued quadratic Bezier curve into canonical polynomial
 *		form, writing the coefficient points to the output parameter.
 */
template <class Real>
void toMonomial (LinearCurve4<Real> &m, const LinearCurve4<Real> &b)
{
	m = b * b2m_1<Real>;
}

/**
 * @brief
 *		Converts a 3D vector-valued quadratic Bezier curve into canonical polynomial
 *		form, writing the coefficient points to the output parameter.
 */
template <class Real>
void toMonomial (LinearCurve3<Real> &m, const LinearCurve3<Real> &b)
{
	m = b * b2m_1<Real>;
}

/**
 * @brief
 *		Converts a 2D vector-valued quadratic Bezier curve into canonical polynomial
 *		form, writing the coefficient points to the output parameter.
 */
template <class Real>
void toMonomial (LinearCurve2<Real> &m, const LinearCurve2<Real> &b)
{
	m = b * b2m_1<Real>;
}

/**
 * @brief
 *		Converts a scalar-valued quadratic Bezier curve into canonical polynomial form,
 *		writing the polynomial coefficients to the output parameter.
 */
template <class Real>
void toMonomial (LinearCurve1<Real> &m, const LinearCurve1<Real> &b)
{
	m = b * b2m_1<Real>;
}


////
// Bezier curve derivatives

/**
 * @brief
 *		Derives the given 4D vector-valued cubic Bezier curve, returning the control
 *		points of the resulting quadratic Bezier curve
 */
template <class Real>
QuadraticCurve4<Real> deriveBezier (const CubicCurve4<Real> &b)
{
	return { Real(3)*(b[1]-b[0]), Real(3)*(b[2]-b[1]), Real(3)*(b[3]-b[2]) };
}

/**
 * @brief
 *		Derives the given 3D vector-valued cubic Bezier curve, returning the control
 *		points of the resulting quadratic Bezier curve
 */
template <class Real>
QuadraticCurve3<Real> deriveBezier (const CubicCurve3<Real> &b)
{
	return { Real(3)*(b[1]-b[0]), Real(3)*(b[2]-b[1]), Real(3)*(b[3]-b[2]) };
}

/**
 * @brief
 *		Derives the given 2D vector-valued cubic Bezier curve, returning the control
 *		points of the resulting quadratic Bezier curve
 */
template <class Real>
QuadraticCurve2<Real> deriveBezier (const CubicCurve2<Real> &b)
{
	return { Real(3)*(b[1]-b[0]), Real(3)*(b[2]-b[1]), Real(3)*(b[3]-b[2]) };
}

/**
 * @brief
 *		Derives the given scalar-valued cubic Bezier curve, returning the control points
 *		of the resulting quadratic Bezier curve
 */
template <class Real>
QuadraticCurve1<Real> deriveBezier (const CubicCurve1<Real> &b)
{
	return { Real(3)*(b[1]-b[0]), Real(3)*(b[2]-b[1]), Real(3)*(b[3]-b[2]) };
}

/**
 * @brief
 *		Derives the given 4D vector-valued quadratic Bezier curve, returning the control
 *		points of the resulting quadratic Bezier curve
 */
template <class Real>
LinearCurve4<Real> deriveBezier (const QuadraticCurve4<Real> &b)
{
	return LinearCurve4<Real>{ Real(2)*(b.col(1)-b.col(0)), Real(2)*(b.col(2)-b.col(1)) };
}

/**
 * @brief
 *		Derives the given 3D vector-valued quadratic Bezier curve, returning the control
 *		points of the resulting quadratic Bezier curve
 */
template <class Real>
LinearCurve3<Real> deriveBezier (const QuadraticCurve3<Real> &b)
{
	return LinearCurve3<Real>{ Real(2)*(b.col(1)-b.col(0)), Real(2)*(b.col(2)-b.col(1)) };
}

/**
 * @brief
 *		Derives the given 2D vector-valued quadratic Bezier curve, returning the control
 *		points of the resulting quadratic Bezier curve
 */
template <class Real>
LinearCurve2<Real> deriveBezier (const QuadraticCurve2<Real> &b)
{
	return LinearCurve2<Real>{ Real(2)*(b.col(1)-b.col(0)), Real(2)*(b.col(2)-b.col(1)) };
}

/**
 * @brief
 *		Derives the given scalar-valued quadratic Bezier curve, returning the control
 *		points of the resulting quadratic Bezier curve
 */
template <class Real>
LinearCurve1<Real> deriveBezier (const QuadraticCurve1<Real> &b)
{
	return { Real(2)*(b[1]-b[0]), Real(2)*(b[2]-b[1]) };
}


////
// Bezier curve evaluation

/** @brief Evaluates the given 4D vector-valued cubic Bezier curve at a given t. */
template <class Real>
Vec<4,Real> evalBezier (const CubicCurve4<Real> &b, Real t)
{
	Mat<3, 4, Real> v = {
		glm::mix(b[0], b[1], t), glm::mix(b[1], b[2], t), glm::mix(b[2], b[3], t)
	};
	v[0] = glm::mix(v[0], v[1], t);
	v[1] = glm::mix(v[1], v[2], t);
	return glm::mix(v[0], v[1], t);
}

/** @brief Evaluates the given 3D vector-valued cubic Bezier curve at a given t. */
template <class Real>
Vec<3, Real> evalBezier (const CubicCurve3<Real> &b, Real t)
{
	Mat<3, 3, Real> v = {
		glm::mix(b[0], b[1], t), glm::mix(b[1], b[2], t), glm::mix(b[2], b[3], t)};
	v[0] = glm::mix(v[0], v[1], t);
	v[1] = glm::mix(v[1], v[2], t);
	return glm::mix(v[0], v[1], t);
}

/** @brief Evaluates the given 2D vector-valued cubic Bezier curve at a given t. */
template <class Real>
Vec<2,Real> evalBezier (const CubicCurve2<Real> &b, Real t)
{
	Mat<3, 2, Real> v = {
		glm::mix(b[0], b[1], t), glm::mix(b[1], b[2], t), glm::mix(b[2], b[3], t)
	};
	v[0] = glm::mix(v[0], v[1], t);
	v[1] = glm::mix(v[1], v[2], t);
	return glm::mix(v[0], v[1], t);
}

/** @brief Evaluates the given scalar-valued cubic Bezier curve at a given t. */
template <class Real>
Real evalBezier (const CubicCurve1<Real> &b, Real t)
{
	Vec<3, Real> v = {
		glm::mix(b[0], b[1], t), glm::mix(b[1], b[2], t), glm::mix(b[2], b[3], t)
	};
	v[0] = glm::mix(v[0], v[1], t);
	v[1] = glm::mix(v[1], v[2], t);
	return glm::mix(v[0], v[1], t);
}

/** @brief Evaluates the given 4D vector-valued quadratic Bezier curve at a given t. */
template <class Real>
Vec<4,Real> evalBezier (const QuadraticCurve4<Real> &b, Real t)
{
	const Mat<2, 4, Real> v = { glm::mix(b.col(0), b.col(1), t), glm::mix(b.col(1), b.col(2), t) };
	return glm::mix(v.col(0), v.col(1), t);
}

/** @brief Evaluates the given 3D vector-valued quadratic Bezier curve at a given t. */
template <class Real>
Vec<3,Real> evalBezier (const QuadraticCurve3<Real> &b, Real t)
{
	const Mat<2, 3, Real> v = { glm::mix(b.col(0), b.col(1), t), glm::mix(b.col(1), b.col(2), t) };
	return glm::mix(v.col(0), v.col(1), t);
}

/** @brief Evaluates the given 2D vector-valued quadratic Bezier curve at a given t. */
template <class Real>
Vec<2,Real> evalBezier (const QuadraticCurve2<Real> &b, Real t)
{
	const Mat<2, 2, Real> v = { glm::mix(b.col(0), b.col(1), t), glm::mix(b.col(1), b.col(2), t) };
	return glm::mix(v.col(0), v.col(1), t);
}

/** @brief Evaluates the given scalar-valued quadratic Bezier curve at a given t=0..1. */
template <class Real>
Real evalBezier (const QuadraticCurve1<Real> &b, Real t)
{
	const Vec<2, Real> v = { glm::mix(b[0], b[1], t), glm::mix(b[1], b[2], t) };
	return glm::mix(v[0], v[1], t);
}


////
// Monomial curve evaluation

/**
 * @brief
 *		Efficiently evaluates the given 4D vector-valued cubic monomial curve at a given
 *		t using Horner's method.
 */
template <class Real>
Vec<4,Real> evalMonomial (const CubicCurve4<Real> &m, Real t)
{
	return m[3] + t*(m[2] + t*(m[1] + t*m[0]));
}

/**
 * @brief
 *		Efficiently evaluates the given 3D vector-valued cubic monomial curve at a given
 *		t using Horner's method.
 */
template <class Real>
Vec<3, Real> evalMonomial (const CubicCurve3<Real> &m, Real t)
{
	return m[3] + t*(m[2] + t*(m[1] + t*m[0]));
}

/**
 * @brief
 *		Efficiently evaluates the given 2D vector-valued cubic monomial curve at a given
 *		t using Horner's method.
 */
template <class Real>
Vec<2,Real> evalMonomial (const CubicCurve2<Real> &m, Real t)
{
	return m[3] + t*(m[2] + t*(m[1] + t*m[0]));
}

/**
 * @brief
 *		Efficiently evaluates the given scalar-valued cubic monomial curve at a given
 *		t using Horner's method.
 */
template <class Real>
Real evalMonomial (const CubicCurve1<Real> &m, Real t)
{
	return m[3] + t*(m[2] + t*(m[1] + t*m[0]));
}

/**
 * @brief
 *		Efficiently evaluates the given 4D vector-valued quadratic monomial curve at a
 *		given t using Horner's method.
 */
template <class Real>
Vec<4,Real> evalMonomial (const QuadraticCurve4<Real> &m, Real t)
{
	return m[2] + t*(m[1] + t*m[0]);
}

/**
 * @brief
 *		Efficiently evaluates the given 3D vector-valued quadratic monomial curve at a
 *		given t using Horner's method.
 */
template <class Real>
Vec<3,Real> evalMonomial (const QuadraticCurve3<Real> &m, Real t)
{
	return m[2] + t*(m[1] + t*m[0]);
}

/**
 * @brief
 *		Efficiently evaluates the given 2D vector-valued quadratic monomial curve at a
 *		given t using Horner's method.
 */
template <class Real>
Vec<2,Real> evalMonomial (const QuadraticCurve2<Real> &m, Real t)
{
	return m[2] + t*(m[1] + t*m[0]);
}

/**
 * @brief
 *		Efficiently evaluates the given scalar-valued quadratic monomial curve at a given
 *		t using Horner's method.
 */
template <class Real>
Real evalMonomial (const QuadraticCurve1<Real> &m, Real t)
{
	return m[2] + t*(m[1] + t*m[0]);
}

/** @brief Evaluates the given 4D vector-valued quadratic monomial curve at a given t. */
template <class Real>
Vec<4,Real> evalMonomial (const LinearCurve4<Real> &m, Real t)
{
	return t*m[0] + m[1];
}

/** @brief Evaluates the given 3D vector-valued quadratic monomial curve at a given t. */
template <class Real>
Vec<3,Real> evalMonomial (const LinearCurve3<Real> &m, Real t)
{
	return t*m[0] + m[1];
}

/** @brief Evaluates the given 2D vector-valued quadratic monomial curve at a given t. */
template <class Real>
Vec<2,Real> evalMonomial (const LinearCurve2<Real> &m, Real t)
{
	return t*m[0] + m[1];
}

/** @brief Evaluates the given scalar-valued quadratic monomial curve at a given t. */
template <class Real>
Real evalMonomial (const LinearCurve1<Real> &m, Real t)
{
	return t*m[0] + m[1];
}



////
// Linear root solvers

/** @brief Finds the roots of the given scalar-valued linear monomial curve. */
template <class Real>
LinearRoots1<Real> solveLinear (const LinearCurve1<Real> &m)
{
	LinearRoots1<Real> result;
	if (m[0] == 0)
		result.num[0] = 0;
	else {
		result.num[0] = 1;
		result.roots[0] = -m[1]/m[0];
	}
	return result;
}

/** @brief Finds the roots of the given 4D-vector valued linear monomial curve. */
template <class Real, unsigned dims>
LinearRoots<Real, dims> solveLinear (const LinearCurve<Real, dims> &m)
{
	// transpose curve control matrix so we can use each column as a component function
	LinearRoots<Real, dims> result;
	for (unsigned i=0; i<dims; i++)
	{
		const auto component_root = solveLinear(m.row(i));
		result.roots[i] = component_root.roots[0];
		result.num[i] = component_root.num[0];
	}
	return result;
}


////
// Misc utilities

// Checks if a given value is inside the given interval
inline bool isBetween (const float t, const float tmin, const float tmax)
{
	return t >= tmin && t <= tmax;
}

// Checks if a given value is between 0 and 1
inline bool isBetween01 (const float t)
{
	return isBetween(t, 0, 1);
}

// Checks if a given value is outside the given interval
inline bool isOutside (const float t, const float tmin, const float tmax)
{
	return t < tmin || t > tmax;
}

// Checks if a given value is outside of 0 and 1
inline bool isOutside01 (const float t)
{
	return isOutside(t, 0, 1);
}


#endif // ifndef __CURVEUTILS_H__
