/**
 * @file
 * @brief Sub header defining general core types used throughout the API.
 */

#ifndef __TYPES_H__
#define __TYPES_H__


//////
//
// Typedefs and structs
//

/**
 * @brief A convenience struct for representing 2D vectors.
 *
 * @see otv__Vec2()
 */
typedef struct OTV_Vec2
{
	/// @brief The @a x component.
	float x;

	/// @brief The @a y component.
	float y;
} OTV_Vec2;

/**
 * @brief A convenience struct for representing 3D vectors.
 *
 * @see otv__Vec3()
 */
typedef struct OTV_Vec3
{
	/// @brief The @a x component.
	float x;

	/// @brief The @a y component.
	float y;

	/// @brief The @a z component.
	float z;
} OTV_Vec3;

/**
 * @brief A convenience struct for representing 4D vectors.
 *
 * @see otv__Vec4()
 */
typedef struct OTV_Vec4
{
	/// @brief The @a x component.
	float x;

	/// @brief The @a y component.
	float y;

	/// @brief The @a z component.
	float z;

	/// @brief The @a w component.
	float w;
} OTV_Vec4;

/**
 * @brief A convenience struct for representing RGB values.
 *
 * @see otv__Rgb()
 */
typedef struct OTV_Rgb
{
	/// @brief The @a red component, in the range <c>0..1</c>.
	float r;

	/// @brief The @a green component, in the range <c>0..1</c>.
	float g;

	/// @brief The @a blue component, in the range <c>0..1</c>.
	float b;
} OTV_Rgb;

/**
 * @brief A convenience struct for representing an inclusive interval.
 *
 * @see otv__Interval()
 */
typedef struct OTV_Interval
{
	/// @brief The @a minimum number in the interval, i.e. its lower bound.
	float min;

	/// @brief The @a maximum number in the interval, i.e. its upper bound.
	float max;
} OTV_Interval;



//////
//
// Functions
//

// --------------------------------------------------------------------------------------------------------------------
// otv__Vec2

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Constructs an instance of the @c OTV_Vec2 struct for convenient on-the-fly construction as a pass-by-value
 *		argument to other functions.
 *
 * @param x The value for the field @c OTV_Vec2::x
 * @param y The value for the field @c OTV_Vec2::y
 *
 * @return An instance of the @c OTV_Vec2 struct.
 */
OTV_API OTV_Vec2 otv__Vec2 (const float x, const float y);
#endif

/// @brief The function pointer type for the @c otv__Vec2() function.
typedef OTV_Vec2(*otv__Vec2_funct)(const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__Vec2()
extern otv__Vec2_funct otv__Vec2;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__Vec3

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Constructs an instance of the @c OTV_Vec3 struct for convenient on-the-fly construction as a pass-by-value
 *		argument to other functions.
 *
 * @param x The value for the field @c OTV_Vec3::x
 * @param y The value for the field @c OTV_Vec3::y
 * @param z The value for the field @c OTV_Vec3::z
 *
 * @return An instance of the @c OTV_Vec3 struct.
 */
OTV_API OTV_Vec3 otv__Vec3 (const float x, const float y, const float z);
#endif

/// @brief The function pointer type for the @c otv__Vec3() function.
typedef OTV_Vec3(*otv__Vec3_funct)(const float, const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__Vec3()
extern otv__Vec3_funct otv__Vec3;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__Vec4

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Constructs an instance of the @c OTV_Vec4 struct for convenient on-the-fly construction as a pass-by-value
 *		argument to other functions.
 *
 * @param x The value for the field @c OTV_Vec4::x
 * @param y The value for the field @c OTV_Vec4::y
 * @param z The value for the field @c OTV_Vec4::z
 * @param w The value for the field @c OTV_Vec4::w
 *
 * @return An instance of the @c OTV_Vec4 struct.
 */
OTV_API OTV_Vec4 otv__Vec4 (const float x, const float y, const float z, const float w);
#endif

/// @brief The function pointer type for the @c otv__Vec4() function.
typedef OTV_Vec4(*otv__Vec4_funct)(const float, const float, const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__Vec4()
extern otv__Vec4_funct otv__Vec4;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__Rgb

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Constructs an instance of the @c OTV_Rgb struct for convenient on-the-fly construction as a pass-by-value
 *		argument to other functions.
 *
 * @param r The value for the field @c OTV_Rgb::r
 * @param g The value for the field @c OTV_Rgb::g
 * @param b The value for the field @c OTV_Rgb::b
 *
 * @return An instance of the @c OTV_Rgb struct.
 */
OTV_API OTV_Rgb otv__Rgb (const float r, const float g, const float b);
#endif

/// @brief The function pointer type for the @c otv__Rgb() function.
typedef OTV_Rgb(*otv__Rgb_funct)(const float, const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__Rgb()
extern otv__Rgb_funct otv__Rgb;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__Interval

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Constructs an instance of the @c OTV_Interval struct for convenient on-the-fly construction as a pass-by-value
 *		argument to other functions.
 *
 * @param min The value for the field @c OTV_Interval::min
 * @param max The value for the field @c OTV_Interval::max
 *
 * @return An instance of the @c OTV_Interval struct.
 */
OTV_API OTV_Interval otv__Interval (const float min, const float max);
#endif

/// @brief The function pointer type for the @c otv__Interval() function.
typedef OTV_Rgb(*otv__Interval_funct)(const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__Interval()
extern otv__Interval_funct otv__Interval;
#endif


#endif // ifdef __TYPES_H__
