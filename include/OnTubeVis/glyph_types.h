/**
 * @file
 * @brief Sub header defining types and functions for dealing with glyphs.
 */

#ifndef __GLYPH_TYPES_H__
#define __GLYPH_TYPES_H__


//////
//
// Includes
//

// C standard library
#include <stdint.h>

// Public API
#include <OnTubeVis/config.h>
#include <OnTubeVis/types.h>



//////
//
// Language config [OPEN]
//

// Make sure we don't get the bugged-out warning on MSCV (we don't have custom constructors, the warning is just wrong)
#ifdef _MSC_VER
	#pragma warning(push)
	#pragma warning(disable : 4190)
#endif



//////
//
// Typedefs and structs
//

// --------------------------------------------------------------------------------------------------------------------
// Enums

/// @brief Enumeration of all color maps known by this version of the API.
typedef enum OTV_ColorMap {
	// Sequential color maps, since OnTubeVis API v0
	Acton = 0,
	Bamako,
	Batlow,
	BatlowK,
	BatlowW,
	Bilbao,
	Buda,
	Davos,
	Devon,
	Glasgow,
	GrayC,
	Hawaii,
	Imola,
	Lajolla,
	Lapaz,
	Lipari,
	Navia,
	Nuuk,
	Oslo,
	Rainbow,
	Tokyo,
	Turbo,
	Turku,

	// Diverging color maps, since OnTubeVis API v0
	Bam = 8192,
	Berlin,
	Broc,
	Cork,
	Lisbon,
	Managua,
	Roma,
	Tofino,
	Vanimo,
	Vik,

	/**
	 * @brief
	 *		A dummy enum that can be used when a corresponding function argument won't be used but has to be set anyway.
	 *
	 * Also causes the enum to be 32 bits as an intended side effect.
	 */
	UndefinedColormap = 0x7fffffff
} OTV_ColorMap;

/// @brief Enumeration of all glyph/plot types known by this version of the API.
typedef enum OTV_GlyphType {
	// Discrete glyphs, since OnTubeVis API v0
	Circle = 0,
	Rect,
	IsoscelesTriangle,
	SignBlob,

	// Plot-likes, since OnTubeVis API v0
	SurfaceColor = 16384,
	LinePlot,

	// Force enum to be 32 bits
	GlyphType_FORCE32 = 0x7fffffff
} OTV_GlyphType;

/// @brief Enumeration of all shading interpolation modes known by this version of the API.
typedef enum OTV_InterpolationMode {
	// Since OnTubeVis API v0
	Nearest = 0,
	Linear,
	Cubic,

	// Force enum to be 32 bits
	InterpolationMode_FORCE32 = 0x7fffffff
} OTV_InterpolationMode;

/// @brief Enumeration of static parameter flags for @c OTV_SurfaceColorInfo structs.
typedef enum OTV_SurfaceColorInfoStaticFlags {
	// Since OnTubeVis API v0
	SCI_STATIC_NONE = 0,
	SCI_STATIC_COLOR = 1,

	// Force enum to be 32 bits
	SurfaceColorInfoStaticFlags_FORCE32 = 0x7fffffff
} OTV_SurfaceColorInfoStaticFlags;

/// @brief Enumeration of static parameter flags for @c OTV_RectangleInfo structs.
typedef enum OTV_RectangleInfoStaticFlags {
	// Since OnTubeVis API v0
	RI_STATIC_NONE = 0,
	RI_STATIC_COLOR = 1,
	RI_STATIC_WIDTH = 2,
	RI_STATIC_HEIGHT = 4,

	// Force enum to be 32 bits
	RectangleInfoStaticFlags_FORCE32 = 0x7fffffff
} OTV_RectangleInfoStaticFlags;

/// @brief Enumeration of static parameter flags for @c OTV_SignBlobInfo structs.
typedef enum OTV_SignBlobInfoStaticFlags {
	// Since OnTubeVis API v0
	SBI_STATIC_NONE = 0,
	SBI_STATIC_COLOR = 1,
	SBI_STATIC_VALUE = 2,

	// Force enum to be 32 bits
	SignBlobInfoStaticFlags_FORCE32 = 0x7fffffff
} OTV_SignBlobInfoStaticFlags;


// --------------------------------------------------------------------------------------------------------------------
// Generic structs

/**
 * @brief
 *		A struct representing a generic glyph that can be up-casted into a struct for any of the known concrete glyph
 *		types.
 */
typedef struct OTV_GlyphInfo
{
	/// @brief The number of floats the glyph information consists of.
	const uint32_t N;

	/**
	 * @brief
	 *		Storage for up to 31 floats describing the static glyph properties. Semantics depend on the concrete glyph
	 *		type.
	 */
	 float params[31];
} OTV_GlyphInfo;

/**
 * @brief
 *		A struct generically representing a single glyph instance that can be up-casted into a struct for any of the
 *		known concrete glyph types.
 */
typedef struct OTV_GlyphData
{
	/// @brief The number of data floats (i.e. excluding the arc length @c s) the glyph data consists of.
	const uint32_t N;

	/// @brief The arc length along the trajectory that the glyph is located.
	float s;

	/**
	 * @brief
	 *		Storage for up to 14 floats describing the static glyph properties. Semantics depend on the concrete glyph
	 *		type.
	 */
	 float params[14];
} OTV_GlyphData;


// --------------------------------------------------------------------------------------------------------------------
// Concrete structs

////
// SurfaceColor

/**
 * @brief The info struct for the @c GlyphType::SurfaceColor glyph.
 *
 * @see otv__construct_SurfaceColorInfo()
 * @see otv__construct_empty_SurfaceColorInfo()
 */
typedef struct OTV_SurfaceColorInfo
{
	/// @brief The number of 32bit static properties of a surface color layer. Must be 2 always.
	const uint32_t N;

	/// @brief The surface color in case a static color is assigned.
	OTV_Rgb rgb;

	/// @brief The selected color map to query colors from if no static color is used.
	OTV_ColorMap color_map;

	/// @brief The selected interpolation strategy for shading between samples
	OTV_InterpolationMode interpolation_mode;

	/// @brief Which of the dynamic properties should statically assume the values defined in this info struct.
	OTV_SurfaceColorInfoStaticFlags static_flags;
} OTV_SurfaceColorInfo;

/**
 * @brief The data struct for the @c GlyphType::SurfaceColor glyph.
 *
 * @see otv__construct_SurfaceColorData()
 * @see otv__construct_empty_SurfaceColorData()
 */
typedef struct OTV_SurfaceColorData
{
	/**
	 * @brief
	 *		The number of 32bit dynamic properties (excluding arc length @c s) of a surface color layer. Must be 1
	 *		always.
	 */
	const uint32_t N;

	/// @brief The arc length along the trajectory that the surface color control point is located.
	float s;

	/// @brief A value in the range <c>0..1</c> to query the selected color map with.
	float color;
} OTV_SurfaceColorData;


////
// LinePlot

/**
 * @brief The info struct for the @c GlyphType::LinePlot glyph.
 *
 * @see otv__construct_LinePlotInfo()
 * @see otv__construct_empty_LinePlotInfo()
 */
typedef struct OTV_LinePlotInfo
{
	/// @brief The number of 32bit static properties of a line plot layer. Must be 14 always.
	const uint32_t N;

	/// @brief The selected interpolation strategy for shading between samples.
	OTV_InterpolationMode interpolation_mode;

	/// @brief The number of sub plots - only values of 1, 2, 3 or 4 are allowed.
	uint32_t num_subplots;

	/// @brief The colors to use for each sub plot. Unused slots may be left uninitialized.
	OTV_Rgb subplot_colors[4];
} OTV_LinePlotInfo;

/**
 * @brief The data struct for the @c GlyphType::LinePlot glyph.
 *
 * @see otv__construct_LinePlotData()
 * @see otv__construct_empty_LinePlotData()
 */
typedef struct OTV_LinePlotData
{
	/**
	 * @brief The number of 32bit dynamic properties (excluding arc length @c s) of a line plot layer. Must be 4 always.
	 */
	const uint32_t N;

	/// @brief The arc length along the trajectory that the surface color control point is located.
	float s;

	/**
	 * @brief
	 *		The values in the range <c>0..1</c> for each sub plot. Only the first @a n values need to be set when the
	 *		corresponding layer defines @a n @link OTV_LinePlotInfo::num_subplots sub plots @endlink.
	 */
	float values[4];
} OTV_LinePlotData;


////
// Rectangle

/**
 * @brief The info struct for the @c GlyphType::Rect glyph.
 *
 * @see otv__construct_RectangleInfo()
 * @see otv__construct_empty_RectangleInfo()
 */
typedef struct OTV_RectangleInfo
{
	/// @brief The number of 32bit static properties of a rectangle glyph layer. Must be 7 always.
	const uint32_t N;

	/// @brief The glyph color in case a static color is assigned.
	OTV_Rgb rgb;

	/// @brief The selected color map to query colors from if no static color is used.
	OTV_ColorMap color_map;

	/**
	 * @brief The @a width of the rectangle in multiples of the tube/ribbon radius in case a static value is to be used.
	 */
	float width;

	/**
	 * @brief
	 *		The @a height of the rectangle in multiples of the tube/ribbon radius in case a static value is to be used.
	 */
	float height;

	/// @brief Which of the dynamic glyph properties should statically assume the values defined in this info struct.
	OTV_RectangleInfoStaticFlags static_flags;
} OTV_RectangleInfo;

/**
 * @brief The data struct for the @c GlyphType::Rect glyph.
 *
 * @see otv__construct_RectangleData()
 * @see otv__construct_empty_RectangleData()
 * @see otv__instantiate_Rectangle()
 */
typedef struct OTV_RectangleData
{
	/**
	 * @brief
	 *		The number of 32bit dynamic properties (excluding arc length @c s) of a rectangle glyph layer. Must be 3
	 *		always.
	 */
	const uint32_t N;

	/// @brief The arc length along the trajectory that the surface color control point is located.
	float s;

	/**
	 * @brief
	 *		A value in the range <c>0..1</c> to query the selected color map with in case a dynamic color is to be used.
	 */
	float color;

	/**
	 * @brief
	 *		The @a width of the rectangle in multiples of the tube/ribbon radius in case a dynamic value is to be used.
	 */
	float width;

	/**
	 * @brief
	 *		The @a height of the rectangle in multiples of the tube/ribbon radius in case a dynamic value is to be used.
	 */
	float height;
} OTV_RectangleData;


////
// SignBlob

/**
 * @brief The info struct for the @c GlyphType::SignBlob glyph.
 *
 * @see otv__construct_SignBlobInfo()
 * @see otv__construct_empty_SignBlobInfo()
 */
typedef struct OTV_SignBlobInfo
{
	/// @brief The number of 32bit static properties of a sign blob layer. Must be 7 always.
	const uint32_t N;

	/// @brief The glyph color in case a static color is assigned.
	OTV_Rgb rgb;

	/// @brief The selected color map to query colors from if no static color is used.
	OTV_ColorMap color_map;

	// The radius of the glyph instances (always stays fixed for sign blobs)
	float radius;

	/// @brief The @a value of the sign blob in the range <c>-1..1</c> in case a static value is to be used.
	float value;

	/// @brief Which of the dynamic glyph properties should statically assume the values defined in this info struct.
	OTV_SignBlobInfoStaticFlags static_flags;
} OTV_SignBlobInfo;

/**
 * @brief The data struct for the @c GlyphType::SignBlob glyph.
 *
 * @see otv__construct_SignBlobData()
 * @see otv__construct_empty_SignBlobData()
 * @see otv__instantiate_SignBlob()
 */
typedef struct OTV_SignBlobData
{
	/**
	 * @brief
	 *		The number of 32bit dynamic properties (excluding arc length @c s) of a sign blob glyph layer. Must be 2
	 *		always.
	 */
	const uint32_t N;

	/// @brief The arc length along the trajectory that the surface color control point is located.
	float s;

	/**
	 * @brief
	 *		A value in the range <c>0..1</c> to query the selected color map with in case a dynamic color is to be used.
	 */
	float color;

	/// @brief The @a value of the sign blob in the range <c>-1..1</c> in case a dynamic value is to be used.
	float value;
} OTV_SignBlobData;



//////
//
// Functions
//

////
// ####################################################################################################################
// OTV_SurfaceColorInfo

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_SurfaceColorInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_SurfaceColorInfo struct that will be correctly up- and downcastable.
 *
 * @param rgb The value for the field @c OTV_SurfaceColorInfo::rgb
 * @param color_map The value for the field @c OTV_SurfaceColorInfo::color_map
 * @param interpolation_mode The value for the field @c OTV_SurfaceColorInfo::interpolation_mode
 * @param static_flags The value for the field @c OTV_SurfaceColorInfo::static_flags
 *
 * @return An instance of the @c OTV_SurfaceColorInfo struct, downcasted to the generic @c OTV_GlyphInfo.
 *
 * @note It is necessary to return a generic @c OTV_GlyphInfo view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_SurfaceColorInfo form, you can upcast it
 * using @c otv__upcast_SurfaceColorInfo().
 */
OTV_API OTV_GlyphInfo otv__construct_SurfaceColorInfo (
	OTV_Rgb rgb, const OTV_ColorMap color_map, const OTV_InterpolationMode interpolation_mode,
	const OTV_SurfaceColorInfoStaticFlags static_flags
);
#endif

/// @brief The function pointer type for the @c otv__construct_SurfaceColorInfo() function.
typedef OTV_GlyphInfo(*otv__construct_SurfaceColorInfo_funct)(const OTV_ColorMap, const OTV_InterpolationMode);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_SurfaceColorInfo()
extern otv__construct_SurfaceColorInfo_funct otv__construct_SurfaceColorInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_SurfaceColorInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_SurfaceColorInfo()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_SurfaceColorInfo()
 */
OTV_API OTV_GlyphInfo otv__construct_empty_SurfaceColorInfo (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_SurfaceColorInfo() function.
typedef OTV_GlyphInfo(*otv__construct_empty_SurfaceColorInfo_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_SurfaceColorInfo()
extern otv__construct_empty_SurfaceColorInfo_funct otv__construct_empty_SurfaceColorInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_SurfaceColorInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphInfo struct to a @c OTV_SurfaceColorInfo.
 *
 * @param surface_color_info The @c OTV_GlyphInfo to upcast.
 *
 * @return The upcasted @c OTV_SurfaceColorInfo view on the given @c OTV_GlyphInfo.
 */
OTV_API OTV_SurfaceColorInfo* otv__upcast_SurfaceColorInfo (const OTV_GlyphInfo *surface_color_info);
#endif

/// @brief The function pointer type for the @c otv__upcast_SurfaceColorInfo() function.
typedef OTV_SurfaceColorInfo*(*otv__upcast_SurfaceColorInfo_funct)(const OTV_GlyphInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_SurfaceColorInfo()
extern otv__upcast_SurfaceColorInfo_funct otv__upcast_SurfaceColorInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_SurfaceColorInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_SurfaceColorInfo struct to a @c OTV_GlyphInfo.
 *
 * @param surface_color_info The @c OTV_SurfaceColorInfo to downcast.
 *
 * @return The downcasted @c OTV_GlyphInfo view on the given @c OTV_SurfaceColorInfo.
 */
OTV_API OTV_GlyphInfo* otv__downcast_SurfaceColorInfo (const OTV_SurfaceColorInfo *surface_color_info);
#endif

/// @brief The function pointer type for the @c otv__downcast_SurfaceColorInfo() function.
typedef OTV_GlyphInfo*(*otv__downcast_SurfaceColorInfo_funct)(const OTV_SurfaceColorInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_SurfaceColorInfo()
extern otv__downcast_SurfaceColorInfo_funct otv__downcast_SurfaceColorInfo;
#endif


////
// ####################################################################################################################
// OTV_SurfaceColorData

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_SurfaceColorData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_SurfaceColorData struct that will be correctly up- and downcastable.
 *
 * @param s The value for the field @c OTV_SurfaceColorData::s
 * @param color The value for the field @c OTV_SurfaceColorData::color
 *
 * @return An instance of the @c OTV_SurfaceColorData struct, downcasted to the generic @c OTV_GlyphData.
 *
 * @note It is necessary to return a generic @c OTV_GlyphData view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_SurfaceColorData form, you can upcast it
 * using @c otv__upcast_SurfaceColorData().
 */
OTV_API OTV_GlyphData otv__construct_SurfaceColorData (const float s, const float color);
#endif

/// @brief The function pointer type for the @c otv__construct_SurfaceColorData() function.
typedef OTV_GlyphData(*otv__construct_SurfaceColorData_funct)(const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_SurfaceColorData()
extern otv__construct_SurfaceColorData_funct otv__construct_SurfaceColorData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_SurfaceColorData

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_SurfaceColorData()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_SurfaceColorData()
 */
OTV_API OTV_GlyphData otv__construct_empty_SurfaceColorData (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_SurfaceColorData() function.
typedef OTV_GlyphData(*otv__construct_empty_SurfaceColorData_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_SurfaceColorData()
extern otv__construct_empty_SurfaceColorData_funct otv__construct_empty_SurfaceColorData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_SurfaceColorData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphData struct to a @c OTV_SurfaceColorData.
 *
 * @param surface_color_data The @c OTV_GlyphData to upcast.
 *
 * @return The upcasted @c OTV_SurfaceColorData view on the given @c OTV_GlyphData.
 */
OTV_API OTV_SurfaceColorData* otv__upcast_SurfaceColorData (const OTV_GlyphData *surface_color_data);
#endif

/// @brief The function pointer type for the @c otv__upcast_SurfaceColorData() function.
typedef OTV_SurfaceColorData*(*otv__upcast_SurfaceColorData_funct)(const OTV_GlyphData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_SurfaceColorData()
extern otv__upcast_SurfaceColorData_funct otv__upcast_SurfaceColorData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_SurfaceColorData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_SurfaceColorData struct to a @c OTV_GlyphData.
 *
 * @param surface_color_data The @c OTV_SurfaceColorData to downcast.
 *
 * @return The downcasted @c OTV_GlyphData view on the given @c OTV_SurfaceColorData.
 */
OTV_API OTV_GlyphData* otv__downcast_SurfaceColorData (const OTV_SurfaceColorData *surface_color_data);
#endif

/// @brief The function pointer type for the @c otv__downcast_SurfaceColorData() function.
typedef OTV_GlyphData*(*otv__downcast_SurfaceColorData_funct)(const OTV_SurfaceColorData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_SurfaceColorData()
extern otv__downcast_SurfaceColorData_funct otv__downcast_SurfaceColorData;
#endif


////
// ####################################################################################################################
// OTV_LinePlotInfo

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_LinePlotInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_LinePlotInfo struct that will be correctly up- and downcastable.
 *
 * @param interpolation_mode The value for the field @c OTV_LinePlotInfo::interpolation_mode
 * @param num_subplots The value for the field @c OTV_LinePlotInfo::num_subplots
 * @param subplot_colors
 *		The values for the field @c OTV_LinePlotInfo::subplot_colors. Only the first @a num_subplots entries need to be
 *		set.
 *
 * @return An instance of the @c OTV_LinePlotInfo struct, downcasted to the generic @c OTV_GlyphInfo.
 *
 * @note It is necessary to return a generic @c OTV_GlyphInfo view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_LinePlotInfo form, you can upcast it
 * using @c otv__upcast_LinePlotInfo().
 */
OTV_API OTV_GlyphInfo otv__construct_LinePlotInfo (
	const OTV_InterpolationMode interpolation_mode, const uint32_t num_subplots, const OTV_Rgb subplot_colors[]
);
#endif

/// @brief The function pointer type for the @c otv__construct_LinePlotInfo() function.
typedef OTV_GlyphInfo(*otv__construct_LinePlotInfo_funct)(const OTV_InterpolationMode, const uint32_t, const OTV_Rgb[]);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_LinePlotInfo()
extern otv__construct_LinePlotInfo_funct otv__construct_LinePlotInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_LinePlotInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_LinePlotInfo()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_LinePlotInfo()
 */
OTV_API OTV_GlyphInfo otv__construct_empty_LinePlotInfo (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_LinePlotInfo() function.
typedef OTV_GlyphInfo(*otv__construct_empty_LinePlotInfo_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_LinePlotInfo()
extern otv__construct_empty_LinePlotInfo_funct otv__construct_empty_LinePlotInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_LinePlotInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphInfo struct to a @c OTV_LinePlotInfo.
 *
 * @param line_plot_info The @c OTV_GlyphInfo to upcast.
 *
 * @return The upcasted @c OTV_LinePlotInfo view on the given @c OTV_GlyphInfo.
 */
OTV_API OTV_LinePlotInfo* otv__upcast_LinePlotInfo (const OTV_GlyphInfo *line_plot_info);
#endif

/// @brief The function pointer type for the @c otv__upcast_LinePlotInfo() function.
typedef OTV_LinePlotInfo*(*otv__upcast_LinePlotInfo_funct)(const OTV_GlyphInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_LinePlotInfo()
extern otv__upcast_LinePlotInfo_funct otv__upcast_LinePlotInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_LinePlotInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_LinePlotInfo struct to a @c OTV_GlyphInfo.
 *
 * @param line_plot_info The @c OTV_LinePlotInfo to downcast.
 *
 * @return The downcasted @c OTV_GlyphInfo view on the given @c OTV_LinePlotInfo.
 */
OTV_API OTV_GlyphInfo* otv__downcast_LinePlotInfo (const OTV_LinePlotInfo *line_plot_info);
#endif

/// @brief The function pointer type for the @c otv__downcast_LinePlotInfo() function.
typedef OTV_GlyphInfo*(*otv__downcast_LinePlotInfo_funct)(const OTV_LinePlotInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_LinePlotInfo()
extern otv__downcast_LinePlotInfo_funct otv__downcast_LinePlotInfo;
#endif


////
// ####################################################################################################################
// OTV_LinePlotData

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_LinePlotData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_LinePlotData struct that will be correctly up- and downcastable.
 *
 * @param s The value for the field @c OTV_LinePlotData::s
 * @param num_values The number of values the caller wants to pass in via the @a values argument.
 * @param values
 *		The values for the field @c OTV_LinePlotData::values. Only the first @a num_subplots entries need to be
 *		set.
 *
 * @return An instance of the @c OTV_LinePlotData struct, downcasted to the generic @c OTV_GlyphData.
 *
 * @note It is necessary to return a generic @c OTV_GlyphData view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_LinePlotData form, you can upcast it
 * using @c otv__upcast_LinePlotData().
 */
OTV_API OTV_GlyphData otv__construct_LinePlotData (const float s, const uint32_t num_values, const float values[]);
#endif

/// @brief The function pointer type for the @c otv__construct_LinePlotData() function.
typedef OTV_GlyphData(*otv__construct_LinePlotData_funct)(const float, const uint32_t, const float[]);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_LinePlotData()
extern otv__construct_LinePlotData_funct otv__construct_LinePlotData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_LinePlotData

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_LinePlotData()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_LinePlotData()
 */
OTV_API OTV_GlyphData otv__construct_empty_LinePlotData (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_LinePlotData() function.
typedef OTV_GlyphData(*otv__construct_empty_LinePlotData_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_LinePlotData()
extern otv__construct_empty_LinePlotData_funct otv__construct_empty_LinePlotData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_LinePlotData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphData struct to a @c OTV_LinePlotData.
 *
 * @param line_plot_data The @c OTV_GlyphData to upcast.
 *
 * @return The upcasted @c OTV_LinePlotData view on the given @c OTV_GlyphData.
 */
OTV_API OTV_LinePlotData* otv__upcast_LinePlotData (const OTV_GlyphData *line_plot_data);
#endif

/// @brief The function pointer type for the @c otv__upcast_LinePlotData() function.
typedef OTV_LinePlotData*(*otv__upcast_LinePlotData_funct)(const OTV_GlyphData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_LinePlotData()
extern otv__upcast_LinePlotData_funct otv__upcast_LinePlotData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_LinePlotData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_LinePlotData struct to a @c OTV_GlyphData.
 *
 * @param line_plot_data The @c OTV_LinePlotData to downcast.
 *
 * @return The downcasted @c OTV_GlyphData view on the given @c OTV_LinePlotData.
 */
OTV_API OTV_GlyphData* otv__downcast_LinePlotData (const OTV_LinePlotData *line_plot_data);
#endif

/// @brief The function pointer type for the @c otv__downcast_LinePlotData() function.
typedef OTV_GlyphData*(*otv__downcast_LinePlotData_funct)(const OTV_LinePlotData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_LinePlotData()
extern otv__downcast_LinePlotData_funct otv__downcast_LinePlotData;
#endif


////
// ####################################################################################################################
// OTV_RectangleInfo

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_RectangleInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_RectangleInfo struct that will be correctly up- and downcastable.
 *
 * @param rgb The value for the field @c OTV_RectangleInfo::rgb
 * @param color_map The value for the field @c OTV_RectangleInfo::color_map
 * @param width The value for the field @c OTV_RectangleInfo::width
 * @param height The value for the field @c OTV_RectangleInfo::height
 * @param static_flags The value for the field @c OTV_RectangleInfo::static_flags
 *
 * @return An instance of the @c OTV_RectangleInfo struct, downcasted to the generic @c OTV_GlyphInfo.
 *
 * @note It is necessary to return a generic @c OTV_GlyphInfo view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_RectangleInfo form, you can upcast it
 * using @c otv__upcast_RectangleInfo().
 */
OTV_API OTV_GlyphInfo otv__construct_RectangleInfo (
	const OTV_Rgb rgb, const OTV_ColorMap color_map, const float width, const float height,
	const OTV_RectangleInfoStaticFlags static_flags
);
#endif

/// @brief The function pointer type for the @c otv__construct_RectangleInfo() function.
typedef OTV_GlyphInfo(*otv__construct_RectangleInfo_funct)(
	const OTV_ColorMap color_map, const OTV_InterpolationMode interpolation_mode
);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_RectangleInfo()
extern otv__construct_RectangleInfo_funct otv__construct_RectangleInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_RectangleInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_RectangleInfo()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_RectangleInfo()
 */
OTV_API OTV_GlyphInfo otv__construct_empty_RectangleInfo (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_RectangleInfo() function.
typedef OTV_GlyphInfo(*otv__construct_empty_RectangleInfo_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_RectangleInfo()
extern otv__construct_empty_RectangleInfo_funct otv__construct_empty_RectangleInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_RectangleInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphInfo struct to a @c OTV_RectangleInfo.
 *
 * @param line_plot_info The @c OTV_GlyphInfo to upcast.
 *
 * @return The upcasted @c OTV_RectangleInfo view on the given @c OTV_GlyphInfo.
 */
OTV_API OTV_RectangleInfo* otv__upcast_RectangleInfo (const OTV_GlyphInfo *line_plot_info);
#endif

/// @brief The function pointer type for the @c otv__upcast_RectangleInfo() function.
typedef OTV_RectangleInfo*(*otv__upcast_RectangleInfo_funct)(const OTV_GlyphInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_RectangleInfo()
extern otv__upcast_RectangleInfo_funct otv__upcast_RectangleInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_RectangleInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_RectangleInfo struct to a @c OTV_GlyphInfo.
 *
 * @param line_plot_info The @c OTV_RectangleInfo to downcast.
 *
 * @return The downcasted @c OTV_GlyphInfo view on the given @c OTV_RectangleInfo.
 */
OTV_API OTV_GlyphInfo* otv__downcast_RectangleInfo (const OTV_RectangleInfo *line_plot_info);
#endif

/// @brief The function pointer type for the @c otv__downcast_RectangleInfo() function.
typedef OTV_GlyphInfo*(*otv__downcast_RectangleInfo_funct)(const OTV_RectangleInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_RectangleInfo()
extern otv__downcast_RectangleInfo_funct otv__downcast_RectangleInfo;
#endif


////
// ####################################################################################################################
// OTV_RectangleData

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_RectangleData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_RectangleData struct that will be correctly up- and downcastable.
 *
 * @param s The value for the field @c OTV_RectangleData::s
 * @param color The value for the field @c OTV_RectangleData::color
 * @param width The value for the field @c OTV_RectangleData::width
 * @param height The value for the field @c OTV_RectangleData::height
 *
 * @return An instance of the @c OTV_RectangleData struct, downcasted to the generic @c OTV_GlyphData.
 *
 * @note It is necessary to return a generic @c OTV_GlyphData view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_RectangleData form, you can upcast it
 * using @c otv__upcast_RectangleData().
 */
OTV_API OTV_GlyphData otv__construct_RectangleData (
	const float s, const float color, const float width, const float height
);
#endif

/// @brief The function pointer type for the @c otv__construct_RectangleData() function.
typedef OTV_GlyphData(*otv__construct_RectangleData_funct)(const float, const float, const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_RectangleData()
extern otv__construct_RectangleData_funct otv__construct_RectangleData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_RectangleData

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_RectangleData()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_RectangleData()
 */
OTV_API OTV_GlyphData otv__construct_empty_RectangleData (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_RectangleData() function.
typedef OTV_GlyphData(*otv__construct_empty_RectangleData_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_RectangleData()
extern otv__construct_empty_RectangleData_funct otv__construct_empty_RectangleData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_RectangleData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphData struct to a @c OTV_RectangleData.
 *
 * @param rectangle_data The @c OTV_GlyphData to upcast.
 *
 * @return The upcasted @c OTV_RectangleData view on the given @c OTV_GlyphData.
 */
OTV_API OTV_RectangleData* otv__upcast_RectangleData (const OTV_GlyphData *rectangle_data);
#endif

/// @brief The function pointer type for the @c otv__upcast_RectangleData() function.
typedef OTV_RectangleData*(*otv__upcast_RectangleData_funct)(const OTV_GlyphData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_RectangleData()
extern otv__upcast_RectangleData_funct otv__upcast_RectangleData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_RectangleData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_RectangleData struct to a @c OTV_GlyphData.
 *
 * @param rectangle_data The @c OTV_RectangleData to downcast.
 *
 * @return The downcasted @c OTV_GlyphData view on the given @c OTV_RectangleData.
 */
OTV_API OTV_GlyphData* otv__downcast_RectangleData (const OTV_RectangleData *rectangle_data);
#endif

/// @brief The function pointer type for the @c otv__downcast_RectangleData() function.
typedef OTV_GlyphData*(*otv__downcast_RectangleData_funct)(const OTV_RectangleData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_RectangleData()
extern otv__downcast_RectangleData_funct otv__downcast_RectangleData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__instantiate_Rectangle

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Instantiates the given Rectangle glyph to calculate its geometry and returns its extents along the trajectory.
 *
 * @param traj_radius The radius of the trajectory on which to instantiate the Rectangle.
 * @param info The static glyph parameters to use during instantiation.
 * @param data The data to instantiate the Rectangle with.
 *
 * @return
 *		The extents of the given Rectangle glyph relative to its anchor position. @c OTV_Vec2::x will contain the radius
 *		in trailing direction of the trajectory, and @c OTV_Vec2::y the radius in leading direction.
 */
OTV_API OTV_Vec2 otv__instantiate_Rectangle (
	const float traj_radius, const OTV_RectangleInfo *info, const OTV_RectangleData *data
);
#endif

/// @brief The function pointer type for the @c otv__instantiate_Rectangle() function.
typedef OTV_Vec2(*otv__instantiate_Rectangle_funct)(const float, const OTV_RectangleInfo*, const OTV_RectangleData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__instantiate_Rectangle()
extern otv__instantiate_Rectangle_funct otv__instantiate_Rectangle;
#endif


////
// ####################################################################################################################
// OTV_SignBlobInfo

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_SignBlobInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_SignBlobInfo struct that will be correctly up- and downcastable.
 *
 * @param rgb The value for the field @c OTV_SignBlobInfo::rgb
 * @param color_map The value for the field @c OTV_SignBlobInfo::color_map
 * @param radius The value for the field @c OTV_SignBlobInfo::radius
 * @param value The value for the field @c OTV_SignBlobInfo::value
 * @param static_flags The value for the field @c OTV_SignBlobInfo::static_flags
 *
 * @return An instance of the @c OTV_SignBlobInfo struct, downcasted to the generic @c OTV_GlyphInfo.
 *
 * @note It is necessary to return a generic @c OTV_GlyphInfo view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_SignBlobInfo form, you can upcast it
 * using @c otv__upcast_SignBlobInfo().
 */
OTV_API OTV_GlyphInfo otv__construct_SignBlobInfo (
	const OTV_Rgb rgb, const OTV_ColorMap color_map, const float radius, const float value,
	const OTV_SignBlobInfoStaticFlags static_flags
);
#endif

/// @brief The function pointer type for the @c otv__construct_SignBlobInfo() function.
typedef OTV_GlyphInfo(*otv__construct_SignBlobInfo_funct)(
	const OTV_ColorMap color_map, const OTV_InterpolationMode interpolation_mode
);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_SignBlobInfo()
extern otv__construct_SignBlobInfo_funct otv__construct_SignBlobInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_SignBlobInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_SignBlobInfo()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_SignBlobInfo()
 */
OTV_API OTV_GlyphInfo otv__construct_empty_SignBlobInfo (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_SignBlobInfo() function.
typedef OTV_GlyphInfo(*otv__construct_empty_SignBlobInfo_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_SignBlobInfo()
extern otv__construct_empty_SignBlobInfo_funct otv__construct_empty_SignBlobInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_SignBlobInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphInfo struct to a @c OTV_SignBlobInfo.
 *
 * @param line_plot_info The @c OTV_GlyphInfo to upcast.
 *
 * @return The upcasted @c OTV_SignBlobInfo view on the given @c OTV_GlyphInfo.
 */
OTV_API OTV_SignBlobInfo* otv__upcast_SignBlobInfo (const OTV_GlyphInfo *line_plot_info);
#endif

/// @brief The function pointer type for the @c otv__upcast_SignBlobInfo() function.
typedef OTV_SignBlobInfo*(*otv__upcast_SignBlobInfo_funct)(const OTV_GlyphInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_SignBlobInfo()
extern otv__upcast_SignBlobInfo_funct otv__upcast_SignBlobInfo;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_SignBlobInfo

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_SignBlobInfo struct to a @c OTV_GlyphInfo.
 *
 * @param line_plot_info The @c OTV_SignBlobInfo to downcast.
 *
 * @return The downcasted @c OTV_GlyphInfo view on the given @c OTV_SignBlobInfo.
 */
OTV_API OTV_GlyphInfo* otv__downcast_SignBlobInfo (const OTV_SignBlobInfo *line_plot_info);
#endif

/// @brief The function pointer type for the @c otv__downcast_SignBlobInfo() function.
typedef OTV_GlyphInfo*(*otv__downcast_SignBlobInfo_funct)(const OTV_SignBlobInfo*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_SignBlobInfo()
extern otv__downcast_SignBlobInfo_funct otv__downcast_SignBlobInfo;
#endif


////
// ####################################################################################################################
// OTV_SignBlobData

// --------------------------------------------------------------------------------------------------------------------
// otv__construct_SignBlobData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Constructs an instance of the @c OTV_SignBlobData struct that will be correctly up- and downcastable.
 *
 * @param s The value for the field @c OTV_SignBlobData::s
 * @param color The value for the field @c OTV_SignBlobData::color
 * @param value The value for the field @c OTV_SignBlobData::width
 *
 * @return An instance of the @c OTV_SignBlobData struct, downcasted to the generic @c OTV_GlyphData.
 *
 * @note It is necessary to return a generic @c OTV_GlyphData view on the created instance because of limitations
 * of the C language. If you need access to the object in its concrete @c OTV_SignBlobData form, you can upcast it
 * using @c otv__upcast_SignBlobData().
 */
OTV_API OTV_GlyphData otv__construct_SignBlobData (const float s, const float color, const float value);
#endif

/// @brief The function pointer type for the @c otv__construct_SignBlobData() function.
typedef OTV_GlyphData(*otv__construct_SignBlobData_funct)(const float, const float, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_SignBlobData()
extern otv__construct_SignBlobData_funct otv__construct_SignBlobData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__construct_empty_SignBlobData

#ifndef OTV_NO_PROTOTYPES
/**
 * @copybrief otv__construct_SignBlobData()
 * All but the constant @c N will be left uninitialized.
 *
 * @copydetails otv__construct_SignBlobData()
 */
OTV_API OTV_GlyphData otv__construct_empty_SignBlobData (void);
#endif

/// @brief The function pointer type for the @c otv__construct_empty_SignBlobData() function.
typedef OTV_GlyphData(*otv__construct_empty_SignBlobData_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__construct_empty_SignBlobData()
extern otv__construct_empty_SignBlobData_funct otv__construct_empty_SignBlobData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__upcast_SignBlobData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Upcasts a @c OTV_GlyphData struct to a @c OTV_SignBlobData.
 *
 * @param sign_blob_data The @c OTV_GlyphData to upcast.
 *
 * @return The upcasted @c OTV_SignBlobData view on the given @c OTV_GlyphData.
 */
OTV_API OTV_SignBlobData* otv__upcast_SignBlobData (const OTV_GlyphData *sign_blob_data);
#endif

/// @brief The function pointer type for the @c otv__upcast_SignBlobData() function.
typedef OTV_SignBlobData*(*otv__upcast_SignBlobData_funct)(const OTV_GlyphData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__upcast_SignBlobData()
extern otv__upcast_SignBlobData_funct otv__upcast_SignBlobData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__downcast_SignBlobData

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Downcasts a @c OTV_SignBlobData struct to a @c OTV_GlyphData.
 *
 * @param sign_blob_data The @c OTV_SignBlobData to downcast.
 *
 * @return The downcasted @c OTV_GlyphData view on the given @c OTV_SignBlobData.
 */
OTV_API OTV_GlyphData* otv__downcast_SignBlobData (const OTV_SignBlobData *sign_blob_data);
#endif

/// @brief The function pointer type for the @c otv__downcast_SignBlobData() function.
typedef OTV_GlyphData*(*otv__downcast_SignBlobData_funct)(const OTV_SignBlobData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__downcast_SignBlobData()
extern otv__downcast_SignBlobData_funct otv__downcast_SignBlobData;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__instantiate_SignBlob

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Instantiates the given Sign Blob glyph to calculate its geometry and returns its extents along the trajectory.
 *
 * @param traj_radius The radius of the trajectory on which to instantiate the Sign Blob.
 * @param info The static glyph parameters to use during instantiation.
 * @param data The data to instantiate the Sign Blob with.
 *
 * @return
 *		The extents of the given Sign Blob glyph relative to its anchor position. @c OTV_Vec2::x will contain the radius
 *		in trailing direction of the trajectory, and @c OTV_Vec2::y the radius in leading direction.
 */
OTV_API OTV_Vec2 otv__instantiate_SignBlob (
	const float traj_radius, const OTV_SignBlobInfo *info, const OTV_SignBlobData *data
);
#endif

/// @brief The function pointer type for the @c otv__instantiate_SignBlob() function.
typedef OTV_Vec2(*otv__instantiate_SignBlob_funct)(const float, const OTV_SignBlobInfo*, const OTV_SignBlobData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__instantiate_SignBlob()
extern otv__instantiate_SignBlob_funct otv__instantiate_SignBlob;
#endif


////
// ####################################################################################################################
// Misc functions

// --------------------------------------------------------------------------------------------------------------------
// otv__string_from_ColorMap

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Returns a human-readable string representation of the given @c OTV_ColorMap enum.
 *
 * @param color_map The color map to return a string for.
 *
 * @return A human-readable string representation of the given @c OTV_ColorMap enum.
 */
OTV_API const char *const otv__string_from_ColorMap (const OTV_ColorMap color_map);
#endif

/// @brief The function pointer type for @c otv__string_from_ColorMap().
typedef const char *const(*otv__string_from_ColorMap_funct)(const OTV_ColorMap);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__string_from_ColorMap()
extern otv__string_from_ColorMap_funct otv__string_from_ColorMap;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__string_from_GlyphType

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Returns a human-readable string representation of the given @c OTV_GlyphType enum.
 *
 * @param glyph_type The glyph type to return a string for.
 *
 * @return A human-readable string representation of the given @c OTV_GlyphType enum.
 */
OTV_API const char *const otv__string_from_GlyphType (const OTV_GlyphType glyph_type);
#endif

/// @brief The function pointer type for @c otv__string_from_GlyphType().
typedef const char *const(*otv__string_from_GlyphType_funct)(const OTV_GlyphType);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__string_from_GlyphType()
extern otv__string_from_GlyphType_funct otv__string_from_GlyphType;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__string_from_InterpolationMode

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Returns a human-readable string representation of the given @c OTV_InterpolationMode enum.
 *
 * @param interpolation_mode The glyph type to return a string for.
 *
 * @return A human-readable string representation of the given @c OTV_InterpolationMode enum.
 */
OTV_API const char *const otv__string_from_InterpolationMode (const OTV_InterpolationMode interpolation_mode);
#endif

/// @brief The function pointer type for @c otv__string_from_InterpolationMode().
typedef const char *const(*otv__string_from_InterpolationMode_funct)(const OTV_InterpolationMode);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__string_from_InterpolationMode()
extern otv__string_from_InterpolationMode_funct otv__string_from_InterpolationMode;
#endif



//////
//
// Language config [CLOSE]
//

#ifdef _MSC_VER
	#pragma warning(pop)
#endif


#endif // ifdef __GLYPH_TYPES_H__
