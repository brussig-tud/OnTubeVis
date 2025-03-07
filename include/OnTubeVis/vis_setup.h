/**
 * @file
 * @brief Sub header exposing the visualization setup API.
 */

#ifndef __VIS_SETUP_H__
#define __VIS_SETUP_H__


//////
//
// Includes
//

// Public API
#include <OnTubeVis/glyph_types.h>



//////
//
// Typedefs and structs
//

/// @brief A struct storing the configuration for a single on-tube visualization layer.
typedef struct OTV_LayerConfig
{
	/// @brief The glyph/plot type to be used on this layer.
	OTV_GlyphType type;

	/**
	 * @brief
	 *		The thickness of the outline to be drawn around glyphs/plots on this layer. Some glyph or plot types don't
	 *		actually support outlines (or it would not make sense for them to support outlines); in this case the value
	 *		assigned to this field will be ignored.
	 */
	float outline;

	/// @brief Additional glyph/plot-specific static properties that apply for all samples visualized on this layer.
	OTV_GlyphInfo static_params;
} OTV_LayerConfig;

/// @brief A handle type representing a visualization setup.
typedef struct OTV_VisSetup OTV_VisSetup;

/// @brief The typed handle for @link OTV_VisSetup visualization setups @endlink.
typedef OTV_VisSetup *OTV_VisSetupHandle;



//////
//
// Functions
//

// --------------------------------------------------------------------------------------------------------------------
// otv__create_VisSetup

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Allocates a @link OTV_VisSetup visualization setup @endlink. Must be deallocated via @c otv__free_VisSetup()
 *		when not needed anymore, e.g. after the setup has been used to spawn a visualization session.
 *
 * @param name The name the visualization should show up as within OnTubeVis.
 *
 * @return A handle to the newly created visualization setup.
 */
OTV_API OTV_VisSetupHandle otv__create_VisSetup (const char *const name);
#endif

/// @brief The function pointer type for the @c otv__create_VisSetup() function.
typedef OTV_VisSetupHandle(*otv__create_VisSetup_funct)(const char *const);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__create_VisSetup()
extern otv__create_VisSetup_funct otv__create_VisSetup;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__free_VisSetup

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		De-allocates a @link OTV_VisSetup visualization setup @endlink that was allocated using @c otv__create_VisSetup().
 *
 * @param vis_setup The handle to the visualization setup to destroy.
 */
OTV_API void otv__free_VisSetup (OTV_VisSetupHandle vis_setup);
#endif

/// @brief The function pointer type for the @c otv__free_VisSetup() function.
typedef void(*otv__free_VisSetup_funct)(OTV_VisSetupHandle);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__free_VisSetup()
extern otv__free_VisSetup_funct otv__free_VisSetup;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__add_trajectory

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Adds a trajectory to the given @link OTV_VisSetup visualization setup @endlink.
 *
 * @param vis_setup The visualization setup to change the number of trajectories of.
 * @param radius The desired initial radius for the trajectory.
 *
 * @return
 *		The ID of the new trajectory which it will be accessible by within the visualization session resulting from
 *		the setup, e.g. via @c otv__stream_spline_sample() or @c otv__stream_glyph().
 */
OTV_API uint32_t otv__add_trajectory (OTV_VisSetupHandle vis_setup, const float radius);
#endif

/// @brief The function pointer type for the @c otv__add_trajectory() function.
typedef uint32_t(*otv__add_trajectory_funct)(OTV_VisSetupHandle);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__add_trajectory()
extern otv__add_trajectory_funct otv__add_trajectory;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__add_layer

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Add an on-tube visualization layer with the given configuration to a
 *		@link OTV_VisSetup visualization setup @endlink.
 *
 * Note that there can only be a maximum of 4 visualization layers active. Trying to add more than that will cause this
 * function to fail.
 *
 * @param vis_setup The visualization setup to add the layer to.
 * @param config The layer configuration to use for the new layer.
 *
 * @return @c true if the layer could be added, @c false otherwise (typically because there are already 4 layers).
 */
OTV_API bool otv__add_layer (OTV_VisSetupHandle vis_setup, const OTV_LayerConfig *config);
#endif

/// @brief The function pointer type for the @c otv__add_layer() function.
typedef bool(*otv__add_layer_funct)(OTV_VisSetupHandle, const OTV_LayerConfig*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__add_layer()
extern otv__add_layer_funct otv__add_layer;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__geo_reference

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Provide a geo reference for the trajectory data. Implementations may act on this by e.g. displaying map data, 3D
 *		buildings etc. around the trajectories.
 *
 * When a geo reference is provided for a @link OTV_VisSetup visualization setup @endlink, then all 3D coordinates
 * submitted to the API under this setup will be interpreted as being Cartesian coordinates with unit 1 meter, obtained
 * from transforming @a UTM/WGS84 coordinates using this reference point for the Cartesian origin <code>(0,0,0)</code>.
 *
 * @note
 *		The API does not define what exactly implementations should do with the geographic reference, or if they should
 *		do anything at all with it.
 *
 * @param vis_setup The visualization setup to attach the geo reference to.
 * @param latitude The latitude of the geo reference.
 * @param longitude The latitude of the geo reference.
 */
OTV_API void otv__geo_reference (OTV_VisSetupHandle vis_setup, const double latitude, const double longitude);
#endif

/// @brief The function pointer type for the @c otv__geo_reference() function.
typedef void(*otv__geo_reference_funct)(OTV_VisSetupHandle, const double, const double);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__geo_reference()
extern otv__geo_reference_funct otv__geo_reference;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__extrapolation_length

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Set the amount of Hermite spline segments that should be used for smooth extrapolation of the trajectory while
 *		waiting for new @link otv__stream_spline_node_and_extrapol spline nodes @endlink to arrive. Setting this to 0
 *		disables extrapolation display entirely.
 *
 * In the time between submission of spline nodes, which can (and ideally should) be streamed relatively infrequent
 * (≤ 1/s) to utilize the expressiveness of cubic curves, an extrapolation can be displayed. This enables (a) smooth,
 * continuous updates of the trajectory at screen refresh rates and (b) displaying glyphs / updating plots on the
 * current, to-be-completed segment pending an actual position measurement.
 *
 * The API does not provide or use any sort of predictive models, instead the prediction should be made by the client
 * and submitted to the API. Depending on the sophistication that clients want in their prediction, the number of
 * extrapolated segments can be adjusted to account for more complex paths.
 *
 * When a new measurement arrives, implementations are expected to subdivide the current segment of the extrapolation at
 * the exact time of the measurement, and smoothly morph the resulting set of extrapolated segments onto the actually
 * measured new segment. Exactly how this happens is up to the implementation.
 *
 * @param vis_setup The visualization setup to set the extrapolation length for.
 * @param num_segments The desired length of the trajectory extrapolations, in segments.
 */
OTV_API void otv__extrapolation_length (OTV_VisSetupHandle vis_setup, const uint32_t num_segments);
#endif

/// @brief The function pointer type for the @c otv__extrapolation_length() function.
typedef void(*otv__extrapolation_length_funct)(OTV_VisSetupHandle, const double, const double);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__extrapolation_length()
extern otv__extrapolation_length_funct otv__extrapolation_length;
#endif


#endif // ifdef __VIS_SETUP_H__
