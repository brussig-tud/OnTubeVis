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

	/// @brief The thickness of the outline to be drawn around glyphs/plots on this layer.
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


#endif // ifdef __VIS_SETUP_H__
