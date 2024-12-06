/**
 * @file
 * @brief The main include file containing the core API and including other, more specialized modules.
 */

#ifndef __ONTUBEVIS_H__
#define __ONTUBEVIS_H__


//////
//
// Includes
//

// C standard library
#include <stdbool.h>

// Public API
#include <OnTubeVis/vis_setup.h>



//////
//
// Structs and typedefs
//

/// @brief The return value type for @c otv__terminate().
typedef struct OTV_TerminateResult
{
	/// @brief In case @c terminated is @c true, contains the exit code of the OnTubeVis main loop.
	int exit_code;

	/// @brief Indicates whether the OnTubeVis implementation actually terminated.
	bool terminated;
} OTV_TerminateResult;

/// @brief A timestamped Hermite spline node.
typedef struct OTV_HermiteNode
{
	/// @brief The sample timestamp associated with this node.
	float time;

	/// @brief The 3D position of node.
	OTV_Vec3 position;

	/// @brief The 3D tangent vector of the node.
	OTV_Vec3 tangent;
} OTV_HermiteNode;

/// @brief Struct representing a Hermite segment's arc length parameterization.
typedef struct OTV_SegmentArclen {
	/**
	 * @brief
	 *		A 4-span piece-wise cubic polynomial function approximating a Hermite segment's arc length parameterization.
	 */
	OTV_Vec4 coeffs[4];
} OTV_SegmentArclen;



//////
//
// Core Functions
//

// --------------------------------------------------------------------------------------------------------------------
// otv__startup

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Initializes and starts the OnTubeVis main event loop. Since startup can take a while, this call will not block.
 *		Use @c otv__wait_for_startup() to wait until the implementation is ready to receive commands and check if the
 *		initialization was actually successful.
 *
 * @b Control @b flow. OnTubeVis is typically its own application, and the CGV Framework used internally always operates
 * under the assumption that it runs its own host process. To emulate this when using OnTubeVis as a module, the calling
 * process must enter into the execution of OnTubeVis as if it was the operating system, i.e. it must hand off control
 * to the @c main() function of the CGV Framework. This function handles all of that and hosts the OnTubeVis main loop
 * in its own, specially spawned thread.
 *
 * @b Arguments. We adopt the convention that the very first argument, required to contain the <em>command name</em> by
 * the POSIX standard, should point to the library file containing the OnTubeVis service. @c otv__startup() will prepend
 * this first argument automatically – the arguments passed to this function should only contain actual, functional args
 * without the initial <em>command name</em>.
 *
 * Arguments that the OnTubeVis implementation recognizes include all arguments known to the CGV Framework, as well as
 * the following OnTubeVis-specific options:
 * @todo Document the command line parameters.
 *
 * @param argc
 *		The number of arguments to pass to the internal launcher.
 * @param argv
 *		C-style array of C-style strings containing the arguments for the internal launcher. The provided memory pointed
 *		to by each item in @a argv is guaranteed to remain unchanged (hence it is declared @c const).
 *
 * @return
 *		@c true if the OnTubeVis implementation has started its initialization, @c false otherwise – most notably, if
 *		the implementation was already running when the call was made.
 *
 * @see otv__terminate()
 */
OTV_API bool otv__startup (const int argc, const char *const *argv);
#endif

/// @brief The function pointer type for @c otv__startup().
typedef bool(*otv__startup_funct)(const int, const char *const *);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__startup()
extern otv__startup_funct otv__startup;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__wait_for_startup

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Waits until the startup process initiated by @c otv__startup() is complete and the OnTubeVis implementation
 *		can receive commands.
 *
 * @return @c true if the OnTubeVis implementation has successfully started, @c false if the startup was unsuccessful.
 */
OTV_API bool otv__wait_for_startup (void);
#endif

/// @brief The function pointer type for the @c otv__wait_for_startup() function.
typedef bool(*otv__wait_for_startup_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__wait_for_startup()
extern otv__wait_for_startup_funct otv__wait_for_startup;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__terminate

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Requests the OnTubeVis main event loop to terminate and shut down.
 *
 * @return The status after the termination request. See @c OTV_TerminateResult for details.
 *
 * @note
 *		Due to potentially mutable global state kept by the OnTubeVis implementation that gets initialized upon module
 *		loading, it should <b>not</b> be assumed that starting up the main loop again using @c otv__startup() (or
 *		@c main() directly) after calling @c otv__terminate() will result in a functioning instance of the service. If
 *		restarting OnTubeVis is requried, the library should be unloaded and reloaded again.
 * @note Calling this without first invoking @c otv__startup() (or @c main() directly) is undefined behavior.
 */
OTV_API OTV_TerminateResult otv__terminate (void);
#endif

/// @brief The function pointer type for the @c otv__terminate() function.
typedef OTV_TerminateResult(*otv__terminate_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__terminate()
extern otv__terminate_funct otv__terminate;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__get_module_filepath

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Reports the file path of the currently loaded OnTubeVis library file.
 *
 * Can be used for setting the 0th command line argument to be passed to the OnTubeVis @c main() function in case the
 * library was linked against directly. There should typically be no need to use this whatsoever, since the preferred
 * way to launch the OnTubeVis implementation is @c otv__startup(), which takes care of this automatically. When using
 * <c>dlopen()</c>-like functionality, this path will already be known a-priori by the client anyway, but it can be used
 * to check the location of the library the module was loaded from by the OS during process startup.
 *
 * @return A C-style string containing the filepath of the loaded library implementing the OnTubeVis API.
 */
OTV_API const char * otv__get_module_filepath (void);
#endif

/// @brief The function pointer type for @c otv__get_module_filepath().
typedef const char*(*otv__get_module_filepath_funct)(void);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__get_module_filepath()
extern otv__get_module_filepath_funct otv__get_module_filepath;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__start_vis_session

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Start the visualization with the given setup.
 *
 * When this function returns successfully, clients may start streaming samples to the trajectories created during
 * @link OTV_VisSetup visualization setup @endlink. There can only be one session active at a time, hence there is no
 * handle for the session started this way. A new session can be started (replacing the currently running one, if any)
 * by calling @c otv__start_vis_session again with a new (or the same) setup.
 *
 * @param vis_setup The visualization setup to use.
 *
 * @return @c true if the session could be started, @c false otherwise.
 */
OTV_API bool otv__start_vis_session (OTV_VisSetupHandle vis_setup);
#endif

/// @brief The function pointer type for the @c otv__start_vis_session() function.
typedef bool(*otv__start_vis_session_funct)(OTV_VisSetupHandle);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__start_vis_session()
extern otv__start_vis_session_funct otv__start_vis_session;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__instantiate_Glyph

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Instantiates the given glyph to calculate its geometry and returns its extents along the trajectory.
 *
 * Requires an active visualization session to determine the static glyph parameters.
 *
 * @param traj_id The id of the trajectory to instantiate the glyph on.
 * @param layer The on-tube layer to instantiate the glyph on.
 * @param data The data to instantiate the glyph with.
 *
 * @return
 *		The extents of the given glyph relative to its anchor position. @c OTV_Vec2::x will contain the radius in
 *		trailing direction of the trajectory, and @c OTV_Vec2::y the radius in leading direction.
 */
OTV_API OTV_Vec2 otv__instantiate_Glyph (const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *data);
#endif

/// @brief The function pointer type for the @c otv__instantiate_Glyph() function.
typedef OTV_Vec2(*otv__instantiate_Glyph_funct)(const uint32_t, const uint32_t, const OTV_GlyphData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__instantiate_Glyph()
extern otv__instantiate_Glyph_funct otv__instantiate_Glyph;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__stream_spline_node

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Stream a spline node (made up of position, tangent and a timestamp) to the indicated trajectory.
 *
 * To accomodate time-critical realtime processes, this function is "fire-and-forget", i.e. no checks will be done to
 * determine whether the sample was processed successfully. Instead, the function will return immediatly after the
 * command to add the sample was submitted.
 *
 * @param traj_id The trajectory to stream the node to.
 * @param node The Hermite node to stream.
 * @param arclen
 *		The approximate arclength parameterization of the segment between the most recent and this new node. In case of
 *		the very first sample in a trajectory, this parameter is ignored (it may be @c NULL).
 *
 * @note
 *		Nodes of a trajectory are expected to be submitted in <b>monotonically increasing time-order</b>! Nodes that
 *		arrive <b>out-of-order</b> may be <b>discarded</b> by the implementation.
 *
 * @todo
 *		If nodes are discarded due to wrong ordering, the trajectory arc length seen by OnTubeVis and the one assumed by
 *		the client will be out-of-sync. This will never happen for direct in-memory communication (unless the client is
 *		buggy), but can become a real issue if streaming is done over the network. <b>ToDo:</b> Investigate strategies
 *		to deal with that.
 */
OTV_API void otv__stream_spline_node (
	const uint32_t traj_id, const OTV_HermiteNode *node, const OTV_SegmentArclen *arclen
);
#endif

/// @brief The function pointer type for the @c otv__stream_spline_node() function.
typedef void(*otv__stream_spline_node_funct)(const uint32_t, const OTV_HermiteNode*, const OTV_SegmentArclen*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__stream_spline_node()
extern otv__stream_spline_node_funct otv__stream_spline_node;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__stream_glyph

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Stream a single glyph instance to the specified layer on the specified trajectory.
 *
 * To accomodate time-critical realtime processes, this function is "fire-and-forget", i.e. no checks will be done to
 * determine whether the glyph was processed successfully. Instead, the function will return immediatly after the
 * command to add the glyph was submitted.
 *
 * @param traj_id The trajectory to stream the glyph to.
 * @param layer The on-tube layer the glyph is for.
 * @param glyph_data
 *		The parameters for instantiating the glyph. Responsibility for making sure the proper specialization of the
 *		@c OTV_GlyphData struct is used for the specified layer lies with the caller.
 *
 * @note
 *		Glyphs within a layer on a trajectory are expected to be submitted in <b>monotonically increasing order</b> of
 *		arc length! Glyphs that arrive <b>out-of-order</b> may be <b>discarded</b> by the implementation.
 */
OTV_API void otv__stream_glyph (const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *glyph_data);
#endif

/// @brief The function pointer type for the @c otv__stream_glyph() function.
typedef void(*otv__stream_glyph_funct)(const uint32_t, const uint32_t, const OTV_GlyphData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__stream_glyph()
extern otv__stream_glyph_funct otv__stream_glyph;
#endif


#endif // ifdef __ONTUBEVIS_H__
