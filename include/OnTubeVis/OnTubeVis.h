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

/// @brief Structure encapsulating a single @link otv__extrapolation_length extrapolated @endlink segment.
typedef struct OTV_Extrapolation
{
	/// @brief The end Hermite node of the extrapolated segment.
	OTV_HermiteNode node;

	/// @brief The arc length approximation for the extrapolated segment.
	OTV_SegmentArclen arclen;
} OTV_Extrapolation;



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
 * @brief
 *		Stream a spline node (made up of position, tangent and a timestamp) to the indicated trajectory, when the
 *		active visualization is @link otv__extrapolation_length set up @endlink to @b not use smooth
 *		extrapolation.
 *
 * To accommodate time-critical realtime processes, this function is "fire-and-forget", i.e. no checks will be done to
 * determine whether the sample was processed successfully. Instead, the function will return immediatly after the
 * command to add the sample was submitted.
 *
 * Using this function on a @link otv__extrapolation_length setup that uses at least one extrapolation segment @endlink
 * will cause the extrapolation to be @a point-like, i.e. all extrapolated nodes will be set to the position of the
 * passed-in spline node.
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
// otv__stream_spline_node_and_extrapol

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Stream a spline node (made up of position, tangent and a timestamp) to the indicated trajectory, as well as the
 *		extrapolated path @a following the node that should be used for smooth position refreshes and displaying
 *		as-of-yet orphaned glyphs.
 *
 * To accommodate time-critical realtime processes, this function is "fire-and-forget", i.e. no checks will be done to
 * determine whether the sample was processed successfully. Instead, the function will return immediately after the
 * command to add the sample was submitted.
 *
 * Using this function on a @link otv__extrapolation_length setup that uses no extrapolation segments @endlink
 * will cause the provided extrapolation to be ignored.
 *
 * @param traj_id The trajectory to stream the node to.
 * @param node The Hermite node to stream.
 * @param arclen
 *		The approximate arclength parameterization of the segment between the most recent and this new node. In case of
 *		the very first sample in a trajectory, this parameter is ignored (it may be @c NULL).
 * @param extrapol
 *		Pointer to an array of extrapolations forming the extrapolated path after the provided @a node. The number of
 *		elements pointed-to by this parameter must not be less than the @link otv__extrapolation_length number of
 *		extrapolated segments @endlink configured during setup.
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
OTV_API void otv__stream_spline_node_and_extrapol (
	const uint32_t traj_id, const OTV_HermiteNode *node, const OTV_SegmentArclen *arclen,
	const OTV_Extrapolation *extrapol
);
#endif

/// @brief The function pointer type for the @c otv__stream_spline_node_and_extrapol() function.
typedef void(*otv__stream_spline_node_and_extrapol_funct)(
	const uint32_t, const OTV_HermiteNode*, const OTV_SegmentArclen*, const OTV_Extrapolation*
);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__stream_spline_node_and_extrapol()
extern otv__stream_spline_node_and_extrapol_funct otv__stream_spline_node_and_extrapol;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__compute_arclen

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Compute an arc length parameterization for the given Hermite curve.
 *
 * The arc length up to the segment currently being parameterized, typically called @a sigma, can be provided as well,
 * resulting in all values of @a s computed by the resulting parameterization being offset by this value.
 *
 * The implementation will typically perform a numerical approximation from scratch, which is relatively slow (albeit
 * fairly accurate). Consider using information available to you in your data to infer arc length instead of relying on
 * this function.
 *
 * @note
 *		This computation is typically executed immediately and synchronously inside the thread of the caller, without
 *		generating a command that requires waiting for an answer containing the computation result (the reference
 *		implementation by the OnTubeVis desktop application, for example, does it like this).
 *
 * @param node0 The start Hermite node of the curve.
 * @param node1 The end Hermite node of the curve.
 * @param sigma
 *		A constant offset to apply to the parameterization, e.g. for incorporating cumulative arc length over a whole
 *		spline up to @a node0 .
 *
 * @return An arc length parameterization for the input curve. See @c OTV_SegmentArclen for details.
 */
OTV_API OTV_SegmentArclen otv__compute_arclen (
	const OTV_HermiteNode *node0, const OTV_HermiteNode *node1, const float sigma
);
#endif

/// @brief The function pointer type for the @c otv__compute_arclen() function.
typedef OTV_SegmentArclen(*otv__compute_arclen_funct)(
	const OTV_HermiteNode *node0, const OTV_HermiteNode *node1, const float sigma
);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__compute_arclen()
extern otv__compute_arclen_funct otv__compute_arclen;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__eval_arclen

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Evaluate @a s(t), where @a s is defined by the coefficients of the provided arc length parameterization, to
 *		compute the arc length @a s at the given (time-like) curve parameter @a t.
 *
 * The parameter @a t can be any number, but the arc length parameterization strategy used in the OnTubeVis API by
 * construction only gives reliable results for <em>t=0..1</em> on the corresponding segment, and can potentially even
 * <em>decrease</em> again for <em>t≥1</em> (or increase for <em>t≤0</em>)
 *
 * @note
 *		This computation is typically executed immediately and synchronously inside the thread of the caller, without
 *		generating a command that requires waiting for an answer containing the computation result (the reference
 *		implementation by the OnTubeVis desktop application, for example, does it like this).
 *
 * @param s The arc length parametrization to evaluate.
 * @param t The (time-like) curve parameter to evaluate the arc length for.
 *
 * @return @a s(t), i.e. the arc length @a s at curve parameter @a t.
 */
OTV_API float otv__eval_arclen (const OTV_SegmentArclen *s, const float t);
#endif

/// @brief The function pointer type for the @c otv__compute_arclen() function.
typedef float(*otv__eval_arclen_funct)(const OTV_SegmentArclen*, const float);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__eval_arclen()
extern otv__eval_arclen_funct otv__eval_arclen;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__compute_extrapol

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief
 *		Create a primitive extrapolation of the desired length given one reference Hermite segment to extrapolate from.
 *
 * It is up to the implementation how the extrapolation is done exactly. The OnTubeVis desktop application currently
 * just adds perfectly straight segments with the same velocity as the end node of the provided segment.
 *
 * @param out Pointer to an array of extrapolations large enough to hold @a num elements.
 * @param num The desired number of segments in the extrapolation. Must be at least 1.
 * @param ref_node0 The start Hermite node of the reference segment.
 * @param ref_node1 The end Hermite node of the reference segment.
 * @param ref_arclen The arc length re-parametrization of the reference segment.
 */
OTV_API void otv__compute_extrapol (
	OTV_Extrapolation *out, const uint32_t num, const OTV_HermiteNode *ref_node0, const OTV_HermiteNode *ref_node1,
	const OTV_SegmentArclen *ref_arclen
);
#endif

/// @brief The function pointer type for the @c otv__compute_extrapol() function.
typedef void(*otv__compute_extrapol_funct)(
	OTV_Extrapolation*, const uint32_t, const OTV_HermiteNode*, const OTV_HermiteNode*, const OTV_SegmentArclen*
);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__compute_extrapol()
extern otv__compute_extrapol_funct otv__compute_extrapol;
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
 *		arc length! Glyphs that arrive <b>out-of-order</b> will cause <b>undefined behavior</b>.
 */
OTV_API void otv__stream_glyph (const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *glyph_data);
#endif

/// @brief The function pointer type for the @c otv__stream_glyph() function.
typedef void(*otv__stream_glyph_funct)(const uint32_t, const uint32_t, const OTV_GlyphData*);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__stream_glyph()
extern otv__stream_glyph_funct otv__stream_glyph;
#endif


// --------------------------------------------------------------------------------------------------------------------
// otv__stream_glyphs

#ifndef OTV_NO_PROTOTYPES
/**
 * @brief Stream a number of glyph instances to the specified layer on the specified trajectory.
 *
 * To accomodate time-critical realtime processes, this function is "fire-and-forget", i.e. no checks will be done to
 * determine whether the glyphs were processed successfully. Instead, the function will return immediatly after the
 * command to add the glyphs was submitted.
 *
 * @param traj_id The trajectory to stream the glyphs to.
 * @param layer The on-tube layer the glyphs are for.
 * @param glyphs_data
 *		The parameters for instantiating the glyphs. Responsibility for making sure the proper specialization of the
 *		@c OTV_GlyphData struct is used for the specified layer lies with the caller.
 * @param num_glyphs The number of glyph instances pointed to by @a glyphs_data.
 *
 * @note
 *		Glyphs within a layer on a trajectory are expected to be submitted in <b>monotonically increasing order</b> of
 *		arc length! Glyphs that arrive <b>out-of-order</b> will cause <b>undefined behavior</b>.
 */
OTV_API void otv__stream_glyphs (
	const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *glyphs_data, const uint32_t num_glyphs
);
#endif

/// @brief The function pointer type for the @c otv__stream_glyphs() function.
typedef void(*otv__stream_glyphs_funct)(const uint32_t, const uint32_t, const OTV_GlyphData*, const uint32_t);

#ifdef OTV_NO_PROTOTYPES
/// @copydoc otv__stream_glyphs()
extern otv__stream_glyphs_funct otv__stream_glyphs;
#endif


#endif // ifdef __ONTUBEVIS_H__
