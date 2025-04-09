
#include <vector>
#include <deque>
#include <optional>

#include "gpumem/ring_buffer.inl"
#include "render/state.h"

// Public streaming API
#include <thread>
#include <cgv/gui/gui_driver.h>
#include <OnTubeVis/OnTubeVis.h>

// Implemented header
#include "otv_client.h"

//#include <glm/vec3.hpp>

#include "on_tube_vis.h"


namespace otv {

OTV_ColorMap colormap_name_to_api_enum (const std::string &name)
{
	const std::string name_lower = cgv::utils::to_lower(name);

	// Sequential color maps
	if (name_lower == "acton")
		return OTV_ColorMap::Acton;
	if (name_lower == "bamako")
		return OTV_ColorMap::Bamako;
	if (name_lower == "batlow")
		return OTV_ColorMap::Batlow;
	if (name_lower == "batlowk")
		return OTV_ColorMap::BatlowK;
	if (name_lower == "batloww")
		return OTV_ColorMap::BatlowW;
	if (name_lower == "bilbao")
		return OTV_ColorMap::Bilbao;
	if (name_lower == "buda")
		return OTV_ColorMap::Buda;
	if (name_lower == "davos")
		return OTV_ColorMap::Davos;
	if (name_lower == "devon")
		return OTV_ColorMap::Devon;
	if (name_lower == "glasgow")
		return OTV_ColorMap::Glasgow;
	if (name_lower == "grayc")
		return OTV_ColorMap::GrayC;
	if (name_lower == "hawaii")
		return OTV_ColorMap::Hawaii;
	if (name_lower == "imola")
		return OTV_ColorMap::Imola;
	if (name_lower == "lajolla")
		return OTV_ColorMap::Lajolla;
	if (name_lower == "lapaz")
		return OTV_ColorMap::Lapaz;
	if (name_lower == "lipari")
		return OTV_ColorMap::Lipari;
	if (name_lower == "navia")
		return OTV_ColorMap::Navia;
	if (name_lower == "nuuk")
		return OTV_ColorMap::Nuuk;
	if (name_lower == "oslo")
		return OTV_ColorMap::Oslo;
	if (name_lower == "rainbow")
		return OTV_ColorMap::Rainbow;
	if (name_lower == "tokyo")
		return OTV_ColorMap::Tokyo;
	if (name_lower == "turbo")
		return OTV_ColorMap::Turbo;
	if (name_lower == "turku")
		return OTV_ColorMap::Turku;

	// Diverging color maps
	if (name_lower == "bam")
		return OTV_ColorMap::Bam;
	if (name_lower == "berlin")
		return OTV_ColorMap::Berlin;
	if (name_lower == "broc")
		return OTV_ColorMap::Broc;
	if (name_lower == "cork")
		return OTV_ColorMap::Cork;
	if (name_lower == "lisbon")
		return OTV_ColorMap::Lisbon;
	if (name_lower == "managua")
		return OTV_ColorMap::Managua;
	if (name_lower == "roma")
		return OTV_ColorMap::Roma;
	if (name_lower == "tofino")
		return OTV_ColorMap::Tofino;
	if (name_lower == "vanimo")
		return OTV_ColorMap::Vanimo;
	if (name_lower == "vik")
		return OTV_ColorMap::Vik;

	// Unknown/unsupported
	cgv::gui::get_gui_driver()->message(
		"Config refers to unknown or unsupported color map: "+name+"\nOnTubeVis will now most likely crash."
	);
	assert(false && "INTERNAL LOGIC ERROR: Unknown color map name!");
	return OTV_ColorMap::UndefinedColormap;
}

unsigned colormap_name_to_internal_id (const color_map_manager &colormap_mgr, const std::string &colormap_name)
{
	const auto &colormaps = colormap_mgr.ref_color_maps();
	const auto cm_lower = cgv::utils::to_lower(colormap_name);
	for (unsigned i=0; i<colormaps.size(); i++)
		if (cgv::utils::to_lower(colormaps[i].name) == cm_lower)
			return i;
	assert(false && "INTERNAL LOGIC ERROR: Unknown/corrupted color_map name!");
	return -1;
}

unsigned colormap_api_enum_to_internal_id (const color_map_manager &colormap_mgr, const OTV_ColorMap &color_map)
{
	switch (color_map)
	{
		// Sequential color maps
		case OTV_ColorMap::Acton:
			return colormap_name_to_internal_id(colormap_mgr, "acton");
		case OTV_ColorMap::Bamako:
			return colormap_name_to_internal_id(colormap_mgr, "bamako");
		case OTV_ColorMap::Batlow:
			return colormap_name_to_internal_id(colormap_mgr, "batlow");
		case OTV_ColorMap::BatlowK:
			return colormap_name_to_internal_id(colormap_mgr, "batlowk");
		case OTV_ColorMap::BatlowW:
			return colormap_name_to_internal_id(colormap_mgr, "batloww");
		case OTV_ColorMap::Bilbao:
			return colormap_name_to_internal_id(colormap_mgr, "bilbao");
		case OTV_ColorMap::Buda:
			return colormap_name_to_internal_id(colormap_mgr, "buda");
		case OTV_ColorMap::Davos:
			return colormap_name_to_internal_id(colormap_mgr, "davos");
		case OTV_ColorMap::Devon:
			return colormap_name_to_internal_id(colormap_mgr, "devon");
		case OTV_ColorMap::Glasgow:
			return colormap_name_to_internal_id(colormap_mgr, "glasgow");
		case OTV_ColorMap::GrayC:
			return colormap_name_to_internal_id(colormap_mgr, "grac");
		case OTV_ColorMap::Hawaii:
			return colormap_name_to_internal_id(colormap_mgr, "hawaii");
		case OTV_ColorMap::Imola:
			return colormap_name_to_internal_id(colormap_mgr, "imola");
		case OTV_ColorMap::Lajolla:
			return colormap_name_to_internal_id(colormap_mgr, "lajolla");
		case OTV_ColorMap::Lapaz:
			return colormap_name_to_internal_id(colormap_mgr, "lapaz");
		case OTV_ColorMap::Lipari:
			return colormap_name_to_internal_id(colormap_mgr, "lipari");
		case OTV_ColorMap::Nuuk:
			return colormap_name_to_internal_id(colormap_mgr, "nuuk");
		case OTV_ColorMap::Oslo:
			return colormap_name_to_internal_id(colormap_mgr, "oslo");
		case OTV_ColorMap::Rainbow:
			return colormap_name_to_internal_id(colormap_mgr, "rainbow");
		case OTV_ColorMap::Tokyo:
			return colormap_name_to_internal_id(colormap_mgr, "tokyo");
		case OTV_ColorMap::Turbo:
			return colormap_name_to_internal_id(colormap_mgr, "turbo");
		case OTV_ColorMap::Turku:
			return colormap_name_to_internal_id(colormap_mgr, "turku");

		// Diverging color maps
		case OTV_ColorMap::Bam:
			return colormap_name_to_internal_id(colormap_mgr, "bam");
		case OTV_ColorMap::Berlin:
			return colormap_name_to_internal_id(colormap_mgr, "berlin");
		case OTV_ColorMap::Broc:
			return colormap_name_to_internal_id(colormap_mgr, "broc");
		case OTV_ColorMap::Cork:
			return colormap_name_to_internal_id(colormap_mgr, "cork");
		case OTV_ColorMap::Lisbon:
			return colormap_name_to_internal_id(colormap_mgr, "lisbon");
		case OTV_ColorMap::Managua:
			return colormap_name_to_internal_id(colormap_mgr, "managua");
		case OTV_ColorMap::Roma:
			return colormap_name_to_internal_id(colormap_mgr, "roma");
		case OTV_ColorMap::Tofino:
			return colormap_name_to_internal_id(colormap_mgr, "tofino");
		case OTV_ColorMap::Vanimo:
			return colormap_name_to_internal_id(colormap_mgr, "vanimo");
		case OTV_ColorMap::Vik:
			return colormap_name_to_internal_id(colormap_mgr, "vik");

		default:
			/* see below */;
	}

	// Unknown/unsupported
	cgv::gui::get_gui_driver()->message(
		"Config refers to unknown (potentially corrupted) color map: "+std::to_string(color_map)
		+"\nOnTubeVis will now most likely crash."
	);
	assert(false && "INTERNAL LOGIC ERROR: Unknown/corrupted color_map!");
	return -1;
}

OTV_InterpolationMode interpolation_attribval_to_api_enum (const float value)
{
	if (value < 1/3.f)
		return OTV_InterpolationMode::Nearest;
	if (value < 2/3.f)
		return OTV_InterpolationMode::Linear;
	return OTV_InterpolationMode::Cubic;
}

float interpolation_api_enum_to_attribval (const OTV_InterpolationMode interpolation_mode)
{
	switch (interpolation_mode) {
		case OTV_InterpolationMode::Nearest: return 0;
		case OTV_InterpolationMode::Linear: return .5f;
		case OTV_InterpolationMode::Cubic: return 1;
		default:
			/* see below */;
	}

	// Unknown/unsupported
	cgv::gui::get_gui_driver()->message(
		"Config refers to unknown (potentially corrupted) interpolation mode: "+std::to_string(interpolation_mode)
		+"\nOnTubeVis will now most likely crash."
	);
	assert(false && "INTERNAL LOGIC ERROR: Unknown/corrupted interpolation_mode!");
	return -1;
}

void otv_client::new_session (void) {
	session.reset();
}

bool otv_client::new_session_if_not_in_setup (void) {
	if (!session.is_setup_pending()) {
		session.reset();
		return true;
	}
	return false;
}

void otv_client::begin_setup (const std::string &name) {
	session.start_setup(otv__create_VisSetup(name.c_str()));
}

void otv_client::commit_session (void)
{
	// Transition out of setup state
	OTV_VisSetupHandle setup = session.finish_setup();

	// Add trajectories to setup
	for (unsigned i=0; i<trajectories.size(); i++) {
		auto &traj = trajectories[i];
		traj.id = otv__add_trajectory(setup, data->datasets[0].trajs[i].med_radius);
	}

	// Add layers to setup
	const auto &colormaps = render.visualizations.front().variables->ref_color_map_names();
	const auto &vis = render.visualizations.front();
	for (unsigned i=0; i<vis.config.layer_configs.size(); i++)
	{
		const auto &lcfg = vis.config.layer_configs[i];
		const auto &lmappings = vis.manager.ref_glyph_attribute_mappings()[i];
		switch (lcfg.shape_ptr->type())
		{
			case GT_COLOR:
			{
				// obtain 'interpolate' meta attribute
				constexpr unsigned metaattrib_idx__interpolate=0;
				const auto interpolate= interpolation_attribval_to_api_enum(
					lmappings.ref_attrib_mapping_values()[metaattrib_idx__interpolate].w()
				);
				// get colormap being used
				constexpr unsigned vattrib_idx__color=1;
				const int cidx = lmappings.get_attrib_indices()[vattrib_idx__color];
				const std::optional<OTV_Rgb> color = [&]() -> std::optional<OTV_Rgb> {
					if (cidx < 0) {
						const auto &color = lmappings.ref_attrib_colors()[vattrib_idx__color];
						return otv__Rgb(color.R(), color.G(), color.B());
					}
					return {};
				}();
				const std::optional<OTV_ColorMap> colormap = [&]() -> std::optional<OTV_ColorMap> {
					if (cidx > -1)
						return colormap_name_to_api_enum(colormaps[cidx]);
					return {};
				}();
				// create the layer config
				OTV_LayerConfig cfg {
					OTV_GlyphType::SurfaceColor,
					-1, // unused
					otv__construct_SurfaceColorInfo(
						color.value_or(OTV_Rgb{}), colormap.value_or(OTV_ColorMap{}), interpolate,
						(OTV_SurfaceColorInfoStaticFlags)(color.has_value()?RI_STATIC_COLOR:0)
					)
				};
				otv__add_layer(setup, &cfg);
				break;
			}
			case GT_LINE_PLOT:
			{
				// obtain values of meta attributes
				constexpr unsigned metaattrib_idx__outline=0, metaattrib_idx__interpolate=1;
				const auto outline= lmappings.ref_attrib_mapping_values()[metaattrib_idx__outline].w();
				const auto interpolate= interpolation_attribval_to_api_enum(
					lmappings.ref_attrib_mapping_values()[metaattrib_idx__interpolate].w()
				);
				// build up information about sub plots
				const auto &attrib_sources = lmappings.get_attrib_indices();
				std::vector<OTV_Rgb> subplot_colors; subplot_colors.reserve(4);
				for (unsigned i=0; i<attrib_sources.size(); i++) {
					if (attrib_sources[i] > -1) {
						const auto &color = lmappings.ref_attrib_colors()[i-1];
						subplot_colors.emplace_back(otv__Rgb(color.R(), color.G(), color.B()));
					}
				}
				// create the layer config
				OTV_LayerConfig cfg {
					OTV_GlyphType::LinePlot,
					outline,
					otv__construct_LinePlotInfo(
						interpolate, subplot_colors.size(), subplot_colors.data()
					)
				};
				otv__add_layer(setup, &cfg);
				break;
			}
			case GT_RECTANGLE:
			{
				// obtain values of 'outline' meta attribute
				constexpr unsigned metaattrib_idx__outline=0;
				const auto outline= lmappings.ref_attrib_mapping_values()[metaattrib_idx__outline].w();
				// get static color or colormap
				constexpr unsigned vattrib_idx__color=1;
				const int cidx = lmappings.get_attrib_indices()[vattrib_idx__color];
				const std::optional<OTV_Rgb> color = [&]() -> std::optional<OTV_Rgb> {
					if (cidx < 0) {
						const auto &color = lmappings.ref_attrib_colors()[vattrib_idx__color];
						return otv__Rgb(color.R(), color.G(), color.B());
					}
					return {};
				}();
				const std::optional<OTV_ColorMap> colormap = [&]() -> std::optional<OTV_ColorMap> {
					if (cidx > -1)
						return colormap_name_to_api_enum(colormaps[cidx]);
					return {};
				}();
				// get static width/height if used
				constexpr unsigned vattrib_idx__length=2, vattrib_idx__height=3;
				const std::optional<float> width = [&]() -> std::optional<float> {
					const auto idx = lmappings.get_attrib_indices()[vattrib_idx__length];
					if (idx < 0)
						return lmappings.ref_attrib_mapping_values()[vattrib_idx__length].w();
					return {};
				}();
				const std::optional<float> height = [&]() -> std::optional<float> {
					const auto idx = lmappings.get_attrib_indices()[vattrib_idx__height];
					if (idx < 0)
						return lmappings.ref_attrib_mapping_values()[vattrib_idx__height].w();
					return {};
				}();
				// create the layer config
				OTV_LayerConfig cfg {
					OTV_GlyphType::Rect,
					outline,
					otv__construct_RectangleInfo(
						color.value_or(OTV_Rgb{}), colormap.value_or(OTV_ColorMap{}),
						width.value_or(0), height.value_or(0), (OTV_RectangleInfoStaticFlags)(
							  (color.has_value()?RI_STATIC_COLOR:0) | (width.has_value()?RI_STATIC_WIDTH:0)
							| (height.has_value()?RI_STATIC_HEIGHT:0)
						)
					)
				};
				otv__add_layer(setup, &cfg);
				break;
			}
			case GT_SIGN_BLOB:
			{
				// obtain values of 'outline' meta attribute
				constexpr unsigned metaattrib_idx__outline=0;
				const auto outline= lmappings.ref_attrib_mapping_values()[metaattrib_idx__outline].w();
				// obtain static 'size' visual attribute (this can never be mapped to a data attribute)
				constexpr unsigned vattrib_idx__size=1;
				const auto radius= lmappings.ref_attrib_mapping_values()[vattrib_idx__size].w();
				// get static color or colormap
				constexpr unsigned vattrib_idx__color=2;
				const int cidx = lmappings.get_attrib_indices()[vattrib_idx__color];
				const std::optional<OTV_Rgb> color = [&]() -> std::optional<OTV_Rgb> {
					if (cidx < 0) {
						const auto &color = lmappings.ref_attrib_colors()[vattrib_idx__color];
						return otv__Rgb(color.R(), color.G(), color.B());
					}
					return {};
				}();
				const std::optional<OTV_ColorMap> colormap = [&]() -> std::optional<OTV_ColorMap> {
					if (cidx > -1)
						return colormap_name_to_api_enum(colormaps[cidx]);
					return {};
				}();
				// get static 'value' visual attribute if used
				constexpr unsigned vattrib_idx__value=3;
				const std::optional<float> value = [&]() -> std::optional<float> {
					const auto idx = lmappings.get_attrib_indices()[vattrib_idx__value];
					if (idx < 0)
						return lmappings.ref_attrib_mapping_values()[vattrib_idx__value].w();
					return {};
				}();
				// create the layer config
				OTV_LayerConfig cfg {
					OTV_GlyphType::SignBlob,
					outline,
					otv__construct_SignBlobInfo(
						color.value_or(OTV_Rgb{}), colormap.value_or(OTV_ColorMap{}), radius,
						value.value_or(0), (OTV_SignBlobInfoStaticFlags)(
							  (color.has_value()?SBI_STATIC_COLOR:0) | (value.has_value()?SBI_STATIC_VALUE:0)
						)
					)
				};
				otv__add_layer(setup, &cfg);
				break;
			}

			case GT_TRIANGLE:
			case GT_CIRCLE:
			case GT_WEDGE:
			case GT_ARC_FLAT:
			case GT_ARC_ROUNDED:
			case GT_DROP:
			case GT_STAR:
			case GT_TEMPORAL_HEAT_MAP:
				cgv::gui::get_gui_driver()->message(
					"Config contains unimplemented glyph type: "+lcfg.shape_ptr->name()+"\n"
					"OnTubeVis will now most likely crash."
				);
				assert(false && "INTERNAL LOGIC ERROR: Unimplemented glyph type in layer configuration!");

			default:
				assert(false && "INTERNAL LOGIC ERROR: Unknown glyph type in layer configuration!");
		}
	}

	// Request starting the session
	std::thread init_runner([this, setup] {
		const bool session_start_success = otv__start_vis_session(setup);
		assert(session_start_success && "FATAL ERROR - otv_client: Failed to initialize streaming session!");
		this->session.signal_init_done();
		otv__free_VisSetup(setup);
	});
	init_runner.detach(); // let session init continue in the background
}

void otv_client::update ()
{
	#define DEBUG_OUTPUT 1

	// wait for the session to be ready if necessary (adds a very small constant overhead every update once the session
	// actually is ready)
	session.wait_init_ready();

	// extend all trajectories based on the animation time, emulating streaming
	std::vector<extrapol::node> extrapol_nodes;
	extrapol_nodes.reserve(num_extrapol_segments);
	#if DEBUG_OUTPUT
		cgv::utils::stopwatch sw(/* silent: */true);
		std::clog << "otv_client::update(): starting update for t="<<render.style.max_t<<"s\n";
	#endif
	for (auto &traj : trajectories) {
		// add new nodes, and thereby new segments
		for (
			auto &node_idx = traj.node_idcs.begin;
			node_idx < traj.node_idcs.end;
			++node_idx
		){
			// only add data up to the current playback time
			if (data->timestamps[node_idx] > render.style.max_t) {
				break;
			}

			auto target_traj = find_trajectory(traj.id);

			// construct first GPU node
			const auto col = data->colors[node_idx];
			const node_attribs new_node {
				{data->positions[node_idx], data->radii[node_idx]},
				{col.R(), col.G(), col.B(), 1},
				data->tangents[node_idx],
				{data->timestamps[node_idx], 0, 0, 0}
			};

			// the first node of each trajectory does not create a segment, so there is no arc
			// length parametrization
			const cgv::mat4 *t_to_s {nullptr};

			extrapol_nodes.clear();
			if (!target_traj.ref.is_empty())
			{
				// Compute arc length
				t_to_s = &arclen_data.t_to_s.at(traj.segment_idx);

				// Extrapolate from previous segment
				const auto &prev_node = target_traj.ref.most_recent_node();
				extrapol::compute_path(
					extrapol_nodes, num_extrapol_segments, prev_node, new_node, *t_to_s
				);
			}
			else
			{
				// Init with "blank" extrapolation (just the straight line segment that follows from the first tangent)
				const auto pos0 = cgv::vec3(new_node.pos_rad), tan = cgv::vec3(new_node.tangent);
				const auto fictitious_prev_node = node_attribs {
					cgv::vec4(pos0 - tan, new_node.pos_rad.w()),
					new_node.color, cgv::vec4(tan, 0), cgv::vec4(new_node.t.x()-1, 0, 0, 0)
				};
				const auto tan_len = tan.length();
				extrapol::compute_path(
					extrapol_nodes, num_extrapol_segments, fictitious_prev_node, new_node,
					arclen::single_linear_t_to_s(tan_len, -tan.length())
				);
			}

			// append a node, potentially creating a new segment
			enqueue_node(target_traj, new_node, t_to_s, extrapol_nodes);
			//render.enqueue_node(traj.id, node, t_to_s );

			if (t_to_s == nullptr) {
				continue;
			}

			// if the node created a segment, enqueue its glyphs and advance the index
			render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &)
			{
				// prelude
				const auto data  {glyphs[layer_idx].attribs.data.begin()};
				const auto cur_range = glyphs[layer_idx].ranges.at(traj.segment_idx);
				const auto begin {cur_range.i0};
				const auto end   {cur_range.end()};
				const unsigned stride = target_traj.ref.glyph_to_attrib_count(layer_idx, glyph_count_type{1});

				// skip glyphs we already submitted when handling the previous segment
				auto glyphs_attribs = ro_range {
					data + target_traj.ref.glyph_to_attrib_count(layer_idx, begin),
					data + target_traj.ref.glyph_to_attrib_count(layer_idx, end)
				};
				while (glyphs_attribs.begin != glyphs_attribs.end && *glyphs_attribs.begin <= traj.last_s[layer_idx])
					glyphs_attribs.begin += stride;

				// submit new unique glyphs
				const auto glyph_attribs_on_extrapol = enqueue_glyphs(
					target_traj, layer_idx, glyphs_attribs
				);

				// update last_s
				traj.last_s[layer_idx] = cur_range.n.value ? *(glyphs_attribs.end-stride) : traj.last_s[layer_idx];
			});

			++traj.segment_idx;
		}
	}

	// Flush changes to extrapolation
	const bool extrapol_flush_result = extrapol_mgr.flush_changes();
	#if DEBUG_OUTPUT
		std::clog << "otv_client::update(): flushing extrapolations - "<<(extrapol_flush_result ? "OK\n":"FAILURE\n")
		          << "otv_client::update(): took "<<sw.get_elapsed_time()*1000<<"ms\n";
	#endif
}
void otv_client::enqueue_node (
	trajectory_ref target, const node_attribs &node, const cgv::mat4 *t_to_s,
	const std::vector<extrapol::node> &extrapol
){
	render.enqueue_node(target.id, node, t_to_s);
	if (num_extrapol_segments)
		extrapol_mgr.replace_extrapolation(target.id, node, extrapol);
}

template <class Iter>
ro_range<Iter> otv_client::enqueue_glyphs (trajectory_ref traj, unsigned layer, const ro_range<Iter> &glyph_data)
{
	// Enqueue glyphs to "regular" trajectories
	traj.ref.enqueue_glyphs(layer, glyph_data);

	// Also submit the glyphs to the extrapolation manager for consideration if extrapolation display is enabled
	const auto glyphs_on_extrapol = [&]() -> ro_range<Iter> {
		if (num_extrapol_segments)
			return extrapol_mgr.consider_glyphs(traj.id, layer, glyph_data);
		return extrapol_mgr.skip_glyphs_before(layer, traj.ref.arclength(), glyph_data);
	}();

	// Done!
	otv_instance->session_taa_keep_sampling = true;
	return glyphs_on_extrapol;
}
template ro_range<std_vector_float_iter> otv_client::enqueue_glyphs (
	trajectory_ref, unsigned, const ro_range<std_vector_float_iter>&
);
template ro_range<std_deque_float_iter> otv_client::enqueue_glyphs (
	trajectory_ref, unsigned, const ro_range<std_deque_float_iter>&
);
template ro_range<float*> otv_client::enqueue_glyphs (trajectory_ref, unsigned, const ro_range<float*>&);

void otv_client::service_push_spline_node (
	unsigned traj_id, node_attribs &&node, const cgv::mat4 *t_to_s, std::vector<extrapol::node> &&extrapol
)
{
	// the static constant unit diagonal for use with bounding box updates below
	static const auto diag = []() -> cgv::vec3 {
		cgv::vec3 diag; diag.ones();
		return diag;
	}();

	// obtain the trajectory we want to stream into
	auto traj = find_trajectory(traj_id);

	// get the desired base color and radius of the trajectory from the dummy dataset
	const auto &traj_range = data->datasets[0].trajs[traj_id];
	const auto &ds_idx = *(std::pair<unsigned, unsigned>*)&(data->indices[traj_range.i0]);
	auto &color = data->colors[ds_idx.first];
	auto radius = data->radii[ds_idx.first];
	node.color.set(color.R(), color.G(), color.B(), 1);
	node.pos_rad.w() = radius; node.tangent.w() = 0;
	for (auto &e : extrapol) {
		// set extrapol node radius and color
		e.hnode.pos_rad.w() = radius; e.hnode.tangent.w() = 0;
		e.hnode.color.set(color.R(), color.G(), color.B(), 1);
		// also include in bbox while we're at it
		const auto pos = cgv::vec3(e.hnode.pos_rad);
		otv_instance->bbox.add_point(pos - diag*radius);
		otv_instance->bbox.add_point(pos + diag*radius);
	}
	enqueue_node(traj, node, traj.ref.is_empty() ? nullptr : t_to_s, extrapol);

	// update bounding box
	const auto pos = cgv::vec3(node.pos_rad);
	otv_instance->bbox.add_point(pos - diag*radius);
	otv_instance->bbox.add_point(pos + diag*radius);
	otv_instance->bbox_wire_rd.clear();
	otv_instance->bbox_wire_rd.add(
		otv_instance->bbox.get_center(), otv_instance->bbox.get_extent(), cgv::rgb(0.75f)
	);
	otv_instance->bbox_rd.clear();
	otv_instance->bbox_rd.add(otv_instance->bbox.get_center(), otv_instance->bbox.get_extent());
	if (otv_instance->session_first_node)
	{
		// intialize camera
		otv_instance->set_view();

		// start progression of time
		otv_instance->playback.tstart = otv_instance->render.style.max_t = node.t.x();
		otv_instance->playback.tend = std::numeric_limits<float>::infinity();
		otv_instance->playback.active = true;
		otv_instance->playback.timer.add_time();
		otv_instance->playback.speed = 1;//otv_instance->on_set(&otv_instance->playback.active);

		// done processing very first node of session
		otv_instance->session_first_node = false;
	}
	else
		otv_instance->update_scene_extents();
	otv_instance->session_taa_keep_sampling = true;
}

void otv_client::service_push_glyphs (unsigned traj_id, unsigned layer, std::vector<float> &&glyph_data)
{
	/* debug check */ {
		const unsigned num_floats = glyph_data.size();
		const unsigned stride = render.trajectories.front().glyph_to_attrib_count(
			layer, glyph_count_type{1}
		);
		assert(num_floats % stride == 0 && "INTERNAL LOGIC ERROR: glyph data size mismatch!");
	}
	const auto &data = glyph_data.begin();
	const auto data_range = ro_range{data, data + glyph_data.size()};
	enqueue_glyphs(find_trajectory(traj_id), layer, data_range);
}

node_attribs otv_client::convert_api_node_to_internal (const OTV_HermiteNode &node) {
	node_attribs out;
	out.pos_rad.set(node.position.x, node.position.y, node.position.z);
	out.tangent.set(node.tangent.x, node.tangent.y, node.tangent.z);
	out.t.set(node.time, 0, 0, 0);
	return std::move(out);
}

std::vector<extrapol::node> otv_client::convert_api_extrapol_to_internal (
	const std::vector<OTV_Extrapolation> &extrapol
){
	std::vector<extrapol::node> out;
	out.reserve(extrapol.size());
	for (const auto &e : extrapol)
		out.emplace_back(extrapol::node::from_api_extrapol(e));
	return out;
}

std::vector<float> otv_client::convert_api_glyphs_to_internal (
	unsigned traj_id, unsigned layer, const std::vector<OTV_GlyphData> &glyphs
){
	//const float radius = vis_setup.trajs[traj_id].radius;
	const auto &vis = otv_instance->render.visualizations[0];
	const auto &ds = otv_instance->traj_mgr.dataset(0);
	const float radius = ds.trajectories(ds.positions().attrib)[traj_id].med_radius;
	const auto &lm = vis.manager.ref_glyph_attribute_mappings()[layer];
	const auto &lcfg = vis.config.layer_configs[layer];
	const auto &src_indices = lm.get_attrib_indices();
	std::vector<float> data; data.reserve(glyphs.size()*(glyphs.front().N+2));
	switch (lcfg.shape_ptr->type())
	{
		case GT_COLOR:
			for (const auto &glyph: glyphs)
			{
				data.emplace_back(glyph.s);
				data.emplace_back(.0f); // debug flag, not used when streaming
				constexpr unsigned vattrib_idx__color=1;
				const auto color_src_idx = src_indices[vattrib_idx__color];
				if (color_src_idx >= 0) {
					const auto &gd = *otv__upcast_SurfaceColorData(&glyph);
					data.emplace_back(gd.color);
				}
			}
			return std::move(data);

		case GT_LINE_PLOT:
			for (const auto &glyph: glyphs)
			{
				data.emplace_back(glyph.s);
				data.emplace_back(.0f); // debug flag, not used when streaming
				constexpr unsigned vattrib_idx__subplots=2;
				const auto &gd = *otv__upcast_LinePlotData(&glyph);
				for (unsigned i=0; i<4; i++) {
					const auto vattrib_idx = vattrib_idx__subplots + i+i + 1;
					if (src_indices[vattrib_idx] >= 0)
						data.emplace_back(gd.values[i]);
				}
			}
			return std::move(data);

		case GT_RECTANGLE:
			for (const auto &glyph: glyphs)
			{
				data.emplace_back(glyph.s);
				data.emplace_back(.0f); // debug flag, not used when streaming
				constexpr unsigned vattrib_idx__color=1, vattrib_idx__length=2, vattrib_idx__height=3;
				const auto &gd = *otv__upcast_RectangleData(&glyph);
				if (src_indices[vattrib_idx__color] >= 0)
					data.emplace_back(gd.color);
				if (src_indices[vattrib_idx__length] >= 0)
					data.emplace_back(gd.half_width);
				if (src_indices[vattrib_idx__height] >= 0)
					data.emplace_back(gd.half_height);
			}
			return std::move(data);

		case GT_SIGN_BLOB:
			for (const auto &glyph: glyphs)
			{
				data.emplace_back(glyph.s);
				data.emplace_back(.0f); // debug flag, not used when streaming
				constexpr unsigned vattrib_idx__color=2, vattrib_idx__value=3;
				const auto &gd = *otv__upcast_SignBlobData(&glyph);
				if (src_indices[vattrib_idx__color] >= 0)
					data.emplace_back(gd.color);
				if (src_indices[vattrib_idx__value] >= 0)
					data.emplace_back(gd.value);
			}
			return std::move(data);

		case GT_TRIANGLE:
		case GT_CIRCLE:
		case GT_WEDGE:
		case GT_ARC_FLAT:
		case GT_ARC_ROUNDED:
		case GT_DROP:
		case GT_STAR:
		case GT_TEMPORAL_HEAT_MAP:
			cgv::gui::get_gui_driver()->message(
				"Config contains unimplemented glyph type: "+lcfg.shape_ptr->name()+"\n"
				"OnTubeVis will now most likely crash."
			);
			assert(false && "INTERNAL LOGIC ERROR: Unimplemented glyph type in layer configuration!");

		default:
			/* see below */;
	}

	// Unknown/unsupported
	cgv::gui::get_gui_driver()->message(
		"Config contains unknown glyph type: "+lcfg.shape_ptr->name()+"\n"
		"OnTubeVis will now most likely crash."
	);
	assert(false && "INTERNAL LOGIC ERROR: Unknown glyph type in layer configuration!");
	return {};
}


} // namespace otv
