
// local includes
#include "arclen_helper.h"
#include "render/extrapolation.h"


namespace otv
{
namespace extrapol
{

void compute_path (
	std::vector<extrapol::node> &out, const unsigned num, const node_attribs &ref_node0,
	const node_attribs &ref_node1, const cgv::mat4 &ref_t_to_s
){
	// Compute extrapolation parameters
	const float r = ref_node0.pos_rad.w();
	const auto p1 = cgv::vec3(ref_node1.pos_rad);
	const auto m1 = cgv::vec3(ref_node1.tangent);
	const auto m1_len = m1.length();
	const float t1 = ref_node1.t.x();
	const float dt = t1 - ref_node0.t.x();

	// Extrapolate
	extrapol::node prev_extrapol = {
		ref_node1, ref_t_to_s
	};
	for (unsigned i=0; i<num; i++)
	{
		const node_attribs new_node {
			cgv::vec4(p1 + float(i)*m1, r), ref_node1.color, cgv::vec4(m1, 0),
			cgv::vec4(t1 + float(i)*dt, 0, 0, 0)
		};
		const extrapol::node new_extrapol {
			new_node, arclen::single_linear_t_to_s(m1_len, /*sigma: */ prev_extrapol.t_to_s[15])
		};
		out.emplace_back(new_extrapol);
		prev_extrapol = new_extrapol;
	}
}

} // namespace extrapol


bool extrapolation_manager::create_geom_buffers (unsigned num_trajectories, unsigned num_segments)
{
	// Make sure we don't leak resources when any single one of the steps fails
	auto _ = finalizer(std::bind(&extrapolation_manager::clear, this));

	// Clean up previous contents, if any
	_.f(); // re-use finalizer to perform cleanup of GPU memory arenas

	// Create GPU ring buffer arenas
	const unsigned num_nodes = num_segments+1;
	const bool success
	         = geom.nodes_arena.create(num_trajectories, num_nodes)
	        && geom.t_to_s_arena.create(num_trajectories, num_segments);

	// Distribute per-trajectory ringbuffers and create indices in the process
	trajectories.reserve(num_trajectories);
	geom.node_indices.reserve(num_trajectories * num_segments);
	for (unsigned t=0; t<num_trajectories; t++) {
		trajectories.emplace_back(geom.nodes_arena.buffer(t), geom.t_to_s_arena.buffer(t));
		uint32_t start_index = t*num_nodes;
		for (unsigned i=0; i<num_segments; i++)
			geom.node_indices.emplace_back(start_index+i, start_index+i+1);
	}

	// Done!
	return _.disarm(success);
}

bool extrapolation_manager::create_glyph_and_per_layer_buffers (
	unsigned num_layers, unsigned max_attribs_per_glyph, unsigned num_glyphs_capacity
)
{
	// Sanity checks
	assert(
		!geom.nodes_arena.ring_buffers.empty() &&
		"extrapolation_manager: create_glyph_and_per_layer_buffers() needs pre-created geometry buffers"
	);
	assert(
		geom.ranges_arena.ring_buffers.empty() &&
		"extrapolation_manager: create_glyph_and_per_layer_buffers() must not be called on a fully constructed setup"
	);

	// Make sure we don't leak resources when any single one of the steps fails
	auto _ = finalizer([this] {
		geom.ranges_arena.destroy();
	});

	// Create GPU ring buffer arenas
	// - glyph index mapping ranges
	const auto num_trajectories = geom.t_to_s_arena.num_ring_buffers();
	const auto num_segments = geom.t_to_s_arena.ring_buffer_capacity();
	const auto num_buffers = num_trajectories*num_layers;
	bool success = geom.ranges_arena.create(num_buffers, num_segments);
	// - glyph instances
	const auto num_attribs = max_attribs_per_glyph*num_glyphs_capacity;
	success &= glyphs.glyph_attribs_arena.create(num_buffers, num_attribs);

	// Assign range ring buffers from arena to corresponding layer on each trajectory
	for (unsigned t=0; t<num_trajectories; t++) {
		auto &traj = trajectories[t];
		traj.layers.reserve(num_layers);
		for (unsigned l=0; l<num_layers; l++) {
			const unsigned per_layer_buffer_id = t*num_layers + l;
			traj.layers.emplace_back(
				geom.ranges_arena.buffer(per_layer_buffer_id),
				glyphs.glyph_attribs_arena.buffer(per_layer_buffer_id)
			);
		}
	}

	// Done!
	return _.disarm(success);
}

void extrapolation_manager::replace_extrapolation (unsigned traj_id, const std::vector<extrapol::node> &extrapolation)
{
	#define DEBUG_OUTPUT 0

	// Retrieve trajectory
	auto &traj = trajectories[traj_id];

	// Replace buffer contents
	#if DEBUG_OUTPUT
		std::clog << "### UPDATING EXTRAPOLATED NODES ##############################\n";
	#endif
	const auto num = std::min<size_t>(traj.nodes.capacity(), extrapolation.size());
	for (unsigned i=0; i<num; i++) {
		traj.nodes.contents[i] = extrapolation[i].hnode;
		traj.t_to_s.contents[i] = extrapolation[i].t_to_s;
	}
	// Flush nodes buffer
	//const auto flush_result = traj.nodes.contents_mem.flush();
	#if DEBUG_OUTPUT
		for (unsigned i=0; i<num; i++) {
			std::clog << "Elem["<<i<<"]:\n"
			          << " .pos_rad = "<<traj.nodes.contents[i].pos_rad<<"\n"
			          << " .color = "<<traj.nodes.contents[i].color<<"\n"
			          << " .tangent = "<<traj.nodes.contents[i].tangent<<"\n"
			          << " .t = "<<traj.nodes.contents[i].t<<"\n";
		}
		//std::clog << "flush_result: "<<flush_result << std::endl;
		//std::clog.flush();
	#endif
}

bool extrapolation_manager::flush_changes(void) const {
	bool result = geom.nodes_arena.flush_all();
	result &= geom.t_to_s_arena.flush_all();
	result &= geom.ranges_arena.flush_all();
	result &= glyphs.glyph_attribs_arena.flush_all();
	return result;
}



} // namespace otv
