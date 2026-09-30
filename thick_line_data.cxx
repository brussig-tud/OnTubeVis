#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <string>
#include "thick_line_data.h"
#include <cgv/math/functions.h>
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>
#include "ply.h"

void thick_line_data::clear()
{
	indices .clear();
	spheres .clear();
	tangents.clear();
	colors  .clear();
	bezier_tubes.clear();
}

int thick_line_data::add_node(const cgv::vec3& center, const cgv::vec3& tangent, float radius, float radius_derivative, const cgv::rgb& color)
{
	spheres.push_back(cgv::vec4(center, radius));
	tangents.push_back(cgv::vec4(tangent, radius_derivative));
	colors.push_back(color);
	return (int)spheres.size() - 1;
}
void thick_line_data::add_segment(int start_node_idx, int end_node_idx)
{
	indices.push_back(start_node_idx);
	indices.push_back(end_node_idx);
}

void thick_line_data::generate(int nr_tubes, int nr_segments)
{
	std::uniform_real_distribution<float> dp(-1.0f, 1.0f);
	std::uniform_real_distribution<float> dc(0.2f, 1.0f);
	std::uniform_real_distribution<float> dr(0.01f, 0.05f);
	std::default_random_engine e;
	for (int i = 0; i < nr_tubes; ++i) {
		cgv::vec4 p(dp(e), dp(e), dp(e), dr(e));
		cgv::vec4 t(dp(e), dp(e), dp(e), dr(e));
		cgv::rgb c(dc(e), dc(e), dc(e));
		add_node(p.down(), t.down(), p[3], t[3], c);
		for (int j = 0; j < nr_segments; ++j) {
			cgv::vec4 p1(dp(e), dp(e), dp(e), dr(e));
			cgv::vec4 t1(dp(e), dp(e), dp(e), dr(e));
			cgv::rgb c1(dc(e), dc(e), dc(e));
			add_node(p1.down(), t1.down(), p1[3], t1[3], c1);
			add_segment((int)spheres.size() - 2, (int)spheres.size()-1);
		}
	}
}

void thick_line_data::init_simple()
{
	// indices = { 0,1,1,2 };
	// spheres = { {-2.f,0.f,0.f,.2f}, {0.f,0.f,0.f,.1f}, {0.f,0.f,-2.f,.3f} };
	// tangents = { {0.f,4.f,0.f,-.2f}, {0.f,-4.f,0.f,.2f}, {0.f,4.f,0.f,.4f } };
	// colors = { {1.f,0.f,0.f}, {0.f,1.f,0.f}, {0.f,0.f,1.f} };
	indices = { 0,1 };
	spheres = { {-1.f,0.f,0.f,.4f}, {1.f,0.25f,0.f,.25f} };
	tangents = { {1.f,1.f,0.f,-.4f}, {1.f,3.f,0.f,-.2f} };
	colors = { {1.f,0.f,0.f}, {0.f,1.f,0.f} };
}

void thick_line_data::compute_bezier_tubes() {
	bezier_tubes.clear();
	for (size_t i = 0; i < indices.size(); i += 2) {
		// transform hermit to bezier representation in ray coordinates resulting in Bezier matrix with radius in 4th comp
		cgv::mat4 B;
		uint32_t j = indices[i], k = indices[i + 1];
		B.col(0) = spheres[j];
		B.col(1) = spheres[j] + (1.0f / 3) * tangents[j];
		B.col(2) = spheres[k] - (1.0f / 3) * tangents[k];
		B.col(3) = spheres[k];
		bezier_tubes.push_back(B);
	}
}
cgv::mat4x3 thick_line_data::get_split_tube(size_t i, int part_idx) const {
	const auto& B = bezier_tubes[i];
	cgv::mat4x3 Bl, Br;
	Bl.set_col(0, B.col(0));
	Bl.set_col(1, cgv::math::lerp(B.col(0), B.col(1), magic_factor));
	Br.set_col(2, B.col(3));
	Br.set_col(1, cgv::math::lerp(B.col(3), B.col(2), magic_factor));
	Bl.set_col(2, cgv::math::lerp(Bl.col(1), Br.col(1), 0.5f));
	Br.set_col(0, Bl.col(2));
	return part_idx == 0 ? Bl : Br;
}
cgv::rgb thick_line_data::get_tube_color(float t, size_t tube_idx, int part_idx) const
{
    if (part_idx != -1)
        t = 0.5f * (t + part_idx);
    return lerp(colors[indices[2 * tube_idx]],colors[indices[2 * tube_idx+1]],t);
}

thick_line_data::thick_line_data() 
{
    //select_scene(scene_type::simple);
	//generate_torus_knot(10, 7, 1.0f, 0.1f, 2.0f, cgv::vec3(0.0f), 50, 0.01f, 5, 2, 1, cgv::rgb(0.7f, 0.2f, 0.5f));
	//generate_woven_bundle(
	//	10, 8, 1.0f, 0.5f,
	//	5, cgv::vec3(0.0f), 2.0f, 50,
	//	0.02f, cgv::rgb(1.0, 0.0, 0.0), cgv::rgb(0.0, 0.0, 1.0));
    //compute_bezier_tubes();
}

void thick_line_data::select_scene(scene_type scene)
{
    switch (scene) {
    case scene_type::simple: clear(); init_simple(); break;
    case scene_type::random: clear(); generate(3,30); break;
    case scene_type::procedural: clear(); generate(3, 30); generate_trajectories(548975); break;
    }
    compute_bezier_tubes();
}

struct BCCHeader
{
    char sign[3];
    unsigned char byteCount;
    char curveType[2];
    char dimensions;
    char upDimension;
    uint64_t curveCount;
    uint64_t totalControlPointCount;
    char fileInfo[40];
};

struct PlyVertex
{
    float x, y, z;
    unsigned char red, green, blue, alpha;
};
static PlyProperty vert_props[] = { /* list of property information for a vertex */
  {(char*)"x", Float32, Float32, offsetof(PlyVertex,x), 0, 0, 0, 0},
  {(char*)"y", Float32, Float32, offsetof(PlyVertex,y), 0, 0, 0, 0},
  {(char*)"z", Float32, Float32, offsetof(PlyVertex,z), 0, 0, 0, 0},
  {(char*)"red", Uint8, Uint8, offsetof(PlyVertex,red), 0, 0, 0, 0},
  {(char*)"green", Uint8, Uint8, offsetof(PlyVertex,green), 0, 0, 0, 0},
  {(char*)"blue", Uint8, Uint8, offsetof(PlyVertex,blue), 0, 0, 0, 0},
  {(char*)"alpha", Uint8, Uint8, offsetof(PlyVertex,alpha), 0, 0, 0, 0},
};

static char* propNames[] = { (char*)"vertex" };


/// read from file with format detected from extension. Currently supported: bcc
bool thick_line_data::read(const std::string &file_path, float default_radius, float tangent_scale)
{
    std::string ext = cgv::utils::to_lower(cgv::utils::file::get_extension(file_path));
    if (ext == "bcc") {
        BCCHeader header;
        FILE* pFile = fopen(file_path.c_str(), "rb");
        fread(&header, sizeof(header), 1, pFile);
        if(header.sign[0] != 'B') return false; // Invalid file signature
        if (header.sign[1] != 'C') return false; // Invalid file signature
        if (header.sign[2] != 'C') return false; // Invalid file signature
        if (header.byteCount != 0x44) return false; // Only supporting 4-byte integers and floats
        if (header.curveType[0] != 'C') return false; // Not a Catmull-Rom curve
        if (header.curveType[1] != '0') return false; // Not uniform parameterization
        if (header.dimensions != 3) return false; // Only curves in 3D
        std::vector<cgv::vec3> controlPoints(header.totalControlPointCount);
        std::vector<int> firstControlPoint(header.curveCount + 1);
        std::vector<char> isCurveLoop(header.curveCount);
        float* cp = &controlPoints.data()[0][0];
        int prevCP = 0;
        for (size_t i = 0; i < header.curveCount; i++) {
            int curveControlPointCount;
            fread(&curveControlPointCount, sizeof(int), 1, pFile);
            isCurveLoop[i] = curveControlPointCount < 0;
            if (curveControlPointCount < 0) curveControlPointCount = -curveControlPointCount;
            fread(cp, sizeof(cgv::vec3), curveControlPointCount, pFile);
            cp += 3*curveControlPointCount;
            firstControlPoint[i] = prevCP;
            prevCP += curveControlPointCount;
        }
        firstControlPoint[header.curveCount] = prevCP;
        std::cout << "curveCount=" << header.curveCount << "; control point count=" << controlPoints.size() << std::endl;

        // convert to hermite spline
        std::default_random_engine e;
        std::uniform_real_distribution<float> d(0.0f, 1.0f);
        spheres.resize(controlPoints.size());
        for (size_t i = 0; i < controlPoints.size(); ++i)
            spheres[i] = cgv::vec4(controlPoints[i], default_radius);
        tangents.clear();
        tangents.resize(controlPoints.size(), cgv::vec4(0.0f));
        colors.resize(controlPoints.size(), cgv::rgb(0.5f, 0.5f, 0.5f));
        // compute tangents
        tangent_scale *= 0.5f;
        for (size_t i = 0; i < header.curveCount; i++) {
            std::cout << "curve[" << i << "] from " << firstControlPoint[i] << " ... " << firstControlPoint[i + 1];
            if (isCurveLoop[i])
                std::cout << " loop" << std::endl;

            cgv::rgb c(d(e), d(e), d(e));
            for (int j = firstControlPoint[i]; j < firstControlPoint[i + 1]; ++j) {
                int jp = -1;
                if (j == firstControlPoint[i]) {
                    if (isCurveLoop[i])
                        jp = firstControlPoint[i + 1] - 1;
                }
                else 
                    jp = j - 1;
                if (jp != -1)
                    tangents[jp] += tangent_scale * cgv::vec4(controlPoints[j], 0.0f);
                size_t jn = -1;
                if (j == firstControlPoint[i + 1] - 1) {
                    if (isCurveLoop[i])
                        jn = firstControlPoint[i];
                }
                else 
                    jn = j + 1;
                if (jn != -1)
                    tangents[jn] -= tangent_scale * cgv::vec4(controlPoints[j], 0.0f);
                if (jp != -1) {
                    indices.push_back(jp);
                    indices.push_back((int)j);
                }
            }
            if (isCurveLoop[i] == 0) {
                tangents[firstControlPoint[i]] *= 2.0f;
                tangents[firstControlPoint[i]] -= cgv::vec4(controlPoints[firstControlPoint[i]], 0.0f);
                tangents[firstControlPoint[i + 1] - 1] *= 2.0f;
                tangents[firstControlPoint[i + 1] - 1] += cgv::vec4(controlPoints[firstControlPoint[i+1]-1], 0.0f);
            }
            std::fill(colors.begin() + firstControlPoint[i], colors.begin() + firstControlPoint[i + 1], c);
        }
        compute_bezier_tubes();
    }
    else if (ext == "ply") {
        std::ifstream contents(file_path, std::ios::binary);
        return contents && read_ply(contents, default_radius, tangent_scale);
    }
    return true;
}

bool thick_line_data::read_ply (std::istream &contents, float default_radius, float tangent_scale)
{
    if (!contents)
        return false;

    // The PLY parser borrows this buffer until all elements have been read.
    std::string bytes;
    try {
        bytes.assign(std::istreambuf_iterator<char>(contents), std::istreambuf_iterator<char>());
    }
    catch (const std::ios_base::failure&) {
        return false;
    }
    if (contents.bad() || bytes.empty())
        return false;

    const auto close_ply_file = [](PlyFile* ply) {
        if (ply) {
            close_ply(ply);
            free_ply(ply);
        }
    };
    std::unique_ptr<PlyFile, decltype(close_ply_file)> ply_in(
        read_ply_from_memory(bytes.data(), bytes.size()), close_ply_file);
    if (!ply_in)
        return false;

    thick_line_data loaded;
    bool found_vertex = false;
    for (int element_type = 0; element_type < ply_in->num_elem_types; ++element_type) {
        int element_count = 0;
        const char* element_name = setup_element_read_ply(ply_in.get(), element_type, &element_count);
        if (!element_name || element_count < 0)
            return false;

        if (std::strcmp("vertex", element_name) != 0) {
            // The PLY reader is sequential, so consume preceding element types.
            for (int i = 0; i < element_count; ++i) {
                PlyVertex ignored{};
                get_element_ply(ply_in.get(), &ignored);
                if (ply_in->read_error)
                    return false;
            }
            continue;
        }
        if (found_vertex)
            return false;
        found_vertex = true;

        const PlyElement* element = ply_in->elems[element_type];
        bool has_position[3] = { false, false, false };
        bool has_color[4] = { false, false, false, false };
        for (int i = 0; i < element->nprops; ++i) {
            const PlyProperty* property = element->props[i];
            const char* name = property->name;
            bool* found = nullptr;
            if (std::strcmp("x", name) == 0) found = &has_position[0];
            if (std::strcmp("y", name) == 0) found = &has_position[1];
            if (std::strcmp("z", name) == 0) found = &has_position[2];
            if (std::strcmp("red", name) == 0) found = &has_color[0];
            if (std::strcmp("green", name) == 0) found = &has_color[1];
            if (std::strcmp("blue", name) == 0) found = &has_color[2];
            if (std::strcmp("alpha", name) == 0) found = &has_color[3];
            if (found) {
                if (property->is_list != PLY_SCALAR || *found)
                    return false;
                *found = true;
            }
        }
        if (!(has_position[0] && has_position[1] && has_position[2])) {
            std::cerr << "PLY stream has no complete position property!" << std::endl;
            return false;
        }

        int stored_count = element_count;
#ifdef _DEBUG
        stored_count = std::min(stored_count, 300);
#endif
        std::cout << "open ply with " << stored_count << " nodes" << std::endl;
        loaded.spheres.resize(stored_count);
        loaded.colors.assign(stored_count, cgv::rgb(0.5f, 0.5f, 0.5f));
        for (int i = 0; i < 3; ++i)
            setup_property_ply(ply_in.get(), &vert_props[i]);
        for (int i = 0; i < 4; ++i)
            if (has_color[i])
                setup_property_ply(ply_in.get(), &vert_props[3 + i]);

        const bool has_rgb = has_color[0] && has_color[1] && has_color[2];
        const float color_scale = 1.0f / 255.0f;
        for (int i = 0; i < element_count; ++i) {
            PlyVertex vertex{};
            get_element_ply(ply_in.get(), &vertex);
            if (ply_in->read_error)
                return false;
            if (i >= stored_count)
                continue;
            loaded.spheres[i] = cgv::vec4(vertex.x, vertex.y, vertex.z, default_radius);
            if (has_rgb)
                loaded.colors[i] = cgv::rgb(
                    color_scale * vertex.red, color_scale * vertex.green, color_scale * vertex.blue);
        }
    }
    if (!found_vertex)
        return false;

    // Construct the same 100-node hair strands as the previous file loader.
    const int strand_length = 100;
    const int nr_strands = static_cast<int>(loaded.spheres.size() / strand_length);
    loaded.tangents.resize(loaded.spheres.size(), cgv::vec4(0.0f));
    tangent_scale *= 0.5f;
    for (int i = 0; i < nr_strands; ++i) {
        for (int j = 0; j < strand_length; ++j) {
            const int k = strand_length * i + j;
            const float v = 1.0f - j * 0.01f;
            loaded.spheres[k][3] = std::sqrt(v) * default_radius;
        }
    }
    for (int i = 0; i < nr_strands; ++i) {
        for (int j = 1; j < strand_length; ++j) {
            const int k = strand_length * i + j;
            loaded.indices.push_back(k - 1);
            loaded.indices.push_back(k);
            loaded.tangents[k - 1] += tangent_scale * loaded.spheres[k];
            if (j < strand_length - 1)
                loaded.tangents[k + 1] -= tangent_scale * loaded.spheres[k];
        }
        const int k0 = strand_length * i;
        const int k1 = strand_length * (i + 1) - 1;
        loaded.tangents[k0 + 1] -= tangent_scale * loaded.spheres[k0];
        loaded.tangents[k0] -= tangent_scale * loaded.spheres[k0];
        loaded.tangents[k0] *= 2.0f;
        loaded.tangents[k1] += tangent_scale * loaded.spheres[k1];
        loaded.tangents[k1] *= 2.0f;
    }
    loaded.compute_bezier_tubes();

    indices.swap(loaded.indices);
    spheres.swap(loaded.spheres);
    tangents.swap(loaded.tangents);
    colors.swap(loaded.colors);
    bezier_tubes.swap(loaded.bezier_tubes);
    return true;
}

int thick_line_data::generate_torus_knot(
    int p, int q, float ring_radius, float tube_offset, float z_scale,
    const cgv::vec3& center, int key_count, float tube_radius,
    int bead_freq, float bead_amp, float bead_phase,
    const cgv::rgb& color_a, const cgv::rgb& color_b)
{
    if (key_count < 8 || p < 1 || q < 1)
        return -1;

    const float dphi = 2.0f * kPi / (float)key_count;   // phi-span of one segment

    std::vector<int> idx(key_count);
    for (int i = 0; i < key_count; ++i)
    {
        const float phi = (float)i * dphi;
        const float cq = std::cos((float)q * phi), sq = std::sin((float)q * phi);
        const float cp = std::cos((float)p * phi), sp = std::sin((float)p * phi);
        const float w = ring_radius + tube_offset * cq;

        const cgv::vec3 P = center + cgv::vec3(w * cp,
            z_scale * tube_offset * sq,
            w * sp);
        const float dw = -tube_offset * (float)q * sq;
        const cgv::vec3 dP(dw * cp - w * (float)p * sp,
            z_scale * tube_offset * (float)q * cq,
            dw * sp + w * (float)p * cp);

        const float A = 1.0f + bead_amp * std::sin((float)bead_freq * phi + bead_phase);
        const float B = 1.0f + 0.18f * std::sin(2.0f * (float)bead_freq * phi - 1.1f);
        const float dA = bead_amp * (float)bead_freq * std::cos((float)bead_freq * phi + bead_phase);
        const float dB = 0.18f * 2.0f * (float)bead_freq * std::cos(2.0f * (float)bead_freq * phi - 1.1f);
        const float r = tube_radius * A * B;
        const float dr = tube_radius * (dA * B + A * dB);

        // periodic color pulse, slightly offset from the bead peaks;
        // integer bead_freq over phi in [0, 2pi] -> seamless wrap
        const float cw = 0.5f + 0.5f * std::sin((float)bead_freq * phi + bead_phase + 1.7f);

        idx[i] = add_node(P, dP * dphi, r, dr * dphi, lerp(color_a, color_b, cw));
    }
    for (int i = 0; i < key_count; ++i)                 // close the loop
        add_segment(idx[i], idx[(i + 1) % key_count]);
    return idx[0];
}
// central differences for tangents / radius derivatives (uniform spacing)
float central_difference(const std::vector<float>& v, int i)
{
    const int n = (int)v.size();
    if (n < 2)  return 0.0f;
    if (i == 0)     return v[1] - v[0];
    if (i == n - 1) return v[n - 1] - v[n - 2];
    return 0.5f * (v[i + 1] - v[i - 1]);
}
cgv::vec3 central_difference3(const std::vector<cgv::vec3>& v, int i)
{
    const int n = (int)v.size();
    if (n < 2)  return cgv::vec3(0.0f, 0.0f, 0.0f);
    if (i == 0)     return v[1] - v[0];
    if (i == n - 1) return v[n - 1] - v[n - 2];
    return (v[i + 1] - v[i - 1]) * 0.5f;
}
// ----- arc-length resampling on double precision points (for Lorenz) -----
static double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
static std::vector<std::array<double, 3>> resample_arc_length(
    const std::vector<std::array<double, 3>>& pts, int count)
{
    std::vector<std::array<double, 3>> out;
    if (pts.size() < 2 || count < 2) return out;
    std::vector<double> acc(1, 0.0);
    acc.reserve(pts.size());
    for (size_t i = 1; i < pts.size(); ++i)
        acc.push_back(acc.back() + dist3(pts[i], pts[i - 1]));
    const double total = acc.back();
    if (total <= 0.0) return out;
    out.reserve((size_t)count);
    size_t seg = 0;
    for (int k = 0; k < count; ++k)
    {
        const double target = total * (double)k / (double)(count - 1);
        while (seg + 1 < acc.size() && acc[seg + 1] < target) ++seg;
        const double denom = acc[seg + 1] - acc[seg];
        const double t = denom > 1e-12 ? (target - acc[seg]) / denom : 0.0;
        out.push_back({ pts[seg][0] + (pts[seg + 1][0] - pts[seg][0]) * t,
                        pts[seg][1] + (pts[seg + 1][1] - pts[seg][1]) * t,
                        pts[seg][2] + (pts[seg + 1][2] - pts[seg][2]) * t });
    }
    return out;
}
void thick_line_data::generate_woven_bundle(
    int outer_count, int inner_count, float outer_radius, float inner_radius,
    float turns, const cgv::vec3& base, float length, int key_count,
    float strand_radius,
    const cgv::rgb& outer_color_a, const cgv::rgb& outer_color_b,
    const cgv::rgb& inner_color_a, const cgv::rgb& inner_color_b)
{
    if (key_count < 8 || (outer_count < 1 && inner_count < 1))
        return;

    auto emit_strand = [&](float coil_radius, float dir_sign, float phase0,
        const cgv::rgb& col_a, const cgv::rgb& col_b)
    {
        const float ds = 1.0f / (float)(key_count - 1);   // s-span of one segment

        std::vector<float>    rad(key_count);
        std::vector<cgv::rgb> col(key_count);
        for (int i = 0; i < key_count; ++i)
        {
            const float s = (float)i * ds;
            const float taper = std::sqrt(std::max(std::sin(kPi * s), 0.0f));
            rad[i] = strand_radius * (0.30f + 0.70f * taper);   // rounded ends

            // base->tip gradient with a wave whose phase follows the coil
            // phase -> the strands in a group are visibly distinct
            const float w = cgv::math::clamp(s + 0.15f * std::sin(2.0f * kPi * 3.0f * s + phase0),0.0f,1.0f);
            col[i] = lerp(col_a, col_b, w);
        }

        std::vector<int> idx(key_count);
        for (int i = 0; i < key_count; ++i)
        {
            const float s = (float)i * ds;

            const float fx = 2.0f * kPi * 1.5f * s + 0.7f;
            const float fz = 2.0f * kPi * 2.3f * s + 2.1f;
            const cgv::vec3 spine = base + cgv::vec3(0.16f * std::sin(fx),
                length * s,
                0.16f * std::sin(fz));
            const cgv::vec3 dspine = cgv::vec3(0.16f * 2.0f * kPi * 1.5f * std::cos(fx),
                length,
                0.16f * 2.0f * kPi * 2.3f * std::cos(fz));

            const float th = dir_sign * turns * 2.0f * kPi * s + phase0;
            const float dth = dir_sign * turns * 2.0f * kPi;
            const float ga = 2.0f * kPi * 3.0f * s + 2.0f * phase0;
            const float rc = coil_radius * (1.0f + 0.10f * std::sin(ga));
            const float drc = coil_radius * 0.10f * 2.0f * kPi * 3.0f * std::cos(ga);
            const float cth = std::cos(th), sth = std::sin(th);

            const cgv::vec3 P = spine + cgv::vec3(rc * cth, 0.0f, rc * sth);
            const cgv::vec3 dP = (dspine + cgv::vec3(drc * cth - rc * sth * dth,
                0.0f,
                drc * sth + rc * cth * dth)) * ds;

            idx[i] = add_node(P, dP, rad[i], central_difference(rad, i), col[i]);
        }
        for (int i = 0; i + 1 < key_count; ++i)
            add_segment(idx[i], idx[i + 1]);
    };

    for (int i = 0; i < outer_count; ++i)
        emit_strand(outer_radius, +1.0f,
            2.0f * kPi * (float)i / (float)outer_count,
            outer_color_a, outer_color_b);
    for (int i = 0; i < inner_count; ++i)
        emit_strand(inner_radius, -1.0f,
            2.0f * kPi * (float)i / (float)inner_count + 0.8f,
            inner_color_a, inner_color_b);
}
int thick_line_data::append_open_curve(const std::vector<cgv::vec3>& P,
    const std::vector<float>& radii,
    const std::vector<cgv::rgb>& color)
{
    const int n = (int)P.size();
    if (n < 2) return -1;
    std::vector<int> idx(n);
    for (int i = 0; i < n; ++i)
    {
        idx[i] = add_node(P[i], central_difference3(P, i),
            radii[i], central_difference(radii, i), color[i]);
    }
    for (int i = 0; i + 1 < n; ++i)
        add_segment(idx[i], idx[i + 1]);
    return idx[0];
}
int thick_line_data::generate_lorenz_trail(
    uint64_t seed, float target_size, int keys, float base_radius,
    const cgv::vec3& center, const cgv::rgb& head_color, const cgv::rgb& tail_color)
{
    if (keys < 8)
        return -1;

    const double sigma = 10.0, rho = 28.0, beta = 8.0 / 3.0, dt = 0.004;
    auto f = [&](const std::array<double, 3>& v)->std::array<double, 3>
    {
        return { sigma * (v[1] - v[0]),
                 v[0] * (rho - v[2]) - v[1],
                 v[0] * v[1] - beta * v[2] };
    };
    auto rk4 = [&](std::array<double, 3>& x)
    {
        std::array<double, 3> a1 = f(x), a2, a3, a4, xt;
        for (int k = 0; k < 3; ++k) xt[k] = x[k] + 0.5 * dt * a1[k];
        a2 = f(xt);
        for (int k = 0; k < 3; ++k) xt[k] = x[k] + 0.5 * dt * a2[k];
        a3 = f(xt);
        for (int k = 0; k < 3; ++k) xt[k] = x[k] + dt * a3[k];
        a4 = f(xt);
        for (int k = 0; k < 3; ++k)
            x[k] += (a1[k] + 2.0 * a2[k] + 2.0 * a3[k] + a4[k]) * (dt / 6.0);
    };

    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> U(-10.0, 10.0);
    std::array<double, 3> x = { U(rng), U(rng), 20.0 + 0.5 * U(rng) };
    for (int i = 0; i < 4000; ++i) rk4(x);              // fall onto the attractor

    std::vector<std::array<double, 3>> raw;
    raw.reserve((size_t)keys * 2);
    for (int k = 0; k < keys * 2; ++k)
    {
        for (int j = 0; j < 32; ++j) rk4(x);
        raw.push_back(x);
    }
    const std::vector<std::array<double, 3>> rs = resample_arc_length(raw, keys);
    if (rs.empty())
        return -1;
    const int n = (int)rs.size();

    std::array<double, 3> lo = rs[0], hi = rs[0];
    for (const auto& pt : rs)
        for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], pt[k]); hi[k] = std::max(hi[k], pt[k]); }
    const double max_dim = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    const double scale = (double)target_size / std::max(max_dim, 1e-9);
    const std::array<double, 3> mid = { (lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5 };

    std::vector<cgv::vec3> P(n);
    std::vector<float>     rad(n);
    std::vector<cgv::rgb>  col(n);
    for (int i = 0; i < n; ++i)
    {
        const float t = (float)i / (float)(n - 1);      // 0 = head (thick), 1 = tail
        P[i] = center + cgv::vec3((float)((rs[i][0] - mid[0]) * scale),
            (float)((rs[i][1] - mid[1]) * scale),
            (float)((rs[i][2] - mid[2]) * scale));
        rad[i] = base_radius * (0.15f + 0.85f * std::pow(1.0f - t, 1.5f));  // comet
        col[i] = lerp(head_color, tail_color, t);   // bright head, fading tail
    }
    return append_open_curve(P, rad, col);
}
int thick_line_data::generate_tendril(
    std::mt19937_64& rng, const cgv::vec3& base_pos, cgv::vec3 dir,
    int keys, float step, float curl, float base_radius, float bead_phase,
    const cgv::rgb& root_color, const cgv::rgb& tip_color)
{
    if (keys < 8)
        return -1;

    std::uniform_real_distribution<float> J(-1.0f, 1.0f);
    dir.normalize();

    std::vector<cgv::vec3> P(keys);
    std::vector<float>     rad(keys);
    std::vector<cgv::rgb>  col(keys);
    cgv::vec3 pos = base_pos;
    for (int i = 0; i < keys; ++i)
    {
        const float t = (float)i / (float)(keys - 1);
        P[i] = pos;
        rad[i] = base_radius * std::pow(1.0f - t, 0.75f)
            * (1.0f + 0.30f * std::sin(5.0f * kPi * t + bead_phase));
        // gradient whose banding rides on the same wave as the radius knobs
        const float w = cgv::math::clamp(t + 0.12f * std::sin(5.0f * kPi * t + bead_phase), 0.0f,1.0f);
        col[i] = lerp(root_color, tip_color, w);

        const cgv::vec3 jitter(J(rng), 0.35f + 0.5f * J(rng), J(rng));
        dir += jitter * curl;
        dir.normalize();
        pos = pos + dir * (step * (1.0f - 0.55f * t));
    }
    return append_open_curve(P, rad, col);
}

void thick_line_data::generate_trajectories(uint64_t seed)
{
    // 1 - three torus knots, each pulsing between two hues
    generate_torus_knot(2, 3, 1.05f, 0.40f, 1.30f, cgv::vec3(-2.6f, 1.0f, 0.2f),
        72, 0.070f, 9, 0.55f, 0.0f,
        cgv::rgb(0.88f, 0.28f, 0.16f), cgv::rgb(0.99f, 0.75f, 0.30f));
    generate_torus_knot(2, 5, 0.92f, 0.32f, 1.05f, cgv::vec3(2.6f, 0.9f, 0.6f),
        100, 0.052f, 11, 0.50f, 1.3f,
        cgv::rgb(0.95f, 0.63f, 0.18f), cgv::rgb(0.98f, 0.95f, 0.85f));
    generate_torus_knot(3, 4, 0.78f, 0.30f, 1.00f, cgv::vec3(0.1f, -0.8f, -2.9f),
        100, 0.048f, 8, 0.45f, 2.4f,
        cgv::rgb(0.58f, 0.34f, 0.72f), cgv::rgb(0.30f, 0.75f, 0.80f));

    // 2 - woven bundle: blue outer strands, teal inner strands
    generate_woven_bundle(3, 2, 0.46f, 0.20f, 7.0f, cgv::vec3(0.0f, -2.0f, 2.6f),
        4.6f, 96, 0.085f,
        cgv::rgb(0.10f, 0.40f, 0.80f), cgv::rgb(0.55f, 0.85f, 1.00f),
        cgv::rgb(0.12f, 0.55f, 0.50f), cgv::rgb(0.55f, 0.95f, 0.85f));

    // 3 - Lorenz comet: warm bright head, cool fading tail
    generate_lorenz_trail(seed ^ 0x9E3779B97F4A7C15ull, 2.4f, 84, 0.035f,
        cgv::vec3(0.0f, 2.4f, -0.4f),
        cgv::rgb(1.00f, 0.92f, 0.75f), cgv::rgb(0.45f, 0.35f, 0.55f));

    // 4 - six tendrils from three root->tip color pairs
    std::mt19937_64 rng(seed);
    const cgv::rgb roots[3] = { cgv::rgb(0.35f, 0.68f, 0.30f),
                                cgv::rgb(0.58f, 0.34f, 0.72f),
                                cgv::rgb(0.92f, 0.45f, 0.62f) };
    const cgv::rgb tips[3] = { cgv::rgb(0.85f, 0.95f, 0.45f),
                                cgv::rgb(0.85f, 0.65f, 1.00f),
                                cgv::rgb(1.00f, 0.85f, 0.35f) };
    for (int i = 0; i < 6; ++i)
    {
        const float a = 2.0f * kPi * (float)i / 6.0f + 0.3f;
        const cgv::vec3 base_pos(0.55f * std::cos(a), -2.6f, 0.8f + 0.55f * std::sin(a));
        cgv::vec3 dir = cgv::vec3(0.8f * std::cos(a), 1.0f, 0.8f * std::sin(a)); dir.normalize();
        generate_tendril(rng, base_pos, dir, 46, 0.16f, 0.45f, 0.075f,
            2.0f * kPi * (float)i / 3.0f, roots[i % 3], tips[i % 3]);
    }
}
