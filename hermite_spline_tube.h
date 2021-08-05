#pragma once

#include <cgv/render/render_types.h>
#include <cgv/math/functions.h>

class hermite_spline_tube : public cgv::render::render_types {
private:
	struct TubeNode {
		vec3 pos;
		float rad;

		vec3 pos_tan;
		float rad_tan;
	};

	struct Tube {
		TubeNode s;
		TubeNode e;
	};

	struct QTubeNode {
		vec3 pos;
		float rad;
	};

	struct QTube {
		QTubeNode s;
		QTubeNode h;
		QTubeNode e;
	};

	static float saturate(float x) { return std::min(std::max(x, 0.0f), 1.0f); }

	static vec3 ProjToPlane(vec3 vec, vec3 n) {
		return vec - n * dot(vec, n) / dot(n, n);
	}

	template<typename T>
	void EvalCSplineMidPoint(T p1, T t1, T p2, T t2, T out_p, T out_t) {

		T h1 = p1 + t1 * 1.0f/3.0f;
		T h2 = p2 - t2 * 1.0f/3.0f;

		T a1 = (p1 + h1) * 0.5f;
		T a2 = (h1 + h2) * 0.5f;
		T a3 = (h2 + p2) * 0.5f;

		T b1 = (a1 + a2) * 0.5f;
		T b2 = (a2 + a3) * 0.5f;
	
		out_t = (b2 - b1) * 3.0f;
		out_p = (b1 + b2) * 0.5f;
	}

	template<typename T>
	static void SplitCurve(unsigned segment_idx, T v0, T d0, T v1, T d1, T& v0_out, T& h_out, T& v1_out) {

		v0_out = v0;
		h_out = v0 + d0 * 0.25f;
		T h1 =  v1 - d1 * 0.25f;
		v1_out = (h_out + h1) * 0.5f;

		if(segment_idx == 1) {
			v0_out = v1_out;
			h_out = h1;
			v1_out = v1;
		}
	}

	//#define SPLIT_CURVE(SEGMENT_IDX, TUBE, MEM_V, MEM_D, QTUBE)	\
	//SplitCurve(													\
	//	SEGMENT_IDX,											\
	//	TUBE.s.MEM_V, TUBE.s.MEM_D, TUBE.e.MEM_V, TUBE.e.MEM_D,	\
	//	QTUBE.s.MEM_V, QTUBE.h.MEM_V, QTUBE.e.MEM_V				\
	//)

	static void SplitTube(unsigned segment_id, Tube tube, QTube& qTube) {
		
		SplitCurve(segment_id,
			tube.s.pos, tube.s.pos_tan, tube.e.pos, tube.e.pos_tan,
			qTube.s.pos, qTube.h.pos, qTube.e.pos
		);
		SplitCurve(segment_id,
			tube.s.rad, tube.s.rad_tan, tube.e.rad, tube.e.rad_tan,
			qTube.s.rad, qTube.h.rad, qTube.e.rad
		);

		//SPLIT_CURVE(segment_id, tube.s.pos, pos, pos_tan, qTube);
		//SPLIT_CURVE(segment_id, tube, rad, rad_tan, qTube);
	}

	static void SplinePointsToPolyCoeffs(float p0, float h, float p1, float o_c[3]) {

		o_c[0] = p0;
		o_c[1] = -2.0f * p0 + 2.0f * h;
		o_c[2] =    p0 + p1 - 2.0f * h;
	}

	static vec3 ortho_vec(vec3 v) {

		return abs(v.x()) > abs(v.z()) ? vec3(-v.y(), v.x(), 0.0f) : vec3(0.0f, -v.z(), v.y());
	}

	static float EvalPolyD0(float x, float c[3]) {

		return x * (x * c[2] + c[1]) + c[0];
	}

	static mat4 calculate_transformation_matrix(QTube qTube) {

		vec3 x, y, z;
		float xl, yl;
		bool xq = false;
		bool yq = false;
		{
			x = qTube.e.pos - qTube.s.pos;
			xl = length(x);

			if(xl < 0.0001f) {
				y = qTube.h.pos - qTube.s.pos;
				yl = length(y);

				if(yl < 0.0001f) {
					x = vec3(1.0f, 0.0f, 0.0f);
					y = vec3(0.0f, 1.0f, 0.0f);
					z = vec3(0.0f, 0.0f, 1.0f);

					xl = 1.0f; xq = true;
					yl = 1.0f; yq = true;
				} else {
					x = normalize(ortho_vec(x));
					xl = 1.0f; xq = true;

					z = cross(x, y);
				}
			} else {
				y = ProjToPlane(qTube.h.pos - qTube.s.pos, x);
				yl = length(y);

				if(yl < 0.0001f) {
					y = normalize(ortho_vec(x));
					yl = 1.0f; yq = true;
				}

				z = cross(x, y);
			}
		}

		vec3 xd = x / xl;
		vec3 yd = y / yl;
		vec3 zd = normalize(z);

		float xm, xp, ym, yp, zm;
		{
			float xyl = dot(qTube.h.pos - qTube.s.pos, xd);

			float cx[3];
			SplinePointsToPolyCoeffs(0.0f, xyl, xl, cx);

			float cy[3];
			SplinePointsToPolyCoeffs(0.0f, yl, 0.0f, cy);

			float rc[3];
			SplinePointsToPolyCoeffs(qTube.s.rad, qTube.h.rad, qTube.e.rad, rc);

			float c_xm[3];
			c_xm[0] = cx[0] - rc[0]; c_xm[1] = cx[1] - rc[1]; c_xm[2] = cx[2] - rc[2];

			float c_xp[3];
			c_xp[0] = cx[0] + rc[0]; c_xp[1] = cx[1] + rc[1]; c_xp[2] = cx[2] + rc[2];

			xm = std::min(-qTube.s.rad, std::min(xl - qTube.e.rad, EvalPolyD0(saturate(-c_xm[1] / c_xm[2] * 0.5f), c_xm)));
			xp = std::max(+qTube.s.rad, std::max(xl + qTube.e.rad, EvalPolyD0(saturate(-c_xp[1] / c_xp[2] * 0.5f), c_xp)));

			float c_ym[3];
			c_ym[0] = cy[0] - rc[0]; c_ym[1] = cy[1] - rc[1]; c_ym[2] = cy[2] - rc[2];

			float c_yp[3];
			c_yp[0] = cy[0] + rc[0]; c_yp[1] = cy[1] + rc[1]; c_yp[2] = cy[2] + rc[2];

			ym = std::min(-qTube.s.rad, std::min(-qTube.e.rad, EvalPolyD0(saturate(-c_ym[1] / c_ym[2] * 0.5f), c_ym)));
			yp = std::max(+qTube.s.rad, std::max(+qTube.e.rad, EvalPolyD0(saturate(-c_yp[1] / c_yp[2] * 0.5f), c_yp)));

			zm = std::max(qTube.s.rad, std::max(qTube.e.rad, EvalPolyD0(saturate(-rc[1] / rc[2] * 0.5f), rc)));

			if(xq) { xm = -zm; xp = zm; }
			if(yq) { ym = -zm; yp = zm; }
		}

		vec3 center = qTube.s.pos + 0.5f*(xd * (xm + xp) + yd * (ym + yp));

		mat4 M;
		M.set_col(0, vec4((xp - xm) * xd, 0.0f));
		M.set_col(1, vec4((yp - ym) * yd, 0.0f));
		M.set_col(2, vec4(2.0f * zm  * zd, 0.0f));
		M.set_col(3, vec4(center, 1.0f));

		return M;
	}

	// calculates an axis aligned bounding box from the minimal oriented bounding box of a quadratic bezier tube segment
	static box3 q_spline_bbox(QTube qTube) {

		box3 bbox;

		mat4 M = calculate_transformation_matrix(qTube);

		const vec4 corners[4] = {
			vec4(-0.5, -0.5, -0.5, 1.0),
			vec4(+0.5, -0.5, -0.5, 1.0),
			vec4(-0.5, +0.5, -0.5, 1.0),
			vec4(-0.5, -0.5, +0.5, 1.0)
		};

		vec4 p000 = M * corners[0];
		vec4 p100 = M * corners[1];
		vec4 p010 = M * corners[2];
		vec4 p001 = M * corners[3];

		vec4 dy = p010 - p000;
		vec4 dz = p001 - p000;

		vec4 p110 = p100 + dy;
		vec4 p101 = p100 + dz;
		vec4 p011 = p010 + dz;
		vec4 p111 = p110 + dz;

		bbox.add_point(vec3(p000));
		bbox.add_point(vec3(p001));
		bbox.add_point(vec3(p010));
		bbox.add_point(vec3(p011));
		bbox.add_point(vec3(p100));
		bbox.add_point(vec3(p101));
		bbox.add_point(vec3(p110));
		bbox.add_point(vec3(p111));

		return bbox;
	}

	// calculates the exact axis aligned bounding box for a quadratic bezier tube segment
	static box3 q_spline_exact_bbox(QTube qTube) {

		//box3 bbox;

		vec3 p0 = qTube.s.pos;
		vec3 p1 = qTube.h.pos;
		vec3 p2 = qTube.e.pos;

		float r0 = qTube.s.rad;
		float r1 = qTube.h.rad;
		float r2 = qTube.e.rad;

		vec3 mi = cgv::math::min(p0 - r0, p2 - r2);
		vec3 ma = cgv::math::max(p0 + r0, p2 + r2);

		vec3 p0r = p0 - r0;
		vec3 p1r = p1 - r1;
		vec3 p2r = p2 - r2;

		vec3 t = cgv::math::clamp((p0r - p1r) / (p0r - 2.0f*p1r + p2r), 0.0f, 1.0f);
		vec3 s = vec3(1.0f) - t;
		vec3 q = s * s*p0r + 2.0f*s*t*p1r + t * t*p2r;

		mi = cgv::math::min(mi, q);
		ma = cgv::math::max(ma, q);

		p0r = p0 + r0;
		p1r = p1 + r1;
		p2r = p2 + r2;

		t = cgv::math::clamp((p0r - p1r) / (p0r - 2.0f*p1r + p2r), 0.0f, 1.0f);
		s = vec3(1.0f) - t;
		q = s * s*p0r + 2.0f*s*t*p1r + t * t*p2r;

		mi = cgv::math::min(mi, q);
		ma = cgv::math::max(ma, q);
		
		return box3(mi, ma);
	}

public:
	hermite_spline_tube() = delete;
	~hermite_spline_tube() = delete;

	static box3 calculate_bounding_box(vec3 p0, vec3 p1, vec3 pt0, vec3 pt1, float r0, float r1, float rt0, float rt1, bool exact = false) {

		box3 bbox;
		
		Tube tube;
		tube.s.pos = p0;
		tube.s.rad = r0;
		tube.s.pos_tan = pt0;
		tube.s.rad_tan = rt0;

		tube.e.pos = p1;
		tube.e.rad = r1;
		tube.e.pos_tan = pt1;
		tube.e.rad_tan = rt1;

		QTube qTube0, qTube1;
		SplitTube(0, tube, qTube0);
		SplitTube(1, tube, qTube1);

		if(exact) {
			bbox.add_axis_aligned_box(q_spline_exact_bbox(qTube0));
			bbox.add_axis_aligned_box(q_spline_exact_bbox(qTube1));
		} else {
			bbox.add_axis_aligned_box(q_spline_bbox(qTube0));
			bbox.add_axis_aligned_box(q_spline_bbox(qTube1));
		}

		return bbox;
	}

	// TODO: remove later?
	static mat4 matrix(vec4 s, vec4 h, vec4 e) {

		QTube qTube;
		qTube.s.pos = vec3(s);
		qTube.s.rad = s.w();
		qTube.h.pos = vec3(h);
		qTube.h.rad = h.w();
		qTube.e.pos = vec3(e);
		qTube.e.rad = e.w();

		return calculate_transformation_matrix(qTube);
	}
};
