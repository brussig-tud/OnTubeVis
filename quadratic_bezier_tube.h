#pragma once

#include <cgv/render/render_types.h>
#include <cgv/math/functions.h>

class quadratic_bezier_tube : public cgv::render::render_types {
private:
	vec3 project_to_plane(vec3 vec, vec3 n) const {
		return vec - n * dot(vec, n) / dot(n, n);
	}

	void control_points_to_poly_coeffs(float p0, float h, float p1, float o_c[3]) const {
		o_c[0] = p0;
		o_c[1] = -2.0f * p0 + 2.0f * h;
		o_c[2] = p0 + p1 - 2.0f * h;
	}

	float eval_poly_d0(float x, float c[3])  const {
		return x * (x * c[2] + c[1]) + c[0];
	}

	mat4 calculate_transformation_matrix()  const {
		vec3 x, y, z;

		float xl, yl;
		bool xq = false;
		bool yq = false;
		{
			x = c.pos - a.pos;
			xl = length(x);

			if(xl < 0.0001f) {
				y = b.pos - a.pos;
				yl = length(y);

				if(yl < 0.0001f) {
					x = vec3(1.0f, 0.0f, 0.0f);
					y = vec3(0.0f, 1.0f, 0.0f);
					z = vec3(0.0f, 0.0f, 1.0f);

					xl = 1.0f; xq = true;
					yl = 1.0f; yq = true;
				} else {
					x = normalize(cgv::math::ortho(x));
					xl = 1.0f; xq = true;

					z = cross(x, y);
				}
			} else {
				y = project_to_plane(b.pos - a.pos, x);
				yl = length(y);

				if(yl < 0.0001f) {
					y = normalize(cgv::math::ortho(x));
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
			float xyl = dot(b.pos - a.pos, xd);

			float cx[3];
			control_points_to_poly_coeffs(0.0f, xyl, xl, cx);

			float cy[3];
			control_points_to_poly_coeffs(0.0f, yl, 0.0f, cy);

			float rc[3];
			control_points_to_poly_coeffs(a.rad, b.rad, c.rad, rc);

			float c_xm[3];
			c_xm[0] = cx[0] - rc[0]; c_xm[1] = cx[1] - rc[1]; c_xm[2] = cx[2] - rc[2];

			float c_xp[3];
			c_xp[0] = cx[0] + rc[0]; c_xp[1] = cx[1] + rc[1]; c_xp[2] = cx[2] + rc[2];

			xm = std::min(-a.rad, std::min(xl - c.rad, eval_poly_d0(cgv::math::saturate(-c_xm[1] / c_xm[2] * 0.5f), c_xm)));
			xp = std::max(+a.rad, std::max(xl + c.rad, eval_poly_d0(cgv::math::saturate(-c_xp[1] / c_xp[2] * 0.5f), c_xp)));

			float c_ym[3];
			c_ym[0] = cy[0] - rc[0]; c_ym[1] = cy[1] - rc[1]; c_ym[2] = cy[2] - rc[2];

			float c_yp[3];
			c_yp[0] = cy[0] + rc[0]; c_yp[1] = cy[1] + rc[1]; c_yp[2] = cy[2] + rc[2];

			ym = std::min(-a.rad, std::min(-c.rad, eval_poly_d0(cgv::math::saturate(-c_ym[1] / c_ym[2] * 0.5f), c_ym)));
			yp = std::max(+a.rad, std::max(+c.rad, eval_poly_d0(cgv::math::saturate(-c_yp[1] / c_yp[2] * 0.5f), c_yp)));

			zm = std::max(a.rad, std::max(c.rad, eval_poly_d0(cgv::math::saturate(-rc[1] / rc[2] * 0.5f), rc)));

			if(xq) { xm = -zm; xp = zm; }
			if(yq) { ym = -zm; yp = zm; }
		}

		vec3 center = a.pos + 0.5f*(xd * (xm + xp) + yd * (ym + yp));

		mat4 M;
		M.set_col(0, vec4((xp - xm) * xd, 0.0f));
		M.set_col(1, vec4((yp - ym) * yd, 0.0f));
		M.set_col(2, vec4(2.0f * zm  * zd, 0.0f));
		M.set_col(3, vec4(center, 1.0f));

		return M;
	}

	// calculates the axis aligned bounding box from the minimal oriented bounding box
	box3 calculate_bounding_box() const {
		box3 box;

		mat4 M = calculate_transformation_matrix();

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

		box.add_point(vec3(p000));
		box.add_point(vec3(p001));
		box.add_point(vec3(p010));
		box.add_point(vec3(p011));
		box.add_point(vec3(p100));
		box.add_point(vec3(p101));
		box.add_point(vec3(p110));
		box.add_point(vec3(p111));
		return box;
	}

	// calculates the exact axis aligned bounding box
	box3 calculate_exact_bounding_box() const {
		const vec3& p0 = a.pos;
		const vec3& p1 = b.pos;
		const vec3& p2 = c.pos;

		float r0 = a.rad;
		float r1 = b.rad;
		float r2 = c.rad;

		vec3 mi = cgv::math::min(p0 - r0, p2 - r2);
		vec3 ma = cgv::math::max(p0 + r0, p2 + r2);

		vec3 p0r = p0 - r0;
		vec3 p1r = p1 - r1;
		vec3 p2r = p2 - r2;

		vec3 t = cgv::math::clamp((p0r - p1r) / (p0r - 2.0f * p1r + p2r), 0.0f, 1.0f);
		vec3 s = vec3(1.0f) - t;
		vec3 q = s * s * p0r + 2.0f * s * t * p1r + t * t * p2r;

		mi = cgv::math::min(mi, q);
		ma = cgv::math::max(ma, q);

		p0r = p0 + r0;
		p1r = p1 + r1;
		p2r = p2 + r2;

		t = cgv::math::clamp((p0r - p1r) / (p0r - 2.0f * p1r + p2r), 0.0f, 1.0f);
		s = vec3(1.0f) - t;
		q = s * s * p0r + 2.0f * s * t * p1r + t * t * p2r;

		mi = cgv::math::min(mi, q);
		ma = cgv::math::max(ma, q);

		return box3(mi, ma);
	}

public:
	struct node {
		vec3 pos;
		float rad;
	};

	node a;
	node b;
	node c;
	
	quadratic_bezier_tube(const node& a, const node& b, const node& c) : a(a), b(b), c(c) {};

	quadratic_bezier_tube(const vec3& pa, const vec3& pb, const vec3& pc, const float ra, const float rb, const float rc) {
		a = { pa, ra };
		b = { pb, rb };
		c = { pc, rc };
	}

	box3 bounding_box(bool exact) const {
		if(exact)
			return calculate_exact_bounding_box();
		else
			return calculate_bounding_box();
	}

	// https://www.shadertoy.com/view/ldj3Wh
	float signed_distance(const vec3& pos) const {
		vec3 a = this->b.pos - this->a.pos;
		vec3 b = this->a.pos - 2.0f*this->b.pos + this->c.pos;
		vec3 c = a * 2.0f;
		vec3 d = this->a.pos - pos;

		float kk = 1.0f / dot(b, b);
		float kx = kk * dot(a, b);
		float ky = kk * (2.0f*dot(a, a) + dot(d, b)) / 3.0f;
		float kz = kk * dot(d, a);

		vec2 res;

		float p = ky - kx * kx;
		float p3 = p * p*p;
		float q = kx * (2.0f*kx*kx - 3.0f*ky) + kz;
		float h = q * q + 4.0f*p3;

		if(h >= 0.0f) {
			h = sqrt(h);
			vec2 x = (vec2(h, -h) - q) / 2.0f;
			vec2 uv = sign(x)*cgv::math::pow(abs(x), vec2(1.0f / 3.0f));
			float t = cgv::math::clamp(uv.x() + uv.y() - kx, 0.0f, 1.0f);

			// 1 root
			res = vec2(cgv::math::sqr_length(d + (c + b * t)*t), t);
		} else {
			float z = sqrt(-p);
			float v = acos(q / (p*z*2.0f)) / 3.0f;
			float m = cos(v);
			float n = sin(v)*1.732050808f;
			vec3 t = cgv::math::clamp(vec3(m + m, -n - m, n - m) * z - kx, 0.0f, 1.0f);

			// 3 roots, but only need two
			float dis = cgv::math::sqr_length(d + (c + b * t.x())*t.x());
			res = vec2(dis, t.x());

			dis = cgv::math::sqr_length(d + (c + b * t.y())*t.y());
			if(dis < res.x()) res = vec2(dis, t.y());
		}

		res.x() = sqrt(res.x());

		float rc[3];
		control_points_to_poly_coeffs(this->a.rad, this->b.rad, this->c.rad, rc);
		float radius = eval_poly_d0(res.y(), rc);

		return res.x() - radius;
	}
};
