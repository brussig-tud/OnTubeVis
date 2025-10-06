#version 420 core

layout (binding = 6) uniform sampler2D color_maps_tex;


// Map the interval [0, 1] to an RGB color using a precalculated color scale texture.
vec3 map_to_color(float v, int color_map_idx)
{
	vec2 ts = textureSize(color_maps_tex, 0);
	float half_step = 0.5 * (1.0 / ts.y);
	float map_coord = float(color_map_idx) / ts.y + half_step;
	return pow(texture(color_maps_tex, vec2(v, map_coord)).rgb, vec3(2.2));
}
