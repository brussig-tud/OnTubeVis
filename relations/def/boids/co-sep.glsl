// Designed only for r_pre = r_post = 0.

struct Accum {
	vec3 centr;
	uint count;
	vec3 reject;
};
#define RELATION_REDUCE_T Accum

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Accum(vec3(0), 0, vec3(0));
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T accum)
{
	accum.centr += args.sample_point.position;
	accum.count += 1;
	accum.reject -= normalize(args.offset);
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T accum)
{
	if (accum.count == 0) return relation_background_color;

	const vec3 centr = accum.centr / accum.count;
	const vec3 attract = normalize(centr - args.base_point.position);
	const vec3 reject = normalize(accum.reject);

	return relation_to_color(dot(attract, reject));

	return vec3(
		max(dot(attract, args.query_dir), 0),
		//.5 * length(attract + reject),
		0,
		max(dot(reject, args.query_dir), 0)
	);

	const vec3 delta = attract + reject;
	return relation_to_color(dot(delta, delta) < 1e-5 ? 0 : dot(
		normalize(delta),
		normalize(args.base_point.derivative)
	));
}
