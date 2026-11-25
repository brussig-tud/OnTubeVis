// Designed only for r_pre = r_post = 0.

struct Accum {
	vec3 centr;
	uint count;
};
#define RELATION_REDUCE_T Accum

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Accum(vec3(0), 0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T accum)
{
	accum.centr += args.sample_point.position;
	accum.count += 1;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T accum)
{
	if (accum.count == 0) return relation_background_color;

	accum.centr /= accum.count;
	const vec3 attract = normalize(accum.centr - args.base_point.position);

	return relation_to_color(dot(attract, args.query_dir));
}
