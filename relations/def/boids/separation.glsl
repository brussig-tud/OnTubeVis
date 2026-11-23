// Designed only for r_pre = r_post = 0.

#define RELATION_REDUCE_T vec3

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return vec3(0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T accum)
{
	accum -= normalize(args.offset);
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T accum)
{
	if (accum == 0) return relation_background_color;
	const vec3 reject = normalize(accum);

	return relation_to_color(dot(reject, args.base_normal));
	return relation_to_color(dot(reject, normalize(args.base_point.derivative)));
}
