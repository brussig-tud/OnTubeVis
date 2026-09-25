#define RELATION_REDUCE_T float

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return 0;
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction) {
	reduction += (relation_radius[0] - length(args.offset))
		* (args.duration * weight_space(args) * weight_angle(args));
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (relation_normalize) reduction *= 1e2 / (query_volume() * query_duration());
	return relation_to_color(reduction);
}
