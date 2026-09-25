#define RELATION_REDUCE_T float

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return 0;
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	reduction += dot(normalize(args.base_point.derivative), normalize(args.sample_point.derivative))
		* (args.duration * weight_all(args));
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (relation_normalize) reduction *= 1e2 / (query_volume() * query_duration());
	return relation_to_color(reduction);
}
