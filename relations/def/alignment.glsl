#define RELATION_REDUCE_T float

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return 0;
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	reduction +=
		  dot(normalize(args.base_point.derivative), normalize(args.sample_point.derivative))
		* exp(dot(args.offset, args.offset) * (-5 / sqr(relation_radius[0])))
		* (args.time_weight * args.angle_weight);
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (relation_normalize) reduction *=
		1e3 / (relation_radius[0] * relation_radius[0] * relation_radius[0]  * time_weight());
	return relation_to_color(reduction);
}
