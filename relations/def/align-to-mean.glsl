#define RELATION_REDUCE_T vec3

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return vec3(0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T mean_dir)
{
	if (relation_normalize) args.sample_point.derivative = normalize(args.sample_point.derivative);
	mean_dir += args.duration * weight_all(args) * args.sample_point.derivative;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T mean_dir)
{
	if (mean_dir == vec3(0)) return relation_to_color(0);
	return relation_to_color(dot(normalize(args.base_point.derivative), normalize(mean_dir)));
}
