#define RELATION_REDUCE_T vec3

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return vec3(0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	if (relation_normalize) args.sample_point.derivative = normalize(args.sample_point.derivative);
	reduction += args.sample_point.derivative * (args.duration * weight_all(args));
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (reduction == vec3(0)) return relation_to_color(0);
	return relation_to_color(dot(normalize(args.base_point.derivative), normalize(reduction)));
}
