#define RELATION_REDUCE_T float

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return -2;
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T max_cos)
{
	const float cos = dot(normalize(args.offset), normalize(args.base_point.derivative));
	max_cos = max(cos, max_cos);
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T max_cos)
{
	if (max_cos == -2) return relation_background_color;
	return relation_to_color(1 - acos(max_cos)*.636619772);
}
