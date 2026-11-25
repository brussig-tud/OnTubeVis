struct Accum {
	float speed;
	float weight;
};
#define RELATION_REDUCE_T Accum

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Accum(0, 0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T accum)
{
	const float weight = args.duration * weight_all(args);
	accum.speed += length(args.sample_point.derivative) * weight;
	accum.weight += weight;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T accum)
{
	if (accum.weight == 0) return relation_background_color;

	const float base_speed = length(args.base_point.derivative);
	float result = base_speed - accum.speed / accum.weight;
	if (relation_normalize) result /= base_speed;
	return relation_to_color(result);
}
