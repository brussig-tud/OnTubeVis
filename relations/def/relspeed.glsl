struct Reduction {
	float speed;
	float weight;
};
#define RELATION_REDUCE_T Reduction

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Reduction(0, 0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	const float weight = (args.duration * weight_all(args));
	reduction.speed += length(args.sample_point.derivative) * weight;
	reduction.weight += weight;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (reduction.weight == 0) return relation_background_color;

	const float base_speed = length(args.base_point.derivative);
	float result = base_speed - reduction.speed / reduction.weight;
	if (relation_normalize) result /= base_speed;
	return relation_to_color(result);
}
