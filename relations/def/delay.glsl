struct Reduction
{
	float alignment;
	float time;
};
#define RELATION_REDUCE_T Reduction

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Reduction(-1./0, 0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	const float alignment =
		  (1 + dot(normalize(args.base_point.derivative), normalize(args.sample_point.derivative)))
		* weight_space(args);

	if (alignment > reduction.alignment) {
		reduction.alignment = alignment;
		reduction.time = args.sample_point.time;
	}
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (isinf(reduction.alignment)) return relation_background_color;

	float delay = args.base_point.time - reduction.time;
	if (relation_normalize) {
		const float r = relation_radius[delay < 0 ? 2 : 1];
		if (r != 0) delay /= r;
	}
	return relation_to_color(delay);
}
