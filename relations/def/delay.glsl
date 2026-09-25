struct Reduction
{
	float alignment;
	float delay;
};
#define RELATION_REDUCE_T Reduction

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Reduction(-1./0, 0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	const float alignment =
		  dot(normalize(args.base_point.derivative), normalize(args.sample_point.derivative))
		* weight_space(args);

	if (alignment > reduction.alignment) {
		reduction.alignment = alignment;
		reduction.delay = args.base_point.time - args.sample_point.time;
	}
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (isinf(reduction.alignment)) return relation_background_color;

	if (relation_normalize) {
		const float r = relation_radius[reduction.delay < 0 ? 1 : 2];
		if (r != 0) reduction.delay /= r;
	}
	return relation_to_color(reduction.delay);
}
