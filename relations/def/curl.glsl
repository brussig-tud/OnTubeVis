struct Reduction {
	mat3 jacobi;
	uint nsamples;
};
#define RELATION_REDUCE_T Reduction

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Reduction(mat3(0), 0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	reduction.jacobi += outerProduct(
		(args.sample_point.derivative - args.base_point.derivative) * args.duration,
		1 / args.offset
	);
	++reduction.nsamples;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (reduction.nsamples == 0) return relation_background_color;

	const mat3 j = reduction.jacobi / reduction.nsamples;
	const vec3 curl = vec3(j[1].z - j[2].y, j[2].x - j[0].z, j[0].y - j[1].x);
	return relation_to_color(curl.y);
}
