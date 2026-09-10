#define RELATION_REDUCE_T float

RELATION_REDUCE_T init_relation (TrajPoint base)
{
	return 0;
}
void eval_relation (
	TrajPoint base_point,
	TrajPoint sample_point,
	SampleWeights weight,
	inout RELATION_REDUCE_T reduction
) {
	const vec3 offset = sample_point.position - base_point.position;
	reduction += dot(normalize(base_point.derivative), normalize(sample_point.derivative))
		* exp(dot(offset, offset) * (-6 / sqr(relation_radius[0])))
		* (weight.time * weight.angle);
}
vec3 color_relation (TrajPoint base, RELATION_REDUCE_T reduction)
{
	if (relation_normalize) reduction /= 2*relation_radius[1];
	return relation_to_color(reduction);
}
