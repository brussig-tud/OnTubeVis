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
	reduction += (relation_radius[0] - length(base_point.position - sample_point.position))
		* (weight.time * weight.angle);
}
vec3 color_relation (TrajPoint base, RELATION_REDUCE_T reduction)
{
	if (relation_normalize) reduction /= (relation_radius[0] + 2*relation_radius[1]);
	return relation_to_color(reduction);
}
