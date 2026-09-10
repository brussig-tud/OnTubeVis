struct Reduction
{
	float alignment;
	float delay;
};

#define RELATION_REDUCE_T Reduction

RELATION_REDUCE_T init_relation (TrajPoint base)
{
	return Reduction(-1./0, 0);
}
void eval_relation (
	TrajPoint base_point,
	TrajPoint sample_point,
	SampleWeights weight,
	inout RELATION_REDUCE_T reduction
) {
	const vec3 offset = sample_point.position - base_point.position;
	const float alignment =
		dot(normalize(base_point.derivative), normalize(sample_point.derivative))
		* (sqr(relation_radius[0]) - dot(offset, offset));

	if (alignment > reduction.alignment) {
		reduction.alignment = alignment;
		reduction.delay = base_point.time - sample_point.time;
	}
}
vec3 color_relation (TrajPoint base, RELATION_REDUCE_T reduction)
{
	float delay = sign(reduction.delay) * (relation_radius[1] - abs(reduction.delay));
	if (relation_normalize) delay /= relation_radius[1];
	return relation_to_color(delay);
}
