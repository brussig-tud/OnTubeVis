struct Accum {
	float align;
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
	accum.align += weight
		* dot(normalize(args.base_point.derivative), normalize(args.sample_point.derivative));
	accum.weight += weight;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T accum)
{
	if (accum.weight == 0) return relation_background_color;
	if (relation_normalize) return relation_to_color(accum.align / accum.weight);
	accum.align *= 1e2 / (query_volume() * query_duration());
	return relation_to_color(accum.align);
}
