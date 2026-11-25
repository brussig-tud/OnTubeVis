struct Accum {
	float weight;
	float time;
};
#define RELATION_REDUCE_T Accum

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Accum(0, 0);
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T accum)
{
	accum.weight += args.duration * weight_all(args);
	accum.time += args.duration;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T accum)
{
	if (relation_normalize && accum.time != 0) accum.weight /= accum.time;
	return relation_to_color(accum.weight);
}
