struct Reduction {
	float dist2;
	vec3 offset;
};
#define RELATION_REDUCE_T Reduction

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return Reduction(1./0, vec3(0));
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T reduction)
{
	const float dist2 = dot(args.offset, args.offset);
	if (dist2 >= reduction.dist2) return;
	reduction.dist2 = dist2;
	reduction.offset = args.offset;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T reduction)
{
	if (isinf(reduction.dist2)) return relation_background_color;
	const float cosine = dot(normalize(args.base_point.derivative), normalize(reduction.offset));
	return relation_to_color(.5 - acos(cosine) / 3.1415926535);
}
