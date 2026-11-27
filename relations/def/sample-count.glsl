#define RELATION_REDUCE_T uint

RELATION_REDUCE_T init_relation (InitRelationArgs args)
{
	return 0;
}
void eval_relation (EvalRelationArgs args, inout RELATION_REDUCE_T count)
{
	++count;
}
vec3 color_relation (ColorRelationArgs args, RELATION_REDUCE_T count)
{
	return relation_to_color(count);
}
