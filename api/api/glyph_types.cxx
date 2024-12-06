
//////
//
// Includes
//

// Public interface
#include <OnTubeVis/OnTubeVis.h>



//////
//
// Language config
//

// Make sure we don't get the bugged-out warning on MSCV (we don't have custom constructors, the warning is just wrong)
#ifdef _MSC_VER
	#pragma warning(push)
	#pragma warning(disable : 4190)
#endif



//////
//
// Functions
//

////
// SurfaceColor

// --- ...Info ----------------------------------------------------------
OTV_API OTV_GlyphInfo otv__construct_SurfaceColorInfo (
	const OTV_ColorMap color_map, const OTV_InterpolationMode interpolation_mode
)
{
	OTV_GlyphInfo gi = otv__construct_empty_SurfaceColorInfo();
	auto &ret = *(OTV_SurfaceColorInfo*)&gi;
	ret.color_map = color_map;
	ret.interpolation_mode = interpolation_mode;
	return gi;
}

OTV_API OTV_GlyphInfo otv__construct_empty_SurfaceColorInfo (void)
{
	OTV_GlyphInfo gi{uint32_t((sizeof(OTV_SurfaceColorInfo)-sizeof(OTV_GlyphInfo::N))/sizeof(float))};
	return gi;
}

OTV_API OTV_SurfaceColorInfo* otv__upcast_SurfaceColorInfo (const OTV_GlyphInfo *surface_color_info) {
	return (OTV_SurfaceColorInfo*)const_cast<OTV_GlyphInfo*>(surface_color_info);
}

OTV_API OTV_GlyphInfo* otv__downcast_SurfaceColorInfo (const OTV_SurfaceColorInfo *surface_color_info) {
	return (OTV_GlyphInfo*)const_cast<OTV_SurfaceColorInfo*>(surface_color_info);
}

// --- ...Data ----------------------------------------------------------
OTV_API OTV_GlyphData otv__construct_SurfaceColorData (const float s, const float color)
{
	OTV_GlyphData gd = otv__construct_empty_SurfaceColorData();
	auto &ret = *(OTV_SurfaceColorData*)&gd;
	ret.s = s;
	ret.color = color;
	return gd;
}

OTV_API OTV_GlyphData otv__construct_empty_SurfaceColorData (void)
{
	OTV_GlyphData gd{
		uint32_t((sizeof(OTV_SurfaceColorData)-sizeof(OTV_GlyphData::N)-sizeof(OTV_GlyphData::s))/sizeof(float))
	};
	return gd;
}

OTV_API OTV_SurfaceColorData* otv__upcast_SurfaceColorData (const OTV_GlyphData *surface_color_data) {
	return (OTV_SurfaceColorData*)const_cast<OTV_GlyphData*>(surface_color_data);
}

OTV_API OTV_GlyphData* otv__downcast_SurfaceColorData (const OTV_SurfaceColorData *surface_color_data) {
	return (OTV_GlyphData*)const_cast<OTV_SurfaceColorData*>(surface_color_data);
}


////
// LinePlot

// --- ...Info ----------------------------------------------------------
OTV_API OTV_GlyphInfo otv__construct_LinePlotInfo (
	const OTV_InterpolationMode interpolation_mode, const uint32_t num_subplots, const OTV_Rgb subplot_colors[]
)
{
	OTV_GlyphInfo gi = otv__construct_empty_LinePlotInfo();
	auto &ret = *(OTV_LinePlotInfo*)&gi;
	ret.interpolation_mode = interpolation_mode;
	ret.num_subplots = num_subplots;
	for (unsigned i=0; i<num_subplots; i++)
		ret.subplot_colors[i] = subplot_colors[i];
	return gi;
}

OTV_API OTV_GlyphInfo otv__construct_empty_LinePlotInfo (void)
{
	OTV_GlyphInfo gi{uint32_t((sizeof(OTV_LinePlotInfo)-sizeof(OTV_GlyphInfo::N))/sizeof(float))};
	return gi;
}

OTV_API OTV_LinePlotInfo* otv__upcast_LinePlotInfo (const OTV_GlyphInfo *line_plot_info) {
	return (OTV_LinePlotInfo*)const_cast<OTV_GlyphInfo*>(line_plot_info);
}

OTV_API OTV_GlyphInfo* otv__downcast_LinePlotInfo (const OTV_LinePlotInfo *line_plot_info) {
	return (OTV_GlyphInfo*)const_cast<OTV_LinePlotInfo*>(line_plot_info);
}

// --- ...Data ----------------------------------------------------------
OTV_API OTV_GlyphData otv__construct_LinePlotData (const float s, const uint32_t num_values, const float values[])
{
	OTV_GlyphData gd = otv__construct_empty_LinePlotData();
	auto &ret = *(OTV_LinePlotData*)&gd;
	ret.s = s;
	for (unsigned i=0; i<num_values; i++)
		ret.values[i] = values[i];
	return gd;
}

OTV_API OTV_GlyphData otv__construct_empty_LinePlotData (void)
{
	OTV_GlyphData gd{
		uint32_t((sizeof(OTV_LinePlotData)-sizeof(OTV_GlyphData::N)-sizeof(OTV_GlyphData::s))/sizeof(float))
	};
	return gd;
}

OTV_API OTV_LinePlotData* otv__upcast_LinePlotData (const OTV_GlyphData *line_plot_data) {
	return (OTV_LinePlotData*)const_cast<OTV_GlyphData*>(line_plot_data);
}

OTV_API OTV_GlyphData* otv__downcast_LinePlotData (const OTV_LinePlotData *line_plot_data) {
	return (OTV_GlyphData*)const_cast<OTV_LinePlotData*>(line_plot_data);
}


////
// Rect

// --- ...Info ----------------------------------------------------------
OTV_API OTV_GlyphInfo otv__construct_RectangleInfo (
	const OTV_Rgb rgb, const OTV_ColorMap color_map, const float width, const float height,
	const OTV_RectangleInfoStaticFlags static_flags
)
{
	OTV_GlyphInfo gi = otv__construct_empty_RectangleInfo();
	auto &ret = *(OTV_RectangleInfo*)&gi;
	ret.rgb = rgb;
	ret.color_map = color_map;
	ret.width = width;
	ret.height = height;
	ret.static_flags = static_flags;
	return gi;
}

OTV_API OTV_GlyphInfo otv__construct_empty_RectangleInfo (void)
{
	OTV_GlyphInfo gi{uint32_t((sizeof(OTV_RectangleInfo)-sizeof(OTV_GlyphInfo::N))/sizeof(float))};
	return gi;
}

OTV_API OTV_RectangleInfo* otv__upcast_RectangleInfo (const OTV_GlyphInfo *line_plot_info) {
	return (OTV_RectangleInfo*)const_cast<OTV_GlyphInfo*>(line_plot_info);
}

OTV_API OTV_GlyphInfo* otv__downcast_RectangleInfo (const OTV_RectangleInfo *line_plot_info) {
	return (OTV_GlyphInfo*)const_cast<OTV_RectangleInfo*>(line_plot_info);
}

// --- ...Data ----------------------------------------------------------
OTV_API OTV_GlyphData otv__construct_RectangleData (
	const float s, const float color, const float width, const float height
){
	OTV_GlyphData gd = otv__construct_empty_RectangleData();
	auto &ret = *(OTV_RectangleData*)&gd;
	ret.s = s;
	ret.color = color;
	ret.width = width;
	ret.height = height;
	return gd;
}

OTV_API OTV_GlyphData otv__construct_empty_RectangleData (void)
{
	OTV_GlyphData gd{
		uint32_t((sizeof(OTV_RectangleData)-sizeof(OTV_GlyphData::N)-sizeof(OTV_GlyphData::s))/sizeof(float))
	};
	return gd;
}

OTV_API OTV_RectangleData* otv__upcast_RectangleData (const OTV_GlyphData *rectangle_data) {
	return (OTV_RectangleData*)const_cast<OTV_GlyphData*>(rectangle_data);
}

OTV_API OTV_GlyphData* otv__downcast_RectangleData (const OTV_RectangleData *rectangle_data) {
	return (OTV_GlyphData*)const_cast<OTV_RectangleData*>(rectangle_data);
}

OTV_API OTV_Vec2 otv__instantiate_Rectangle (
	const float traj_radius, const OTV_RectangleInfo *info, const OTV_RectangleData *data
){
	const float hw = .5f*traj_radius * (info->static_flags|RI_STATIC_WIDTH ? info->width : data->width);
	return {-hw, hw};
}


////
// SignBlob

// --- ...Info ----------------------------------------------------------
OTV_API OTV_GlyphInfo otv__construct_SignBlobInfo (
	const OTV_Rgb rgb, const OTV_ColorMap color_map, const float radius, const float value,
	const OTV_SignBlobInfoStaticFlags static_flags
)
{
	OTV_GlyphInfo gi = otv__construct_empty_SignBlobInfo();
	auto &ret = *(OTV_SignBlobInfo*)&gi;
	ret.rgb = rgb;
	ret.color_map = color_map;
	ret.radius = radius;
	ret.value = value;
	ret.static_flags = static_flags;
	return gi;
}

OTV_API OTV_GlyphInfo otv__construct_empty_SignBlobInfo (void)
{
	OTV_GlyphInfo gi{uint32_t((sizeof(OTV_SignBlobInfo)-sizeof(OTV_GlyphInfo::N))/sizeof(float))};
	return gi;
}

OTV_API OTV_SignBlobInfo* otv__upcast_SignBlobInfo (const OTV_GlyphInfo *line_plot_info) {
	return (OTV_SignBlobInfo*)const_cast<OTV_GlyphInfo*>(line_plot_info);
}

OTV_API OTV_GlyphInfo* otv__downcast_SignBlobInfo (const OTV_SignBlobInfo *line_plot_info) {
	return (OTV_GlyphInfo*)const_cast<OTV_SignBlobInfo*>(line_plot_info);
}

// --- ...Data ----------------------------------------------------------
OTV_API OTV_GlyphData otv__construct_SignBlobData (
	const float s, const float color, const float value
){
	OTV_GlyphData gd = otv__construct_empty_SignBlobData();
	auto &ret = *(OTV_SignBlobData*)&gd;
	ret.s = s;
	ret.color = color;
	ret.value = value;
	return gd;
}

OTV_API OTV_GlyphData otv__construct_empty_SignBlobData (void)
{
	OTV_GlyphData gd{
		uint32_t((sizeof(OTV_SignBlobData)-sizeof(OTV_GlyphData::N)-sizeof(OTV_GlyphData::s))/sizeof(float))
	};
	return gd;
}

OTV_API OTV_SignBlobData* otv__upcast_SignBlobData (const OTV_GlyphData *sign_blob_data) {
	return (OTV_SignBlobData*)const_cast<OTV_GlyphData*>(sign_blob_data);
}

OTV_API OTV_GlyphData* otv__downcast_SignBlobData (const OTV_SignBlobData *sign_blob_data) {
	return (OTV_GlyphData*)const_cast<OTV_SignBlobData*>(sign_blob_data);
}

OTV_API OTV_Vec2 otv__instantiate_SignBlob (
	const float traj_radius, const OTV_SignBlobInfo *info, const OTV_SignBlobData *
){
	const float hw = traj_radius*info->radius;
	return {-hw, hw};
}


////
// Stringifier

OTV_API const char *const otv__string_from_ColorMap (const OTV_ColorMap color_map)
{
	switch (color_map)
	{
		// Sequential color maps, since OnTubeVis API v0
		case Acton: return "Acton";
		case Bamako: return "Bamako";
		case Batlow: return "Batlow";
		case BatlowK: return "BatlowK";
		case BatlowW: return "BatlowW";
		case Bilbao: return "Bilbao";
		case Buda: return "Buda";
		case Davos: return "Davos";
		case Devon: return "Devon";
		case Glasgow: return "Glasgow";
		case GrayC: return "GrayC";
		case Hawaii: return "Hawaii";
		case Imola: return "Imola";
		case Lajolla: return "Lajolla";
		case Lapaz: return "Lapaz";
		case Lipari: return "Lipari";
		case Navia: return "Navia";
		case Nuuk: return "Nuuk";
		case Oslo: return "Oslo";
		case Rainbow: return "Rainbow";
		case Tokyo: return "Tokyo";
		case Turbo: return "Turbo";
		case Turku: return "Turku";

		// Diverging color maps, since OnTubeVis API v0
		case Bam: return "Bam";
		case Berlin: return "Berlin";
		case Broc: return "Broc";
		case Cork: return "Cork";
		case Lisbon: return "Lisbon";
		case Managua: return "Managua";
		case Roma: return "Roma";
		case Tofino: return "Tofino";
		case Vanimo: return "Vanimo";
		case Vik: return "Vik";

		// Unknown. Cause the calling application to crash so they notice their mistake.
		default:
			return nullptr;
	}
}

OTV_API const char *const otv__string_from_GlyphType (const OTV_GlyphType glyph_type)
{
	switch (glyph_type)
	{
		// Discrete glyphs, since OnTubeVis API v0
		case Circle: return "Cirlce";
		case Rect: return "Rect";
		case IsoscelesTriangle: return "IsoscelesTriangle";
		case SignBlob: return "SignBlob";

		// Plot-likes, since OnTubeVis API v0
		case SurfaceColor: return "SurfaceColor";
		case LinePlot: return "LinePlot";

		// Unknown. Cause the calling application to crash so they notice their mistake.
		default:
			return nullptr;
	}
}

OTV_API const char *const otv__string_from_InterpolationMode (const OTV_InterpolationMode interpolation_mode)
{
	switch (interpolation_mode)
	{
		// Since OnTubeVis API v0
		case Nearest: return "Nearest";
		case Linear: return "Linear";
		case Cubic: return "Cubic";

		// Unknown. Cause the calling application to crash so they notice their mistake.
		default:
			return nullptr;
	}
}
