
////
// INTERNAL HEADER - DO NOT INCLUDE!!!!!


#ifndef __GLYPHS_H__
#define __GLYPHS_H__


//////
//
// Includes
//

// C++ STL
#include <map>
#include <string>
#include <sstream>

// Private interface
#include <state/core.h>



//////
//
// Functions
//

bool check_glyph_instance_type (const OTV_GlyphType type, const OTV_GlyphData &glyph)
{
	static const std::map<OTV_GlyphType, OTV_GlyphData> ref = {
		{OTV_GlyphType::SurfaceColor, otv__construct_empty_SurfaceColorData()},
		{OTV_GlyphType::LinePlot, otv__construct_empty_LinePlotData()},
		{OTV_GlyphType::Circle, otv__construct_empty_CircleData()},
		{OTV_GlyphType::Rect, otv__construct_empty_RectangleData()},
		{OTV_GlyphType::IsoscelesTriangle, otv__construct_empty_IsoscelesTriangleData()},
		{OTV_GlyphType::SignBlob, otv__construct_empty_SignBlobData()}
	};
	return ref.at(type).N == glyph.N;
}

std::string fmt_glyph_instance (const OTV_LayerConfig &config, const OTV_GlyphData &glyph)
{
	std::stringstream str;
	str << otv__string_from_GlyphType(config.type)<<"(s:"<<glyph.s;
	switch (config.type)
	{
		case OTV_GlyphType::SurfaceColor: {
			const auto &sc = *otv__upcast_SurfaceColorData(&glyph);
			str << ", color:"<<sc.color;
			break;
		}
		case OTV_GlyphType::LinePlot: {
			const auto &lp = *otv__upcast_LinePlotData(&glyph);
			for (unsigned i=0; i<4; i++)
				str << ", value"<<i<<':'<<lp.values;
			break;
		}

		case OTV_GlyphType::Circle: {
			const auto &cfg = *otv__upcast_CircleInfo(&config.static_params);
			const auto &r = *otv__upcast_CircleData(&glyph);
			str << ", color:";
			if (cfg.static_flags & CI_STATIC_COLOR)  str << "<static>";
			else                                     str << r.color;
			str << ", radius:";
			if (cfg.static_flags & CI_STATIC_RADIUS) str << "<static>";
			else                                     str << r.radius;
			break;
		}
		case OTV_GlyphType::Rect: {
			const auto &cfg = *otv__upcast_RectangleInfo(&config.static_params);
			const auto &r = *otv__upcast_RectangleData(&glyph);
			str << ", color:";
			if (cfg.static_flags & RI_STATIC_COLOR)  str << "<static>";
			else                                     str << r.color;
			str << ", width:";
			if (cfg.static_flags & RI_STATIC_WIDTH)  str << "<static>";
			else                                     str << r.width;
			str << ", height:";
			if (cfg.static_flags & RI_STATIC_HEIGHT) str << "<static>";
			else                                     str << r.height;
			break;
		}
		case OTV_GlyphType::IsoscelesTriangle: {
			const auto &cfg = *otv__upcast_IsoscelesTriangleInfo(&config.static_params);
			const auto &r = *otv__upcast_IsoscelesTriangleData(&glyph);
			str << ", color:";
			if (cfg.static_flags & ITI_STATIC_COLOR)       str << "<static>";
			else                                           str << r.color;
			str << ", width:";
			if (cfg.static_flags & ITI_STATIC_WIDTH)       str << "<static>";
			else                                           str << r.width;
			str << ", height:";
			if (cfg.static_flags & ITI_STATIC_HEIGHT)      str << "<static>";
			else                                           str << r.height;
			str << ", orientation:";
			if (cfg.static_flags & ITI_STATIC_ORIENTATION) str << "<static>";
			else                                           str << r.orientation;
			break;
		}
		case OTV_GlyphType::SignBlob: {
			const auto &cfg = *otv__upcast_SignBlobInfo(&config.static_params);
			const auto &sb = *otv__upcast_SignBlobData(&glyph);
			str << ", color:";
			if (cfg.static_flags & SBI_STATIC_COLOR) str << "<static>";
			else                                     str << sb.color;
			str << ", value:";
			if (cfg.static_flags & SBI_STATIC_VALUE) str << "<static>";
			else                                     str << sb.value;
			break;
		}

		default: /* do_nothing() */;
	}
	str << ")";
	return str.str();
}


#endif // ifdef __GLYPHS_H__
