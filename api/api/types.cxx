
//////
//
// Includes
//

// Public interface
#include <OnTubeVis/OnTubeVis.h>



//////
//
// Functions
//

OTV_API OTV_Vec2 otv__Vec2 (const float x, const float y) {
	return {x, y};
}

OTV_API OTV_Vec3 otv__Vec3 (const float x, const float y, const float z) {
	return {x, y, z};
}

OTV_API OTV_Vec4 otv__Vec4 (const float x, const float y, const float z, const float w) {
	return {x, y, z, w};
}

OTV_API OTV_Rgb otv__Rgb (const float r, const float g, const float b) {
	return {r, g, b};
}

OTV_API OTV_Interval otv__Interval (const float min, const float max) {
	return {min, max};
}
