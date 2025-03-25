
#ifndef __LAYER_PROPERTIES_H__
#define __LAYER_PROPERTIES_H__


//////
//
// Includes
//

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>



//////
//
// Module namespace(s)
//

namespace layer {



//////
//
// Structs
//

struct WidthProperty {
	float value;
	inline operator float& (){ return value; }
};

struct HeightProperty {
	float value;
	inline operator float& (){ return value; }
};

struct ValueProperty {
	float value;
	inline operator float& (){ return value; }
};

struct ColormapProperty {
	OTV_ColorMap type;
	inline operator OTV_ColorMap& (){ return type; }
};



//////
//
// Module namespace(s) close
//

// ::layer
}


#endif // ifndef __LAYER_PROPERTIES_H__
