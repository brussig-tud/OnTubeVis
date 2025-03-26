
#ifndef __COMMON_H__
#define __COMMON_H__


//////
//
// Includes
//

// C++ STL
#include <tuple>
#include <vector>
#include <cassert>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// OnTubeVis internals
#include "../util.h"

// Local includes
#include "layer/properties.h"



//////
//
// Typedefs
//

/// A minimal RAII wrapper around the @ref OTV_VisSetupHandle.
typedef RAII<OTV_VisSetupHandle, otv__free_VisSetup> VisSetup;



//////
//
// Classes
//

struct SurfaceColorLayer
{
	typedef OTV_SurfaceColorInfo Info;

	OTV_LayerConfig config;
	Info &info;

	Info& init_info (void) {
		return *otv__upcast_SurfaceColorInfo(&config.static_params);
	}

	inline SurfaceColorLayer(const SurfaceColorLayer &other) noexcept
		: config(other.config), info(init_info())
	{}

	inline SurfaceColorLayer(SurfaceColorLayer &&other) noexcept
		: config(std::move(other.config)), info(init_info())
	{}

	SurfaceColorLayer(const OTV_ColorMap &colormap, const OTV_InterpolationMode &interpolation_mode)
		: config{
			.type=SurfaceColor, .outline=0,
			.static_params=otv__construct_SurfaceColorInfo(
				/* rgb (irgnored, see static flags): */otv__Rgb(0,0,0), /* color_map: */colormap,
				/*interpolation_mode: */interpolation_mode, /*static flags: */SCI_STATIC_NONE
			)
		}, info(init_info())
	{}
};

struct RectangleLayer
{
	typedef OTV_RectangleInfo Info;

	OTV_LayerConfig config;
	Info &info;

	Info& init_info (void) {
		return *otv__upcast_RectangleInfo(&config.static_params);
	}

	inline RectangleLayer(const RectangleLayer &other) noexcept
		: config(other.config), info(init_info())
	{}

	inline RectangleLayer(RectangleLayer &&other) noexcept
		: config(std::move(other.config)), info(init_info())
	{}

	RectangleLayer(const float outline, const OTV_Rgb &color)
		: config{
			.type=Rect, .outline=outline,
			.static_params=otv__construct_RectangleInfo(
				/* rgb: */color, /* color_map (irgnored, see static flags): */UndefinedColormap,
				/* width (irgnored, see static flags): */0, /* height (irgnored, see static flags): */0,
				/*static flags: */RI_STATIC_COLOR
			)
		}, info(init_info())
	{}
	RectangleLayer(const float outline, const OTV_ColorMap colormap, const layer::WidthProperty width)
		: config{
			.type=Rect, .outline=outline,
			.static_params=otv__construct_RectangleInfo(
				/* rgb (irgnored, see static flags): */otv__Rgb(0,0,0), /* color_map: */colormap,
				/* width: */width.value, /* height (irgnored, see static flags): */0,
				/*static flags: */RI_STATIC_WIDTH
			)
		}, info(init_info())
	{}
	RectangleLayer(const float outline, const OTV_ColorMap colormap, const layer::HeightProperty height)
		: config{
			.type=Rect, .outline=outline,
			.static_params=otv__construct_RectangleInfo(
				/* rgb (irgnored, see static flags): */otv__Rgb(0,0,0), /* color_map: */colormap,
				/* width (irgnored, see static flags): */0, /* height: */height.value,
				/*static flags: */RI_STATIC_HEIGHT
			)
		}, info(init_info())
	{}
	RectangleLayer(
		const float outline, const OTV_ColorMap colormap, const layer::WidthProperty width,
		const layer::HeightProperty height
	)
		: config{
			.type=Rect, .outline=outline,
			.static_params=otv__construct_RectangleInfo(
				/* rgb (irgnored, see static flags): */otv__Rgb(0,0,0), /* color_map: */colormap,
				/* width: */width.value, /* height: */height.value,
				/*static flags: */static_cast<OTV_RectangleInfoStaticFlags>(RI_STATIC_WIDTH | RI_STATIC_HEIGHT)
			)
		}, info(init_info())
	{}
	RectangleLayer(const float outline, const OTV_Rgb &color, const layer::WidthProperty width)
		: config{
			.type=Rect, .outline=outline,
			.static_params=otv__construct_RectangleInfo(
				/* rgb: */color, /* color_map (irgnored, see static flags): */UndefinedColormap,
				/* width: */width.value, /* height (irgnored, see static flags): */0,
				/*static flags: */static_cast<OTV_RectangleInfoStaticFlags>(RI_STATIC_COLOR | RI_STATIC_WIDTH)
			)
		}, info(init_info())
	{}
	RectangleLayer(const float outline, const OTV_Rgb &color, const layer::HeightProperty height)
		: config{
			.type=Rect, .outline=outline,
			.static_params=otv__construct_RectangleInfo(
				/* rgb: */color, /* color_map (irgnored, see static flags): */UndefinedColormap,
				/* width (irgnored, see static flags): */0, /* height: */height.value,
				/*static flags: */static_cast<OTV_RectangleInfoStaticFlags>(RI_STATIC_COLOR | RI_STATIC_HEIGHT)
			)
		}, info(init_info())
	{}
	RectangleLayer(const float outline, const OTV_ColorMap colormap)
		: config{
			.type=Rect, .outline=outline,
			.static_params=otv__construct_RectangleInfo(
				/* rgb (irgnored, see static flags): */otv__Rgb(0,0,0), /* color_map: */colormap,
				/* width (irgnored, see static flags): */0, /* height (irgnored, see static flags): */0,
				/*static flags: */RI_STATIC_NONE
			)
		}, info(init_info())
	{}
};

struct SignBlobLayer
{
	typedef OTV_SignBlobInfo Info;

	OTV_LayerConfig config;
	Info &info;

	Info& init_info (void) {
		return *otv__upcast_SignBlobInfo(&config.static_params);
	}

	inline SignBlobLayer(const SignBlobLayer &other) noexcept
		: config(other.config), info(init_info())
	{}

	inline SignBlobLayer(SignBlobLayer &&other) noexcept
		: config(std::move(other.config)), info(init_info())
	{}

	SignBlobLayer(const float outline, const float radius, const OTV_Rgb &color)
		: config{
			.type=SignBlob, .outline=outline,
			.static_params=otv__construct_SignBlobInfo(
				/* rgb: */color, /* color_map (irgnored, see static flags): */UndefinedColormap, /* radius: */radius,
				/* value (irgnored, see static flags): */0, /*static flags: */SBI_STATIC_COLOR
			)
		}, info(init_info())
	{}
	SignBlobLayer(
		const float outline, const float radius, const OTV_ColorMap colormap, const layer::ValueProperty value
	)
		: config{
			.type=SignBlob, .outline=outline,
			.static_params=otv__construct_SignBlobInfo(
				/* rgb (irgnored, see static flags): */otv__Rgb(0,0,0), /* color_map: */colormap,
				/* radius: */radius, /* value: */value.value, /*static flags: */SBI_STATIC_VALUE
			)
		}, info(init_info())
	{}
	SignBlobLayer(const float outline, const float radius, const OTV_ColorMap colormap)
		: config{
			.type=SignBlob, .outline=outline,
			.static_params=otv__construct_SignBlobInfo(
				/* rgb (irgnored, see static flags): */otv__Rgb(0,0,0), /* color_map: */colormap, /* radius: */radius,
				/* value (irgnored, see static flags): */0, /*static flags: */SBI_STATIC_NONE
			)
		}, info(init_info())
	{}
};

template <typename T, typename F, std::size_t... I>
constexpr void visit_impl(T& tup, const size_t idx, F fun, std::index_sequence<I...>)
{
	assert(idx < std::tuple_size_v<T>);
	((I == idx ? fun(std::get<I>(tup)) : void()), ...);
}
template <typename F, typename... Ts, typename Indices = std::make_index_sequence<sizeof...(Ts)>>
constexpr void visit_at(const std::tuple<Ts...>& tup, const size_t idx, F fun)
{
	visit_impl(tup, idx, fun, Indices {});
}

template <class ...LayerTypes>
struct OTVConfiguration
{
	/// The name of the visualization this configuration is for.
	const std::string name;

	/// The configurations for each layer.
	const std::tuple<LayerTypes...> layers;

	/// The number of configured layers.
	static constexpr auto num_layers = (unsigned)std::tuple_size_v<decltype(layers)>;

	/// The number of configured trajectories.
	const unsigned num_trajs;

	/// The array of indices for the configured trajectories.
	std::vector<unsigned> traj_ids;

	/// The configured extrapolation length
	const unsigned extrapol_length;

	/// The configured tube radius.
	const float tube_radius;

	/// Instantiante a particular configuration.
	OTVConfiguration(
		const std::string &name, const unsigned num_trajs, const unsigned extrapol_length, const float tube_radius,
		LayerTypes ...layers
	)
		: name(name), num_trajs(num_trajs), extrapol_length(extrapol_length), tube_radius(tube_radius),
		  layers(std::move(layers)...)
	{}

	/// Apply this configuration to the given visualization setup.
	bool apply (VisSetup &setup)
	{
		// Apply layer configuration
		bool success = true;
		for (unsigned l=0; l<num_layers; ++l) {
			const auto &layer = get_layer_config(l);
			success &= otv__add_layer(setup.handle, &layer);
		}
		if (!success)
			return false;

		// Add configured amount of trajectories and store their IDs
		traj_ids.reserve(num_trajs);
		for (unsigned i=0; i<num_trajs; ++i)
			traj_ids.emplace_back(otv__add_trajectory(setup.handle, tube_radius));

		// Set configured extrapolation length
		otv__extrapolation_length(setup.handle, extrapol_length);

		// Done!
		return true;
	}

	/// Obtain a reference to the @a l-th layer config.
	const OTV_LayerConfig& get_layer_config (const unsigned l) {
		const OTV_LayerConfig *config;
		visit_at(layers, l, [&config](auto&& arg) {
			config = &arg.config;
		});
		return *config;
	}
};


#endif // ifndef __COMMON_H__
