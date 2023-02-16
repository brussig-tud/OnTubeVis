cmake_minimum_required(VERSION 3.20)

project(OnTubeVis)

# preset CGV Framework options
option(CGV_BUILD_WITH_AUDIO OFF)
option(CGV_BUILD_EXAMPLES OFF)
set(CGV_CPPSTD17 OFF CACHE BOOL "" FORCE) # FIXME: C++17 implementation of cgv_utils (using std::filesystem) is currently broken!

# OnTubeVis options
set(ENABLE_OPTIX_SUPPORT_DOC "Build with support for hardware raytracing using NVIDIA OptiX")
option(ENABLE_OPTIX_SUPPORT ${ENABLE_OPTIX_SUPPORT_DOC} ON)

# Fetch CGV Framework
include(FetchContent)
FetchContent_Declare(
	cgv GIT_REPOSITORY https://github.com/sgumhold/cgv.git
	GIT_TAG b646c54d0bd9bf3b9c243847e788b55b9a42febe
)
FetchContent_MakeAvailable(cgv)

# check optional dependencies
# - include additional find scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# - dependencies needed for hardware raytracing
if (ENABLE_OPTIX_SUPPORT)
	find_package(CUDA)
	find_package(OptiX)
endif()
# - other optional dependencies
find_package(OpenMP)

# handle optional dependencies
if (CUDA_FOUND AND OptiX_INCLUDE AND NOT OptiX_INCLUDE STREQUAL "OptiX_INCLUDE-NOTFOUND")
	set(__EMIT_OPTIX_WARNING TRUE)
	set(ENABLE_OPTIX_SUPPORT OFF CACHE BOOL ${ENABLE_OPTIX_SUPPORT_DOC} FORCE) #list(APPEND FEATURE_DEFINES "RTX_SUPPORT=1")
else()
	set(ENABLE_OPTIX_SUPPORT OFF CACHE BOOL ${ENABLE_OPTIX_SUPPORT_DOC} FORCE)
endif()
if (OpenMP_CXX_FOUND)
	set(DEPENDENCY_OPENMP OpenMP::OpenMP_CXX)
	list(APPEND FEATURE_DEFINES "OMP_SUPPORT=1")
endif()

# gather source files
# - additional include paths required when baking sub-components into the codebase like we do below
set(ADDITIONAL_INCLUDES
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rd/fastgl>" "$<INSTALL_INTERFACE:3rd/fastgl>"
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rd/peridetic>" "$<INSTALL_INTERFACE:3rd/peridetic>"
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rd/wgs84mercator>" "$<INSTALL_INTERFACE:3rd/wgs84mercator>"
)
# - C/C++ sources
set(SOURCES
	# 3rd party code
	# - FastGL library
	3rd/fastgl/fastgl.cpp

	# arclength helpers
	arclength/bezier.cpp
	arclength/hermite.cpp

	# main app
	arclen_helper.cxx
	attrib_handle_manager.cxx
	bezdat_handler.cxx
	color_map_manager.cxx
	color_map_viewer.cxx
	csv_handler.cxx
	glyph_attribute_mapping.cxx
	glyph_layer_manager.cxx
	obd_handler.cxx
	on_tube_vis.cxx
	regulargrid.cxx
	sepia_handler.cxx
	textured_spline_tube_renderer.cxx
	tgen_handler.cxx
	traj_loader.cxx
	voxelizer.cxx
)
# - C/C++ headers
set(HEADERS
	# 3rd party code
	# - FastGL library
	3rd/fastgl/fastgl.h
	# - Peridetic ECEF/geodetics header-only library
	3rd/peridetic/periDetail.h
	3rd/peridetic/peridetic.h
	# - Wgs84mercator geoid utility header-only library
	3rd/wgs84mercator/WGS84toCartesian.hpp
	
	# arclength helpers
	arclength/bezier.h
	arclength/hermite.h
	arclength/PolynomialRegression.h
	arclength/v3.h

	# main app
	ambient_occlusion_style.h
	arclen_helper.h
	attrib_handle_manager.h
	bezdat_handler.h
	color_map_manager.h
	color_map_viewer.h
	csv_handler.h
	curveutils.h
	demo.h
	glyph_attribute_mapping.h
	glyph_compiler.h
	glyph_layer_manager.h
	glyph_shapes.h
	gui_util.h
	hermite_spline_tube.h
	obd_handler.h
	on_tube_vis.h
	quadratic_bezier_tube.h
	regulargrid.h
	sepia_handler.h
	textured_spline_tube_renderer.h
	tgen_handler.h
	traj_loader.h
	voxel_grid.h
	voxelizer.h
)
# - GLSL shaders
set (SHADERS
	glsl/clamp.glcs
	glsl/clear.glcs
	glsl/color_maps.glfs
	glsl/color_maps.glpr
	glsl/fxaa.glfs
	glsl/fxaa.glpr
	glsl/mipmap.glcs
	glsl/optix_display.glfs
	glsl/optix_display.glpr
	glsl/screen.glfs
	glsl/screen.glpr
	glsl/surface_mod.glsl
	glsl/taa_resolve.glfs
	glsl/taa_resolve.glpr
	glsl/textured_spline_tube_shading.glfs
	glsl/textured_spline_tube_shading.glpr
	glsl/textured_spline_tube.glfs
	glsl/textured_spline_tube.glgs
	glsl/textured_spline_tube.glpr
	glsl/textured_spline_tube.glvs
	glsl/voxelize.glcs
)
# - add contitional source files
if (ENABLE_OPTIX_SUPPORT)
	list(APPEND SOURCES
		optix_integration.cxx
		optixtracer_textured_spline_tube.cxx
	)
	list(APPEND HEADERS
		optix_integration.h
		optixtracer_textured_spline_tube.h
	)
endif()
# - resources (we'll likely get away with just globbing them)
file(GLOB_RECURSE IMAGES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "*.png" "*.bmp")

# create the OnTubeVis plugin wihtin the CGV Framework build infrastructure
cgv_add_target(OnTubeVis
	TYPE plugin
	SOURCES         ${SOURCES}
	HEADERS         ${HEADERS}
	SHADER_SOURCES  ${SHADERS}
	RESOURCES       ${IMAGES}
	DEPENDENCIES
		glew json fltk cgv_viewer cgv_utils cgv_type cgv_reflect cgv_data cgv_signal cgv_base cgv_media cgv_gui cgv_render cgv_os
		cg_fltk cgv_gl cg_ext cmi_io cg_msdf crg_stereo_view crg_light crg_grid cgv_g2d cgv_app cgv_gpgpu ${DEPENDENCY_OPENMP}

	LINKTIME_PLUGIN_DEPENDENCIES cg_fltk  # <-- we're directly using exported symbols from the cg_fltk GUI provider plugin

	ADDITIONAL_CMDLINE_ARGS
		"config:\"${CMAKE_CURRENT_LIST_DIR}/config.def\""

	OVERRIDE_SHARED_EXPORT_DEFINE TUBES_EXPORTS
)

# schedule a warning message at the very end of the configuration process to make sure it will be easily visible
# we schedule the message after creating the OnTubeVis target such that it's deferred operations alert appears first
if (NOT OpenMP_CXX_FOUND)
	cmake_language(
		DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL message
		"WARNING: OpenMP development files not found on the system!\n"
		"         Preprocessing for large datasets will be slow."
	)
endif()

# schedule temporary OptiX warning to make sure it will be seen - TODO: remove once ready
if (__EMIT_OPTIX_WARNING)
	cmake_language(
		DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL message
		"WARNING: OptiX support is pending integration with CMake build system!\n"
		"         disabling... use native .pj build on Windows to enable OptiX render path"
	)
endif()

# make sure we can compile
target_compile_features(OnTubeVis PUBLIC cxx_std_17)
target_compile_features(OnTubeVis_static PUBLIC cxx_std_17)
set_target_properties(OnTubeVis OnTubeVis_static PROPERTIES CXX_STANDARD 17)
set_target_properties(OnTubeVis OnTubeVis_static PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(OnTubeVis OnTubeVis_static PROPERTIES CXX_EXTENSIONS ON)

# set our additional defines
target_compile_definitions(OnTubeVis PUBLIC ${FEATURE_DEFINES} "INPUT_DIR=${CMAKE_CURRENT_LIST_DIR}")
target_compile_definitions(OnTubeVis_static PUBLIC ${FEATURE_DEFINES} "INPUT_DIR=.")

# make sure to register our additional include directories
target_include_directories(OnTubeVis PUBLIC ${ADDITIONAL_INCLUDES})
target_include_directories(OnTubeVis_static PUBLIC ${ADDITIONAL_INCLUDES})

# Visual Studio fluff
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT OnTubeVis)
