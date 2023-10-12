cmake_minimum_required(VERSION 3.24)

project(OnTubeVis)

# preset CGV Framework options
option(CGV_BUILD_WITH_AUDIO OFF)
option(CGV_BUILD_EXAMPLES OFF)

# OnTubeVis options
set(ENABLE_OPTIX_SUPPORT_DOC "Build with support for hardware raytracing using NVIDIA OptiX")
option(ENABLE_OPTIX_SUPPORT ${ENABLE_OPTIX_SUPPORT_DOC} ON)
option(ENABLE_HOLOGRAPHY
	"Use the experimental Holo-View-Interactor of the CGV Framework for holography-compatible camera management (suitable e.g. for Looking Glass™ displays)"
	OFF
)

# Fetch the CGV Framework if it's not obviously present on the system
if (DEFINED ENV{CGV_DIR})
	set(CGV_DIR $ENV{CGV_DIR})
	message("Using CGV Framework source found at:")
	message("> ${CGV_DIR}")
	add_subdirectory(${CGV_DIR} ./framework)
else()
	include(FetchContent)
	FetchContent_Declare(
		cgv GIT_REPOSITORY https://github.com/sgumhold/cgv.git
		GIT_TAG c299debfefb96fdda2ec8278ec74b3c11f9d3098
	)
	FetchContent_MakeAvailable(cgv)
endif()

# check optional dependencies
# - include additional find scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# - dependencies needed for hardware raytracing
if (ENABLE_OPTIX_SUPPORT)
	find_package(CUDA)
	find_package(CUDAToolkit)
	find_package(OptiX)
endif()
# - other optional dependencies
find_package(OpenMP)

# handle optional dependencies
set(OPTIONAL_DEPENDENCIES "")
if (CUDA_FOUND AND CUDAToolkit_FOUND AND OptiX_INCLUDE AND NOT OptiX_INCLUDE STREQUAL "OptiX_INCLUDE-NOTFOUND")
	add_subdirectory(tool/cuda_resprep) # build the cuda_resprep custom tool
	list(APPEND OPTIONAL_DEPENDENCIES CUDA::cudart CUDA::nvrtc)
	list(APPEND FEATURE_DEFINES "RTX_SUPPORT=1")
	set(ADDITIONAL_INCLUDE_PATHS ${OptiX_INCLUDE}) #  ${CUDA_TOOLKIT_INCLUDE}
	set(NVRTC_INCLUDE_DEFS
		"CUDA_RUNTIME_COMPILER__INC_DIR_CUDA=\"${CUDA_TOOLKIT_INCLUDE}\""
		"CUDA_RUNTIME_COMPILER__INC_DIR_OPTIX=\"${OptiX_INCLUDE}\""
		"CUDA_RUNTIME_COMPILER__INC_DIR_OPTIXSDK=\"${OptiX_INSTALL_DIR}/SDK\""
		"CUDA_RUNTIME_COMPILER__INC_DIR_OPTIXSDK_CUDA=\"${OptiX_INSTALL_DIR}/SDK/cuda\""
	)
else()
	set(ENABLE_OPTIX_SUPPORT OFF CACHE BOOL ${ENABLE_OPTIX_SUPPORT_DOC} FORCE)
endif()
if (OpenMP_CXX_FOUND)
	list(APPEND OPTIONAL_DEPENDENCIES OpenMP::OpenMP_CXX)
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
	layer_config_io.h
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
	glsl/color_maps.glfs
	glsl/color_maps.glpr
	glsl/curve_tools.glsl
	glsl/optix_display.glfs
	glsl/optix_display.glpr
	glsl/rcc.glsl
	glsl/surface_mod.glsl
	glsl/textured_line_primitive.glvs
	glsl/textured_spline_ribbon.glfs
	glsl/textured_spline_ribbon.glgs
	glsl/textured_spline_ribbon.glpr
	glsl/textured_spline_tube.glfs
	glsl/textured_spline_tube.glgs
	glsl/textured_spline_tube.glpr
	glsl/textured_spline_tube_shading.glfs
	glsl/textured_spline_tube_shading.glpr
	glsl/transform.glsl
	glsl/view_aligned_ribbon.glfs
	glsl/view_aligned_ribbon.glgs
	glsl/view_aligned_ribbon.glpr
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

# decide on the view interactor to use
if (ENABLE_HOLOGRAPHY)
	set(VIEW_INTERACTOR_PLUGIN crg_holo_view)
else()
	set(VIEW_INTERACTOR_PLUGIN crg_stereo_view)
endif()

# create the OnTubeVis plugin within the CGV Framework build infrastructure
cgv_add_target(OnTubeVis
	TYPE plugin
	SOURCES         ${SOURCES}
	HEADERS         ${HEADERS}
	SHADER_SOURCES  ${SHADERS}
	RESOURCES       ${IMAGES}
	DEPENDENCIES
		glew json fltk cgv_viewer cgv_utils cgv_type cgv_reflect cgv_data cgv_signal cgv_base cgv_media cgv_gui cgv_render
		cgv_os cg_fltk cgv_gl cg_ext cmi_io cg_msdf ${VIEW_INTERACTOR_PLUGIN} crg_light crg_grid cgv_g2d cgv_app cgv_post cgv_gpgpu
		${OPTIONAL_DEPENDENCIES} 

	LINKTIME_PLUGIN_DEPENDENCIES cg_fltk  # <-- we're directly using exported symbols from the cg_fltk GUI provider plugin

	ADDITIONAL_INCLUDE_PATHS ${ADDITIONAL_INCLUDE_PATHS}

	ADDITIONAL_CMDLINE_ARGS
		"config:\"${CMAKE_CURRENT_LIST_DIR}/config.def\""

	OVERRIDE_SHARED_EXPORT_DEFINE TUBES_EXPORTS

	ADDITIONAL_PRIVATE_DEFINES ${NVRTC_INCLUDE_DEFS}
)

if (ENABLE_OPTIX_SUPPORT)
	# add CUDA sources to single-exec build (CUDA sources are compiled during runtime by the plugin build)
	# - compile include path for nvcc pointing to CUDA and OptiX SDKs
	set(NVCC_INCLUDE_PATH
		"-I\"${CUDA_TOOLKIT_INCLUDE}\"" "-I\"${OptiX_INCLUDE}\"" "-I\"${OptiX_INSTALL_DIR}/SDK\""
		"-I\"${OptiX_INSTALL_DIR}/SDK/cuda\"")
	# - add the CUDA source files and build rule
	cgv_add_custom_sources(OnTubeVis STATIC
		SOURCES
			cuda/optix_curves_builtin2.cu cuda/optix_curves_builtin3.cu cuda/optix_curves_phantom.cu
			cuda/optix_curves_russig.cu
		HEADERS
			cuda/optix_interface.h cuda/optix_curves_impl.cuh cuda/optix_isect_phantom.cuh
			cuda/optix_isect_russig.cuh cuda/optix_tools.cuh

		OUTFILE_TEMPLATE "<<<FN>>>_<<<EXT>>>.cpp"

		BUILD_SUBDIR     "cuda"
		BUILD_TOOL       cuda_resprep
		BUILD_TOOL_ARGS
			"${CUDA_NVCC_EXECUTABLE}" "<<<INFILE>>>" "<<<OUTFILE>>>" "-std=c++17" "-ptx" "-arch=compute_61"
			"--use_fast_math" "-O=3" "-dopt=on" "-D__x86_64" "-I\"<<<INFILE_PATH>>>\"" ${NVCC_INCLUDE_PATH}
	)
endif()

# schedule high-level warning messages at the very end of the configuration process to make sure they will be easily
# visible
# - no OpenMP
if (NOT OpenMP_CXX_FOUND)
	cmake_language(
		DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL message
		"WARNING: OpenMP development files not found on the system!\n"
		"         Preprocessing for large datasets will be slow."
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
