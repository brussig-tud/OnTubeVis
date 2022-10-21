
add_library(OnTubeVis lib_begin.h traj_loader.h traj_loader.cxx regulargrid.h regulargrid.cxx bezdat_handler.h bezdat_handler.cxx csv_handler.h csv_handler.cxx tubes.cxx)

target_link_libraries(OnTubeVis
	cgv_utils cgv_type cgv_data cgv_base cgv_math
	cgv_media cgv_gui cgv_render glew crg_stereo_view)

target_include_directories(OnTubeVis PUBLIC
	$<BUILD_INTERFACE:${CGV_DIR}/libs>
	$<BUILD_INTERFACE:${CGV_DIR}/plugins/crg_stereo_view>)

target_compile_definitions(OnTubeVis PRIVATE TUBES_EXPORTS)

add_dependencies(OnTubeVis
	cgv_viewer
	cg_fltk
	cgv_gl
	cg_ext
	crg_light
	crg_stereo_view)
