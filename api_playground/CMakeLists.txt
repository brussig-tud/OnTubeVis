
# Build-time configuration
option(
	OTVAPI_PLAYGROUND_USE_DLOPEN
	"Whether the 'playground' client should load the headless service dynamically at runtime." OFF
)

# The testbed sample client application
add_executable(api_playground
	main.cxx
	common.h
	layer/properties.h
	testcase/nominal.h
)

target_include_directories(api_playground PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Determine how the headless service is accessed
if (OTVAPI_PLAYGROUND_USE_DLOPEN)
	# We need the system library loading facilities in addition to the include/define setup provided by the OnTubeVis
	# interface target
	target_link_libraries(api_playground ${CMAKE_DL_LIBS} OnTubeVis_api)
	target_compile_definitions(api_playground PRIVATE "OTVAPI_PLAYGROUND_USE_DLOPEN=1" "OTV_NO_PROTOTYPES")
else()
	# Link it in directly
	target_link_libraries(api_playground PRIVATE OnTubeVis_svc)
endif()

# Setup required language standard
set_target_properties(api_playground PROPERTIES C_STANDARD 11)
set_target_properties(api_playground PROPERTIES C_STANDARD_REQUIRED TRUE)
set_target_properties(api_playground PROPERTIES CXX_STANDARD 17)
set_target_properties(api_playground PROPERTIES CXX_STANDARD_REQUIRED ON)
target_compile_features(api_playground PUBLIC cxx_std_17)
