
# Build-time configuration
option(
	OTVAPI_EXAMPLE_USE_DLOPEN "Whether the client example should load the OnTubeVis service dynamically at runtime." OFF
)

# The testbed sample client application
add_executable(api_example main.c)

# Determine how the headless service is accessed
if (OTVAPI_EXAMPLE_USE_DLOPEN)
	# We need the system library loading facilities in addition to the include/define setup provided by the OnTubeVis
	# interface target
	target_link_libraries(api_example ${CMAKE_DL_LIBS} OnTubeVis_api)
	target_compile_definitions(api_example PRIVATE "OTVAPI_EXAMPLE_USE_DLOPEN=1" "OTV_NO_PROTOTYPES")
else()
	# Link it in directly
	target_link_libraries(api_example PRIVATE OnTubeVis_svc)
endif()

# Setup required language standard
set_target_properties(api_example PROPERTIES C_STANDARD 11)
set_target_properties(api_example PROPERTIES C_STANDARD_REQUIRED TRUE)

# Make sure the OnTubeVis library (and all other libraries we depend on, for that matter) end up next to our
# api_example build artifact
#add_custom_command(TARGET api_example POST_BUILD
#	COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:api_example> $<TARGET_RUNTIME_DLLS:api_example>
#	COMMAND_EXPAND_LISTS
#)
