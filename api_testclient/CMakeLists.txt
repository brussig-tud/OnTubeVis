
# Build-time configuration
option(
	OTVAPI_CLIENT_USE_DLOPEN "Whether the test client should load the headless service dynamically at runtime." OFF
)

# The testbed sample client application
add_executable(api_testclient main.c)

# Determine how the headless service is accessed
if (OTVAPI_CLIENT_USE_DLOPEN)
	# We need the system library loading facilities in addition to the include/define setup provided by the OnTubeVis
	# interface target
	target_link_libraries(api_testclient ${CMAKE_DL_LIBS} OnTubeVis_api)
	target_compile_definitions(api_testclient PRIVATE "OTVAPI_CLIENT_USE_DLOPEN=1" "OTV_NO_PROTOTYPES")
else()
	# Link it in directly
	target_link_libraries(api_testclient PRIVATE OnTubeVis_svc)
endif()

# Setup required language standard
set_target_properties(api_testclient PROPERTIES C_STANDARD 11)
set_target_properties(api_testclient PROPERTIES C_STANDARD_REQUIRED TRUE)

# Make sure the OnTubeVis library (and all other libraries we depend on, for that matter) end up next to our
# api_testclient build artifact
#add_custom_command(TARGET api_testclient POST_BUILD
#	COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:api_testclient> $<TARGET_RUNTIME_DLLS:api_testclient>
#	COMMAND_EXPAND_LISTS
#)
