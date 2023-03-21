
# create the tool executable target
add_executable(cuda_resprep cuda_resprep.cxx)

# setup compilation
target_compile_definitions(cuda_resprep PRIVATE "CGV_FORCE_STATIC")
target_compile_features(cuda_resprep PRIVATE cxx_std_17)
set_target_properties(cuda_resprep PROPERTIES CXX_STANDARD 17)
set_target_properties(cuda_resprep PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(cuda_resprep PROPERTIES CXX_EXTENSIONS ON)
