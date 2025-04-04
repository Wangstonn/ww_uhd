# tests/CMakeLists.txt
configure_file( ${CMAKE_BINARY_DIR}/src/estim/preamble.mem preamble.mem COPYONLY)

# Add executables
add_executable(InterfEstim interf_estim.cpp)
target_link_libraries(InterfEstim ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)