# tests/CMakeLists.txt
configure_file( ${CMAKE_BINARY_DIR}/src/estim/preamble.mem preamble.mem COPYONLY)

# Add executables
add_executable(digitalLoopbackTest digital_loopback.cpp)
target_link_libraries(digitalLoopbackTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(fwdLoopbackTest fwd_loopback.cpp)
target_link_libraries(fwdLoopbackTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(fwdLoopbackCap fwd_loopback_cap.cpp)
target_link_libraries(fwdLoopbackCap ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

