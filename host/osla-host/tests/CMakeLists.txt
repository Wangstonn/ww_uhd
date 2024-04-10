# tests/CMakeLists.txt


# Add executables
add_executable(digitalLoopbackTest digital_loopback.cpp)
target_link_libraries(digitalLoopbackTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

