# tests/CMakeLists.txt
configure_file( ${CMAKE_BINARY_DIR}/src/estim/preamble.mem preamble.mem COPYONLY)

# Add executables
add_executable(digitalLoopbackTest digital_loopback.cpp)
target_link_libraries(digitalLoopbackTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# add_executable(dlbPrmblStream dlb_prmbl_stream.cpp)
# target_link_libraries(dlbPrmblStream ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(dlbPrmblCap dlb_prmbl_cap.cpp)
target_link_libraries(dlbPrmblCap ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(fwdAlbPrmblCap fwd_alb_prmbl_cap.cpp)
target_link_libraries(fwdAlbPrmblCap ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(fwdAlbCap fwd_alb_cap.cpp)
target_link_libraries(fwdAlbCap ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(fwdAlbDebug fwd_alb_debug.cpp)
target_link_libraries(fwdAlbDebug ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(fwdLoopbackTest fwd_loopback.cpp)
target_link_libraries(fwdLoopbackTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)


