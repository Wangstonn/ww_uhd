# tests/CMakeLists.txt
configure_file( ${CMAKE_BINARY_DIR}/src/estim/preamble.mem preamble.mem COPYONLY)

# Add executables

# # digital test------------------------------------------------------------
# add_executable(dlbTest dlb_test.cpp)
# target_link_libraries(dlbTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# # add_executable(dlbPrmblStream dlb_prmbl_stream.cpp)
# # target_link_libraries(dlbPrmblStream ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# add_executable(dlbPrmblCap dlb_prmbl_cap.cpp)
# target_link_libraries(dlbPrmblCap ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# #fwd test------------------------------------------------------------------------
add_executable(fwdAlbEstim fwd_alb_estim.cpp)
target_link_libraries(fwdAlbEstim ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# add_executable(fwdAlbCap fwd_alb_cap.cpp)
# target_link_libraries(fwdAlbCap ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# add_executable(fwdAlbDebug fwd_alb_debug.cpp)
# target_link_libraries(fwdAlbDebug ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(fwdAlbTest fwd_alb_test.cpp)
target_link_libraries(fwdAlbTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# # fb tests-----------------------------------------------------------------

# add_executable(fbAlbCap fb_alb_cap.cpp)
# target_link_libraries(fbAlbCap ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# add_executable(fbAlbTest fb_alb_test.cpp)
# target_link_libraries(fbAlbTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)


# p2p tests
# add_executable(SrcDestEstim src_dest_estim.cpp)
# target_link_libraries(SrcDestEstim ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# # add_executable(SrcDestTest src_dest_test.cpp)
# # target_link_libraries(SrcDestTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(P2pAwgnEstim p2p_awgn_estim.cpp)
target_link_libraries(P2pAwgnEstim ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(P2pAwgnTest p2p_awgn_test.cpp)
target_link_libraries(P2pAwgnTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(P2pIntfEstim p2p_intf_estim.cpp)
target_link_libraries(P2pIntfEstim ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

add_executable(P2pIntfTest p2p_intf_test.cpp)
target_link_libraries(P2pIntfTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# add_executable(ThresholdBerTest threshold_ber_test.cpp)
# target_link_libraries(ThresholdBerTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)

# rss estimation
add_executable(RssEstim rss_estim.cpp)
target_link_libraries(RssEstim ${UHD_LIBRARIES} ${Boost_LIBRARIES} mmio estim)