# estim/CMakeLists.txt

# Create a library from estim sources
add_library(estim #create library target. This is added by ../cmakelists.txt
    estim.cpp
    estim.h
)

# Add the preamble to the build directory
configure_file(preamble.mem ${CMAKE_BINARY_DIR}/src/estim/preamble.mem COPYONLY)

# Add test executable
add_executable(upsampleTest upsample.test.cpp)
#target_link_libraries(mmioTest ${UHD_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(upsampleTest estim mmio)

add_executable(ch_estimTest ch_estim.test.cpp)
target_link_libraries(ch_estimTest ${UHD_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(ch_estimTest estim mmio)