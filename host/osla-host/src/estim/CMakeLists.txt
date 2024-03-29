# xcorr_slow/CMakeLists.txt

# Create a library from xcorr sources
add_library(estim #create library target. This is added by ../cmakelists.txt
    estim.cpp
    estim.h
)



# Add test executable
add_executable(upsampleTest upsample.test.cpp)
#target_link_libraries(mmioTest ${UHD_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(upsampleTest estim mmio)