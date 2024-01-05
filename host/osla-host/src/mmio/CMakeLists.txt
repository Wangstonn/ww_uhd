# xcorr_slow/CMakeLists.txt

# Create a library from xcorr sources
add_library(mmio #create library target. This is added by ../cmakelists.txt
    mmio.cpp
    mmio.h
)

# Add xcorr test executable
#add_executable(mmioTest mmio.test.cpp)

#target_link_libraries(mmioTest mmio)