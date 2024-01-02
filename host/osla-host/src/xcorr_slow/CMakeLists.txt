# xcorr_slow/CMakeLists.txt

# Create a library from xcorr sources
add_library(xcorr_slow #create library target. This is added by ../cmakelists.txt
    xcorr_slow.cpp
    xcorr_slow.h
)

# Add xcorr test executable
add_executable(xcorrTest xcorr_slow.test.cpp)

target_link_libraries(xcorrTest xcorr_slow)