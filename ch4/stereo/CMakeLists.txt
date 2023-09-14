project(myStereoVision)

# both of the libraries can be including here or the root cmakelists.txt
# include_directories("usr/include/eigen3")
# set(Pangolin_DIR /usr/local/Pangolin/build)
# find_package(Pangolin REQUIRED)

add_executable(stereoVisionExe stereoVision.cpp)
target_link_libraries(stereoVisionExe ${OpenCV_LIBS} ${Pangolin_LIBRARIES})