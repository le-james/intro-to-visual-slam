cmake_minimum_required(VERSION 3.0)
project(openCV_examples)

# opencv library
find_package(OpenCV REQUIRED)

# pangolin library
set(Pangolin_DIR /usr/local/Pangolin/build)
find_package(Pangolin REQUIRED)

# eigen library
include_directories("/usr/include/eigen3")

# adds the folder imageBasics to the build folder
add_subdirectory(imageBasics)
add_subdirectory(stereo)

