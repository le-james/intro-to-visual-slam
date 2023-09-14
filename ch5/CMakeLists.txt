cmake_minimum_required(VERSION 3.0)
project(ch5)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

# loads the variables and other stuff
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# CMAKE_MODULE_PATH is the path for CMake modules to be loaded by the include() or find_package()
# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH "/usr/local/g2o/cmake_modules")
# g2o uses MODULE mode which needs a file called Find<package>.cmake to find variables
find_package(G2O MODULE REQUIRED)
# PROJECT_SOURCE_DIR = /home/james/intro_to_vslam/ch5
message(project source dir: ${PROJECT_SOURCE_DIR})

# the compiler searches for the header files here when building
include_directories("/usr/local/include/eigen")
include_directories("/usr/include/eigen3")
include_directories(${G2O_INCLUDE_DIRS})

add_subdirectory(ch5_code)
