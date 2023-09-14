cmake_minimum_required(VERSION 2.8)
project(chapter3)

# checking variables
get_directory_property(_vars_before VARIABLES)

include_directories("/usr/include/eigen3")

# for some reason pangolin didn't need set() in /usr/include/ folder but in /usr/local/ it does
set(Pangolin_DIR /usr/local/Pangolin/build)
find_package(Pangolin REQUIRED)

# "Pangolin_INCLUDE_DIRS" doesn't seem to do anything
# include_directories(${Pangolin_INCLUDE_DIRS})
# message("PANGOLIN_INCLUDE_DIRS: ${Pangolin_INCLUDE_DIRS}")

# not needed? it works without it
# find_package(Eigen3 REQUIRED)

add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
# below does not work
# read about the double colon using: cmake --help-policy CMP0028


add_executable(eigenGeometry eigenGeometry.cpp)
add_executable(eigenMatrix eigenMatrix.cpp)

message("Pangolin_LIBRARIES: ${Pangolin_LIBRARIES}")

# checking variables
get_directory_property(_vars VARIABLES)
list(REMOVE_ITEM _vars _vars_before ${_vars_before})
foreach(_var IN LISTS _vars)
    message(STATUS "${_var} = ${${_var}}")
endforeach()