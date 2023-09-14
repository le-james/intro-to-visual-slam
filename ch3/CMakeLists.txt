cmake_minimum_required(VERSION 3.0)
project(basic_Sophus)

# used to get variables of the cmakelist
get_directory_property(vars_before VARIABLES)


# config mode tries to find the <package>config.cmake in the path
set(Sophus_DIR "/usr/local/Sophus/build")
# message(${Sophus_DIR})

# find_package() using CONFIG mode which looks for SophusConfig.cmake in the build folder
# to find the file in the build folder, find_package() needs to set a variable <package_DIR> to the path
find_package(Sophus CONFIG REQUIRED)
# or
# find_package(Sophus REQUIRED)

# the compiler searches for the header files here when building
include_directories("/usr/include/eigen3")

add_executable(basicSophus basicSophus.cpp)
# target_link_libraries(basicSophus "/usr/local/Sophus")
# or
# read about the double colon using: cmake --help-policy CMP0028
# does it this way since sophues doesn't provide a Sophus_LIBRARY variable
# it's an import
target_link_libraries(basicSophus Sophus::Sophus)



# # used to get variables of the cmakelist
# get_directory_property(vars VARIABLES)
# # removes contents of vars_before and ${vars_before} from vars
# list(REMOVE_ITEM vars vars_before ${vars_before})

# foreach(var IN LISTS vars)
#     message(STATUS "${var} = ${${var}}")
# endforeach()
