cmake_minimum_required(VERSION 3.0)
project(ch6)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(ch6_code)


