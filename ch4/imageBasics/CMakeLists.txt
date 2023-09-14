project(myImageBasics)

# PROJECT_NAME == myImageBasics
message("my projects name is: ${PROJECT_NAME}")

# the name of the executable is imageBasics
add_executable(imageBasicsExe imageBasics.cpp)
# since imageBasics.cpp uses the opencv library, need to link the libraries
target_link_libraries(imageBasicsExe ${OpenCV_LIBS})

add_executable(undistortImageExe undistortImage.cpp)
target_link_libraries(undistortImageExe ${OpenCV_LIBS})
