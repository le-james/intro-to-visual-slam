project(ch6_visual_odometry_code)

add_executable(orb_cv_Exe orb_cv.cpp)
target_link_libraries(orb_cv_Exe ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d_Exe pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d_Exe ${OpenCV_LIBS})

add_executable(triangulation_Exe triangulation.cpp)
target_link_libraries(triangulation_Exe ${OpenCV_LIBS})
