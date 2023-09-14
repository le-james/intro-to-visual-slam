project(ch5_optimization_code)

add_executable(gaussNewtonExe gaussNewton.cpp)
add_executable(ceresCurveFittingExe ceresCurveFitting.cpp)
add_executable(g2oCurveFittingExe g2oCurveFitting.cpp)

target_link_libraries(gaussNewtonExe ${OpenCV_LIBS})
target_link_libraries(ceresCurveFittingExe ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(g2oCurveFittingExe ${OpenCV_LIBS} glog::glog ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})