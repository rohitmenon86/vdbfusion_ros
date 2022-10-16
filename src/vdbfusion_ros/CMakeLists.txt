add_library(transforms STATIC Transform.cpp)
target_link_libraries(transforms PUBLIC
  sophus::sophus
)
target_include_directories(transforms PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node VDBVolume_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${catkin_LIBRARIES}
  VDBFusion::vdbfusion
  igl::core
  transforms
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${CMAKE_SOURCE_DIR}/include/vdbfusion_ros
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
