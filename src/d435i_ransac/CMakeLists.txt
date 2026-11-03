cmake_minimum_required(VERSION 3.8)
project(d435i_ransac CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(ground_segment_node src/ground_segment_node.cpp)
ament_target_dependencies(ground_segment_node rclcpp sensor_msgs shape_msgs)
target_link_libraries(ground_segment_node Eigen3::Eigen)

add_executable(d435i_cloud_node src/d435i_cloud_node.cpp)
ament_target_dependencies(d435i_cloud_node rclcpp sensor_msgs)
target_link_libraries(d435i_cloud_node realsense2::realsense2)

add_executable(ground_frame_node src/ground_frame_node.cpp)
ament_target_dependencies(ground_frame_node rclcpp shape_msgs geometry_msgs tf2_ros)
target_link_libraries(ground_frame_node Eigen3::Eigen)
target_include_directories(ground_frame_node PRIVATE include)

install(TARGETS ground_segment_node d435i_cloud_node ground_frame_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  add_executable(test_ground_transform test/test_ground_transform.cpp)
  target_link_libraries(test_ground_transform Eigen3::Eigen)
  target_include_directories(test_ground_transform PRIVATE include)
  add_test(NAME ground_transform_regression COMMAND test_ground_transform)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ground_segment_regression
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ground_segment.py
      $<TARGET_FILE:ground_segment_node>)
  set_tests_properties(ground_segment_regression PROPERTIES TIMEOUT 60)
  add_test(NAME tilted_ground_regression
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_tilted_ground.py
      $<TARGET_FILE:ground_segment_node>)
endif()

ament_package()
