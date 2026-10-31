cmake_minimum_required(VERSION 3.8)
project(stick_detector_cpp CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(stick_detector_node src/stick_detector_node.cpp)
target_include_directories(stick_detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(stick_detector_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs)
target_link_libraries(stick_detector_node Eigen3::Eigen)

add_executable(calib_check_node src/calib_check_node.cpp)
ament_target_dependencies(calib_check_node rclcpp sensor_msgs)

add_executable(ground_segment_node src/ground_segment_node.cpp)
target_include_directories(ground_segment_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(ground_segment_node rclcpp sensor_msgs shape_msgs)
target_link_libraries(ground_segment_node Eigen3::Eigen)

add_executable(cloud_sim_node src/cloud_sim_node.cpp)
target_include_directories(cloud_sim_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(cloud_sim_node rclcpp sensor_msgs)
target_link_libraries(cloud_sim_node Eigen3::Eigen)

install(TARGETS stick_detector_node cloud_sim_node calib_check_node ground_segment_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ground_segment_regression
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ground_segment.py
      $<TARGET_FILE:ground_segment_node>)
  set_tests_properties(ground_segment_regression PROPERTIES TIMEOUT 45)
endif()

ament_package()
