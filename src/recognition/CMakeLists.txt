cmake_minimum_required(VERSION 3.8)
project(recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(recognition_component SHARED
  src/recognition_node.cpp
)
target_include_directories(recognition_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(recognition_component PUBLIC cxx_std_17)
ament_target_dependencies(recognition_component
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(recognition_component ${OpenCV_LIBS})
rclcpp_components_register_node(recognition_component
  PLUGIN "recognition::RecognitionNode"
  EXECUTABLE recognition_node
)

add_executable(recognition_main src/main.cpp)
target_include_directories(recognition_main PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(recognition_main recognition_component)

add_executable(sim_image_publisher src/sim_image_publisher.cpp)
target_compile_features(sim_image_publisher PUBLIC cxx_std_17)
ament_target_dependencies(sim_image_publisher rclcpp sensor_msgs cv_bridge)
target_link_libraries(sim_image_publisher ${OpenCV_LIBS})

install(TARGETS recognition_component recognition_main sim_image_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
