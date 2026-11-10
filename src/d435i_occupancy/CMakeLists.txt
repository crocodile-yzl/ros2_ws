cmake_minimum_required(VERSION 3.8)
project(d435i_occupancy)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# 系统 librealsense2（dpkg 安装 2.58.4），非 ROS 包
find_library(REALSENSE2_LIBRARY realsense2 REQUIRED)
find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp REQUIRED)

add_library(voxel_map src/voxel_map.cpp)
target_include_directories(voxel_map PUBLIC include)

# librealsense 原生采集节点：深度 480x270@60（可调至90），发布 Image+CameraInfo
add_executable(rs_camera_publisher src/rs_camera_publisher.cpp)
target_link_libraries(rs_camera_publisher ${REALSENSE2_LIBRARY})
target_include_directories(rs_camera_publisher PRIVATE ${REALSENSE2_INCLUDE_DIR})
ament_target_dependencies(rs_camera_publisher rclcpp sensor_msgs)

# 占据建图节点（与 berxel_occupancy 同算法，换默认话题）
add_executable(occupancy_node src/occupancy_node.cpp)
target_link_libraries(occupancy_node voxel_map)
ament_target_dependencies(occupancy_node rclcpp sensor_msgs nav_msgs std_msgs visualization_msgs)

install(TARGETS occupancy_node rs_camera_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
 enable_testing()
 add_executable(test_voxel_map test/test_voxel_map.cpp)
 target_link_libraries(test_voxel_map voxel_map)
 add_test(NAME voxel_map COMMAND test_voxel_map)
endif()
ament_package()
