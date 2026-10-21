cmake_minimum_required(VERSION 3.10)
project(d435i_video_c C CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(d435i_video_c src/d435i_video_c.c)

target_include_directories(d435i_video_c PRIVATE
  ${rcl_INCLUDE_DIRS}
  ${rosidl_runtime_c_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

if(TARGET rcl::rcl)
  set(RCL_LINK_TARGETS rcl::rcl)
else()
  set(RCL_LINK_TARGETS ${rcl_LIBRARIES})
endif()

target_link_libraries(d435i_video_c
  ${RCL_LINK_TARGETS}
  ${sensor_msgs_TARGETS}
  realsense2::realsense2
)

# 源码是 C；链接的 rosidl/rcl 依赖里有 C++ 运行时，交给 C++ 链接器收尾
set_target_properties(d435i_video_c PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS d435i_video_c DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
