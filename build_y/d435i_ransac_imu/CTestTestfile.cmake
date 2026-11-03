# CMake generated Testfile for 
# Source directory: /home/y/ros2_ws/src/d435i_ransac_imu
# Build directory: /home/y/ros2_ws/build_y/d435i_ransac_imu
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ground_transform_regression "/home/y/ros2_ws/build_y/d435i_ransac_imu/test_ground_transform")
set_tests_properties(ground_transform_regression PROPERTIES  _BACKTRACE_TRIPLES "/home/y/ros2_ws/src/d435i_ransac_imu/CMakeLists.txt;38;add_test;/home/y/ros2_ws/src/d435i_ransac_imu/CMakeLists.txt;0;")
add_test(ground_segment_regression "/usr/bin/python3" "/home/y/ros2_ws/src/d435i_ransac_imu/test/test_ground_segment.py" "/home/y/ros2_ws/build_y/d435i_ransac_imu/ground_segment_node")
set_tests_properties(ground_segment_regression PROPERTIES  TIMEOUT "60" _BACKTRACE_TRIPLES "/home/y/ros2_ws/src/d435i_ransac_imu/CMakeLists.txt;40;add_test;/home/y/ros2_ws/src/d435i_ransac_imu/CMakeLists.txt;0;")
add_test(tilted_ground_regression "/usr/bin/python3" "/home/y/ros2_ws/src/d435i_ransac_imu/test/test_tilted_ground.py" "/home/y/ros2_ws/build_y/d435i_ransac_imu/ground_segment_node")
set_tests_properties(tilted_ground_regression PROPERTIES  _BACKTRACE_TRIPLES "/home/y/ros2_ws/src/d435i_ransac_imu/CMakeLists.txt;44;add_test;/home/y/ros2_ws/src/d435i_ransac_imu/CMakeLists.txt;0;")
