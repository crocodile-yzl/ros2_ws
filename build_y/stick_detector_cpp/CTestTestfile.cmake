# CMake generated Testfile for 
# Source directory: /home/y/ros2_ws/src/stick_detector_cpp
# Build directory: /home/y/ros2_ws/build_y/stick_detector_cpp
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ground_segment_regression "/usr/bin/python3" "/home/y/ros2_ws/src/stick_detector_cpp/test/test_ground_segment.py" "/home/y/ros2_ws/build_y/stick_detector_cpp/ground_segment_node")
set_tests_properties(ground_segment_regression PROPERTIES  TIMEOUT "45" _BACKTRACE_TRIPLES "/home/y/ros2_ws/src/stick_detector_cpp/CMakeLists.txt;49;add_test;/home/y/ros2_ws/src/stick_detector_cpp/CMakeLists.txt;0;")
