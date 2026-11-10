// Copyright 2026 Apache-2.0
#include <rclcpp/rclcpp.hpp>
#include <recognition/recognition_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<recognition::RecognitionNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
