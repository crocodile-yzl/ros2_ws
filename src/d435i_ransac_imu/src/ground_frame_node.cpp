#include <chrono>
#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/plane.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "d435i_ransac_imu/ground_transform.hpp"

class GroundFrameNode : public rclcpp::Node {
 public:
  GroundFrameNode() : Node("ground_visualization_frame") {
    const double height = declare_parameter<double>("camera_height", 0.9);
    const double pitch = declare_parameter<double>("camera_pitch_deg", 34.0) *
                         std::acos(-1.0) / 180.0;
    if (!d435i_ransac_imu::ground_transform(
            Eigen::Vector3d(0.0, std::cos(pitch), std::sin(pitch)), -height,
            camera_to_ground_)) {
      throw std::invalid_argument("Invalid camera height/pitch for ground visualization");
    }
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<shape_msgs::msg::Plane>(
        "ground/plane", rclcpp::QoS(5),
        [this](const shape_msgs::msg::Plane::SharedPtr plane) {
          Eigen::Isometry3d candidate;
          if (d435i_ransac_imu::ground_transform(
                  Eigen::Vector3d(plane->coef[0], plane->coef[1], plane->coef[2]),
                  plane->coef[3], candidate)) {
            camera_to_ground_ = candidate;
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Ignoring invalid ground plane for visualization");
          }
        });
    timer_ = create_wall_timer(std::chrono::nanoseconds(33333333), [this]() {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now();
      tf.header.frame_id = "d435i_ground";
      tf.child_frame_id = "camera_depth_optical_frame";
      tf.transform.translation.z = camera_to_ground_.translation().z();
      const Eigen::Quaterniond rotation(camera_to_ground_.linear());
      tf.transform.rotation.x = rotation.x();
      tf.transform.rotation.y = rotation.y();
      tf.transform.rotation.z = rotation.z();
      tf.transform.rotation.w = rotation.w();
      broadcaster_->sendTransform(tf);
    });
  }
 private:
  Eigen::Isometry3d camera_to_ground_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<shape_msgs::msg::Plane>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFrameNode>());
  rclcpp::shutdown();
  return 0;
}
