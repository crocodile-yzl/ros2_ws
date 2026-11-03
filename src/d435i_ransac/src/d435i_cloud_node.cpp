#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <librealsense2/rs.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class D435iCloudNode final : public rclcpp::Node {
 public:
  D435iCloudNode() : Node("d435i_cloud") {
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    max_range_ = declare_parameter<double>("max_range", 3.0);
    serial_ = declare_parameter<std::string>("serial_no", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
    topic_ = declare_parameter<std::string>("cloud_topic", "/d435i/points");
    if (width_ < 1 || height_ < 1 || fps_ < 1 || !std::isfinite(max_range_) || max_range_ <= 0.0)
      throw std::invalid_argument("width, height, fps and max_range must be positive");

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_, rclcpp::SensorDataQoS());
    rs2::config config;
    if (!serial_.empty()) config.enable_device(serial_);
    config.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
    profile_ = pipeline_.start(config);
    RCLCPP_INFO(get_logger(), "D435i depth %dx%d@%d, scale %.6f m/unit, topic %s",
                width_, height_, fps_,
                profile_.get_device().first<rs2::depth_sensor>().get_depth_scale(),
                topic_.c_str());
    timer_ = create_wall_timer(std::chrono::milliseconds(1),
                               std::bind(&D435iCloudNode::capture, this));
  }

 private:
  void capture() {
    rs2::frameset frames;
    if (!pipeline_.poll_for_frames(&frames)) return;
    auto depth = frames.get_depth_frame();
    if (!depth) return;
    rs2::points points = pointcloud_.calculate(depth);
    const auto* vertices = points.get_vertices();
    const size_t count = points.size();

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = frame_id_;
    cloud.height = static_cast<uint32_t>(depth.get_height());
    cloud.width = static_cast<uint32_t>(depth.get_width());
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.fields.resize(3);
    const char* names[] = {"x", "y", "z"};
    for (uint32_t i = 0; i < 3; ++i) {
      cloud.fields[i].name = names[i];
      cloud.fields[i].offset = i * 4;
      cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[i].count = 1;
    }
    cloud.data.resize(count * cloud.point_step);
    auto* dst = reinterpret_cast<float*>(cloud.data.data());
    for (size_t i = 0; i < count; ++i) {
      const float z = vertices[i].z;
      if (!std::isfinite(z) || z <= 0.0f || z > max_range_) {
        dst[3 * i] = dst[3 * i + 1] = dst[3 * i + 2] =
            std::numeric_limits<float>::quiet_NaN();
      } else {
        dst[3 * i] = vertices[i].x;
        dst[3 * i + 1] = vertices[i].y;
        dst[3 * i + 2] = z;
      }
    }
    publisher_->publish(std::move(cloud));
  }

  int width_, height_, fps_;
  double max_range_;
  std::string serial_, frame_id_, topic_;
  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;
  rs2::pointcloud pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<D435iCloudNode>());
  } catch (const rs2::error& e) {
    std::fprintf(stderr, "librealsense error: %s (%s, %s)\n", e.what(),
                 e.get_failed_function().c_str(), e.get_failed_args().c_str());
    rclcpp::shutdown();
    return 2;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "D435i node error: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
