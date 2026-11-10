// D435i depth point cloud + optional built-in IMU publisher.
//
// One librealsense pipeline owns the whole camera (the SDK takes exclusive
// access to the USB device, so a separate IMU process fails with
// "Device or resource busy"). Enabling the motion streams here lets
// ground_segment_node use the gravity direction as a plane prior without
// competing for the device.
//
// Publishes:
//   /d435i/points       -> sensor_msgs/PointCloud2   (depth, metres, optical frame)
//   /d435i/accel/sample -> sensor_msgs/Imu           (only when enable_imu)
//   /d435i/gyro/sample  -> sensor_msgs/Imu           (only when enable_imu)

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <librealsense2/rs.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class D435iCloudNode final : public rclcpp::Node {
 public:
  D435iCloudNode() : Node("d435i_cloud") {
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 60);
    max_range_ = declare_parameter<double>("max_range", 3.0);
    serial_ = declare_parameter<std::string>("serial_no", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
    topic_ = declare_parameter<std::string>("cloud_topic", "/d435i/points");
    // D435i 的 IMU 是 BMI085：加速度计 400/200/100 Hz，陀螺仪 400/200 Hz（没有 250）
    enable_imu_ = declare_parameter<bool>("enable_imu", true);
    imu_accel_fps_ = declare_parameter<int>("imu_accel_fps", 200);
    imu_gyro_fps_ = declare_parameter<int>("imu_gyro_fps", 200);
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "camera_imu_optical_frame");
    accel_topic_ = declare_parameter<std::string>("accel_topic", "/d435i/accel/sample");
    gyro_topic_ = declare_parameter<std::string>("gyro_topic", "/d435i/gyro/sample");
    if (width_ < 1 || height_ < 1 || fps_ < 1 || !std::isfinite(max_range_) || max_range_ <= 0.0)
      throw std::invalid_argument("width, height, fps and max_range must be positive");
    if (enable_imu_ && (imu_accel_fps_ < 1 || imu_gyro_fps_ < 1))
      throw std::invalid_argument("imu_accel_fps and imu_gyro_fps must be positive");

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_, rclcpp::SensorDataQoS());
    if (enable_imu_) {
      accel_pub_ = create_publisher<sensor_msgs::msg::Imu>(
          accel_topic_, rclcpp::SensorDataQoS());
      gyro_pub_ = create_publisher<sensor_msgs::msg::Imu>(
          gyro_topic_, rclcpp::SensorDataQoS());
    }

    rs2::config config;
    if (!serial_.empty()) config.enable_device(serial_);
    config.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
    profile_ = pipeline_.start(config);
    RCLCPP_INFO(get_logger(), "D435i depth %dx%d@%d, scale %.6f m/unit, topic %s",
                width_, height_, fps_,
                profile_.get_device().first<rs2::depth_sensor>().get_depth_scale(),
                topic_.c_str());
    if (enable_imu_) start_imu(profile_.get_device());
    timer_ = create_wall_timer(std::chrono::milliseconds(1),
                               std::bind(&D435iCloudNode::capture, this));
  }

  ~D435iCloudNode() override {
    if (!motion_sensor_) return;
    try {
      motion_sensor_.stop();
      motion_sensor_.close();
    } catch (const rs2::error&) {
      // 退出阶段的错误无需处理
    }
  }

 private:
  // 运动传感器单独用回调取流。
  // 若把 accel/gyro 写进 pipeline 的 config，pipeline 内部的 syncer 要凑齐所有流
  // 才吐一个 frameset，200 Hz 的 IMU 会被深度帧拖到 30 Hz 且大量丢帧。
  void start_imu(const rs2::device& dev) {
    for (const rs2::sensor& s : dev.query_sensors()) {
      const auto profiles = s.get_stream_profiles();
      auto pick = [&profiles](rs2_stream type, int want) {
        rs2::stream_profile best;
        int best_fps = -1;
        for (const rs2::stream_profile& p : profiles) {
          const auto motion = p.as<rs2::motion_stream_profile>();
          if (!motion || p.stream_type() != type) continue;
          if (motion.fps() == want) return p;      // 精确命中
          if (motion.fps() > best_fps) {           // 否则记下最高档
            best_fps = motion.fps();
            best = p;
          }
        }
        return best;
      };
      const rs2::stream_profile accel = pick(RS2_STREAM_ACCEL, imu_accel_fps_);
      const rs2::stream_profile gyro = pick(RS2_STREAM_GYRO, imu_gyro_fps_);
      if (!accel || !gyro) continue;

      motion_sensor_ = s;
      motion_sensor_.open({accel, gyro});
      motion_sensor_.start([this](const rs2::frame& f) {
        const auto motion = f.as<rs2::motion_frame>();
        if (motion) publish_imu(motion);
      });
      RCLCPP_INFO(get_logger(), "IMU: accel %d Hz -> %s, gyro %d Hz -> %s",
                  accel.as<rs2::motion_stream_profile>().fps(), accel_topic_.c_str(),
                  gyro.as<rs2::motion_stream_profile>().fps(), gyro_topic_.c_str());
      if (accel.as<rs2::motion_stream_profile>().fps() != imu_accel_fps_ ||
          gyro.as<rs2::motion_stream_profile>().fps() != imu_gyro_fps_) {
        RCLCPP_WARN(get_logger(),
                    "requested IMU %d/%d Hz not available, using %d/%d Hz "
                    "(D435i BMI085: accel 400/200/100, gyro 400/200)",
                    imu_accel_fps_, imu_gyro_fps_,
                    accel.as<rs2::motion_stream_profile>().fps(),
                    gyro.as<rs2::motion_stream_profile>().fps());
      }
      return;
    }
    RCLCPP_WARN(get_logger(), "no motion sensor found, IMU will not be published");
  }

  void capture() {
    rs2::frameset frames;
    if (!pipeline_.poll_for_frames(&frames)) return;
    const auto depth = frames.get_depth_frame();
    if (!depth) return;
    // A depth frame also repeats while the faster motion streams update,
    // so publish the cloud only when the depth frame number advances.
    const auto fnum = depth.get_frame_number();
    if (have_depth_ && fnum == last_depth_fnum_) return;
    last_depth_fnum_ = fnum;
    have_depth_ = true;
    publish_cloud(depth);
  }

  void publish_cloud(const rs2::depth_frame& depth) {
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

  void publish_imu(const rs2::motion_frame& motion) {
    const rs2_stream stream = motion.get_profile().stream_type();
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr* pub = nullptr;
    unsigned long long* last_fnum = nullptr;
    bool* has_last = nullptr;
    if (stream == RS2_STREAM_ACCEL) {
      pub = &accel_pub_;
      last_fnum = &last_accel_fnum_;
      has_last = &has_accel_;
    } else if (stream == RS2_STREAM_GYRO) {
      pub = &gyro_pub_;
      last_fnum = &last_gyro_fnum_;
      has_last = &has_gyro_;
    } else {
      return;
    }
    if (!*pub) return;

    // The pipeline puts the same motion frame into consecutive framesets while
    // the other stream catches up; without this the IMU rate comes out doubled.
    const auto fnum = motion.get_frame_number();
    if (*has_last && fnum == *last_fnum) return;
    *last_fnum = fnum;
    *has_last = true;

    const rs2_vector v = motion.get_motion_data();
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = imu_frame_id_;
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;  // orientation not provided
    if (stream == RS2_STREAM_ACCEL) {
      msg.linear_acceleration.x = v.x;
      msg.linear_acceleration.y = v.y;
      msg.linear_acceleration.z = v.z;
    } else {
      msg.angular_velocity.x = v.x;
      msg.angular_velocity.y = v.y;
      msg.angular_velocity.z = v.z;
    }
    (*pub)->publish(msg);
  }

  int width_, height_, fps_;
  double max_range_;
  bool enable_imu_;
  int imu_accel_fps_, imu_gyro_fps_;
  std::string serial_, frame_id_, topic_, imu_frame_id_, accel_topic_, gyro_topic_;
  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;
  rs2::sensor motion_sensor_;      // IMU 单独开，见 start_imu()
  rs2::pointcloud pointcloud_;
  unsigned long long last_depth_fnum_ = 0, last_accel_fnum_ = 0, last_gyro_fnum_ = 0;
  bool have_depth_ = false, has_accel_ = false, has_gyro_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_pub_, gyro_pub_;
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
