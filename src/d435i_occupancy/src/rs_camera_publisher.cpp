// Copyright 2026 Apache-2.0
// D435i 原生 librealsense 采集节点（绕开 realsense2_camera ROS wrapper 的帧同步坑）
// 发布：
//   /camera/depth/image_rect_raw  16UC1 毫米深度
//   /camera/depth/camera_info     深度内参（由 SDK 标定值生成）
//   /camera/color/image_raw       rgb8 彩色（可选，默认 848x480@60）
//   /camera/color/camera_info
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstring>
#include <memory>

class RsCameraPublisher : public rclcpp::Node
{
public:
  RsCameraPublisher() : Node("rs_camera_publisher")
  {
    serial_ = declare_parameter<std::string>("serial_number", "");
    depth_width_ = declare_parameter<int>("depth_width", 480);
    depth_height_ = declare_parameter<int>("depth_height", 270);
    depth_fps_ = declare_parameter<int>("depth_fps", 60);
    enable_color_ = declare_parameter<bool>("enable_color", true);
    color_width_ = declare_parameter<int>("color_width", 848);
    color_height_ = declare_parameter<int>("color_height", 480);
    color_fps_ = declare_parameter<int>("color_fps", 60);
    enable_ir_ = declare_parameter<bool>("enable_ir", false);
    // 深度后处理滤波全部默认关闭：降延迟、省 CPU（按 realsense 惯例可单独打开）
    temporal_ = declare_parameter<bool>("temporal_filter", false);
    spatial_ = declare_parameter<bool>("spatial_filter", false);
    hole_filling_ = declare_parameter<bool>("hole_filling_filter", false);

    if (depth_width_ <= 0 || depth_height_ <= 0 || depth_fps_ <= 0) {
      throw std::invalid_argument("invalid depth resolution/fps");
    }
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/depth/image_rect_raw", rclcpp::SensorDataQoS());
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/depth/camera_info", rclcpp::SensorDataQoS());
    if (enable_color_) {
      color_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::SensorDataQoS());
      color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", rclcpp::SensorDataQoS());
    }
  }

private:
  static sensor_msgs::msg::CameraInfo make_info(
    const rs2::video_stream_profile & profile, const std::string & frame_id)
  {
    const auto i = profile.get_intrinsics();
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id;
    info.width = i.width;
    info.height = i.height;
    info.distortion_model = "plumb_bob";
    info.d = {i.coeffs[0], i.coeffs[1], i.coeffs[2], i.coeffs[3], i.coeffs[4]};
    info.k = {i.fx, 0., i.ppx, 0., i.fy, i.ppy, 0., 0., 1.};
    info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    info.p = {i.fx, 0., i.ppx, 0., 0., i.fy, i.ppy, 0., 0., 0., 1., 0.};
    return info;
  }

  static sensor_msgs::msg::Image make_image(
    const rs2::video_frame & frame, const std::string & encoding,
    const std::string & frame_id, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.height = frame.get_height();
    msg.width = frame.get_width();
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(
      frame.get_stride_in_bytes());
    const auto * src = static_cast<const uint8_t *>(frame.get_data());
    msg.data.assign(src, src + msg.step * msg.height);
    return msg;
  }

public:
  // 必须在 make_shared 完成后调用（内部使用 shared_from_this）
  void run()
  {
    rs2::config cfg;
    if (!serial_.empty()) {cfg.enable_device(serial_);}
    cfg.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_,
      RS2_FORMAT_Z16, depth_fps_);
    if (enable_color_) {
      cfg.enable_stream(RS2_STREAM_COLOR, color_width_, color_height_,
        RS2_FORMAT_RGB8, color_fps_);
    }
    if (enable_ir_) {
      cfg.enable_stream(RS2_STREAM_INFRARED, 1, depth_width_, depth_height_,
        RS2_FORMAT_Y8, depth_fps_);
    }
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);  // 抛 rs2::error 时由 main 捕获
    auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH)
      .as<rs2::video_stream_profile>();
    const rs2_intrinsics depth_intr = depth_profile.get_intrinsics();
    RCLCPP_INFO(get_logger(),
      "D435i 已开流: depth %dx%d@%d fx=%.1f fy=%.1f (%s), color %s",
      depth_intr.width, depth_intr.height, depth_fps_, depth_intr.fx,
      depth_intr.fy, profile.get_device().get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR),
      enable_color_ ? (std::to_string(color_width_) + "x" +
        std::to_string(color_height_) + "@" + std::to_string(color_fps_)).c_str()
      : "off");

    rs2::temporal_filter tf;
    rs2::spatial_filter sf;
    rs2::hole_filling_filter hf;
    const std::string depth_frame_id = "camera_depth_optical_frame";
    const std::string color_frame_id = "camera_color_optical_frame";

    long count = 0;
    auto fps_t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      auto frames = pipe.wait_for_frames();  // 阻塞跟随相机节奏
      const auto stamp = now();
      for (const auto & f : frames) {
        if (f.is<rs2::depth_frame>()) {
          rs2::frame d = f;
          if (temporal_) {d = tf.process(d);}
          if (spatial_) {d = sf.process(d);}
          if (hole_filling_) {d = hf.process(d);}
          auto depth = d.as<rs2::depth_frame>();
          auto info = make_info(depth.get_profile().as<rs2::video_stream_profile>(),
            depth_frame_id);
          info.header.stamp = stamp;
          depth_pub_->publish(make_image(depth, "16UC1", depth_frame_id, stamp));
          depth_info_pub_->publish(info);
        } else if (enable_color_ && f.is<rs2::video_frame>() &&
          f.get_profile().stream_type() == RS2_STREAM_COLOR)
        {
          auto color = f.as<rs2::video_frame>();
          auto info = make_info(color.get_profile().as<rs2::video_stream_profile>(),
            color_frame_id);
          info.header.stamp = stamp;
          color_pub_->publish(make_image(color, "rgb8", color_frame_id, stamp));
          color_info_pub_->publish(info);
        }
      }
      rclcpp::spin_some(shared_from_this());
      ++count;
      auto t = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(t - fps_t0).count();
      if (elapsed >= 3.0) {
        RCLCPP_INFO(get_logger(), "实际帧率: %.1f fps (%ld 帧/%.1fs)",
          count / elapsed, count, elapsed);
        count = 0;
        fps_t0 = t;
      }
    }
    pipe.stop();
  }

  std::string serial_;
  int depth_width_, depth_height_, depth_fps_;
  int color_width_, color_height_, color_fps_;
  bool enable_color_, enable_ir_, temporal_, spatial_, hole_filling_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_, color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_,
    color_info_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RsCameraPublisher>();
    node->run();
  } catch (const rs2::error & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rs_camera_publisher"),
      "librealsense 错误: %s", e.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rs_camera_publisher"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
