// Copyright 2026 Apache-2.0
// 图像源发布器：
//  - filename 为图片路径 -> 循环发送该图片（仿真自测）
//  - filename 为纯数字   -> 打开对应 UVC 摄像头（如 "6" 即 /dev/video6，D435i RGB），实时采集
#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_source");

  // camera_index >= 0: 打开 /dev/video<index>（UVC）；image_file: 图片路径（仿真）
  node->declare_parameter<int>("camera_index", -1);
  node->declare_parameter<std::string>("image_file", "");
  node->declare_parameter<std::string>("topic", "/video");
  node->declare_parameter<double>("rate", 30.0);
  node->declare_parameter<int>("width", 1280);
  node->declare_parameter<int>("height", 720);
  node->declare_parameter<int>("fps", 30);

  const int camera_index = node->get_parameter("camera_index").as_int();
  const std::string image_file = node->get_parameter("image_file").as_string();
  const std::string topic = node->get_parameter("topic").as_string();
  const double rate = node->get_parameter("rate").as_double();
  const int width = node->get_parameter("width").as_int();
  const int height = node->get_parameter("height").as_int();
  const int fps = node->get_parameter("fps").as_int();

  cv::Mat img;
  cv::VideoCapture cap;
  bool from_camera = false;

  // 相机模式优先
  if (camera_index >= 0) {
    cap.open(camera_index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(node->get_logger(), "无法打开摄像头 /dev/video%d", camera_index);
      rclcpp::shutdown();
      return 1;
    }
    // D435i RGB 的 UVC 接口只有 YUYV（v4l2-ctl 实测），无 MJPG；
    // <=960x540 档位支持 60fps，720p/1080p 上限 30fps
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // 只留 1 帧缓冲，降低延迟
    from_camera = true;

    // 回读实际协商结果：分辨率/fps 可能与请求不符（驱动回退）
    double actual_fps = cap.get(cv::CAP_PROP_FPS);
    unsigned fourcc = static_cast<unsigned>(cap.get(cv::CAP_PROP_FOURCC));
    char fc[5] = {0};
    if (fourcc) {std::memcpy(fc, &fourcc, 4);}
    RCLCPP_INFO(node->get_logger(),
      "摄像头 /dev/video%d 请求 %dx%d@%d | 实际协商 %dx%d@%.1f fourcc=%s -> %s",
      camera_index, width, height, fps,
      static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
      static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
      actual_fps, fc, topic.c_str());
    if (fps > 30 && (width > 960 || height > 540)) {
      RCLCPP_WARN(node->get_logger(),
        "请求 >30fps 但分辨率超过 960x540：D435i RGB 只有 <=960x540 支持 60fps");
    }
  } else {
    img = cv::imread(image_file);
    if (img.empty()) {
      RCLCPP_ERROR(node->get_logger(), "无法读取图片: %s", image_file.c_str());
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "发送 %s (%dx%d) -> %s @%.1fHz",
      image_file.c_str(), img.cols, img.rows, topic.c_str(), rate);
  }

  auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, 5);
  rclcpp::WallRate loop(from_camera ? 1000.0 : rate);  // 相机模式跟随采集节奏

  // 实测发布帧率统计（每 3 秒打印）
  long fps_count = 0;
  auto fps_t0 = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    if (from_camera) {
      if (!cap.read(img) || img.empty()) {continue;}
    }
    std_msgs::msg::Header header;
    header.stamp = node->now();
    header.frame_id = "camera";
    auto msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    pub->publish(*msg);
    rclcpp::spin_some(node);

    ++fps_count;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - fps_t0).count();
    if (elapsed >= 3.0) {
      RCLCPP_INFO(node->get_logger(), "实际发布帧率: %.1f fps (%ld 帧/%.1fs)",
        fps_count / elapsed, fps_count, elapsed);
      fps_count = 0;
      fps_t0 = now;
    }
    loop.sleep();
  }
  if (cap.isOpened()) {cap.release();}
  rclcpp::shutdown();
  return 0;
}
