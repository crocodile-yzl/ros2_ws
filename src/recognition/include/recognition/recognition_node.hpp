// Copyright 2026 Apache-2.0
#ifndef RECOGNITION__RECOGNITION_NODE_HPP_
#define RECOGNITION__RECOGNITION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace recognition
{

// 每目标 HSV 区间：以实拍图标像素的 HSV 中值为中心的兼容范围
// （标定方法：读出图标像素中值 H/S/V，再给 H±10 / S±30~45 / V±40~60 容差）
struct TargetCfg
{
  std::string name;
  int h_low = 0, h_high = 179;
  int s_low = 0, s_high = 255;
  int v_low = 0, v_high = 255;
  bool use_blackhat = false;   // true=白底灰图标流水线（blackhat+核心修剪+白环验证）
  double min_fill = 0.30;      // 每目标填充率下限
  std::string shape = "none";  // 形状验证：none | cross(X型) | eight(倒8：腐蚀分两竖瓣)
};

class RecognitionNode : public rclcpp::Node
{
public:
  explicit RecognitionNode(const rclcpp::NodeOptions & options);

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);

  // 单目标颜色分割（支持 H 环形区间），返回合并后的候选框（无面积阈值）
  std::vector<cv::Rect> detectColor(
    const cv::Mat & hsv, const TargetCfg & cfg, int merge_gap) const;

  // 白底灰图标检测：blackhat 取暗结构 -> 核心修剪 -> 形状/饱和度/白环验证
  std::vector<cv::Rect> detectGrayOnWhite(
    const cv::Mat & hsv, const TargetCfg & cfg, int merge_gap) const;

  std::string image_topic_;
  std::string debug_dir_;
  long debug_count_ = 0;
  bool publish_annotated_;
  double relative_min_ratio_;
  std::vector<TargetCfg> targets_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_;
};

}  // namespace recognition

#endif  // RECOGNITION__RECOGNITION_NODE_HPP_
