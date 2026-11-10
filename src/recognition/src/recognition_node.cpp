// Copyright 2026 Apache-2.0
// 识别节点（多目标版）：对 /video 彩色图做 HSV 颜色分割 + 形状过滤，
// 支持多个目标（每目标独立 HSV 区间/最小面积），输出
// Detection2DArray（class_id=目标名）与标注后的可视化图像。
//
// 两类检测流水线：
//  1. 颜色模式（默认）：inRange(H,S,V) + 形态学 + 连通域 + 填充率过滤 + 近邻合并
//  2. 白底灰图标模式（tools 等）：图标灰色与货架背景同色域，颜色法不可分。
//     借鉴 stick_detector 的"核心修剪"思路：blackhat 提取白底上的暗结构，
//     7x7 开运算只留粗笔画（文字/细纹被剪掉），再按面积/填充率/宽高比/
//     中性灰(S中位数)/白衬底环带 逐级验证。
// 无 cv2/scipy 依赖，纯 C++/OpenCV 实现。
#include <recognition/recognition_node.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace recognition
{

namespace
{
// 黑帽流水线常量（白底灰图标）
constexpr int kBlackhatKernel = 31;     // 局部背景尺度
constexpr int kBlackhatThresh = 25;     // 暗于局部背景的判别阈值
constexpr int kCoreKernel = 7;          // 核心修剪：笔画粗于该核才保留
constexpr double kCoreAspectMin = 0.6, kCoreAspectMax = 1.6;
constexpr int kCoreSMax = 30;           // 中性灰：S 中位数上限
constexpr double kWhiteRingMin = 0.15;  // 图标外扩环带白像素占比下限
constexpr int kRingPad = 20;

// 标注框颜色表（BGR）：红/绿/蓝/黑/橙/紫 轮换
const cv::Scalar kPalette[] = {
  {0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 0, 0},
  {0, 128, 255}, {255, 0, 255}};

std::vector<cv::Rect> mergeNearby(std::vector<cv::Rect> boxes, int gap)
{
  bool changed = true;
  while (changed && boxes.size() > 1) {
    changed = false;
    for (size_t i = 0; i < boxes.size() && !changed; ++i) {
      for (size_t j = i + 1; j < boxes.size() && !changed; ++j) {
        const cv::Rect ri = boxes[i];
        const cv::Rect rj = boxes[j];
        const bool near = (ri & cv::Rect(rj.x - gap, rj.y - gap,
          rj.width + 2 * gap, rj.height + 2 * gap)).area() > 0;
        if (near) {
          boxes[i] = ri | rj;
          boxes.erase(boxes.begin() + j);
          changed = true;
        }
      }
    }
  }
  return boxes;
}

// 同帧同类相对过滤：丢弃明显小于同类最大框的候选。
// 纯相对量，透视/距离变化时尺度自动约掉，不依赖绝对面积阈值。
std::vector<cv::Rect> relativeFilter(std::vector<cv::Rect> boxes, double ratio)
{
  if (boxes.empty() || ratio <= 0.0) {return boxes;}
  double max_area = 0.0;
  for (const auto & b : boxes) {
    max_area = std::max(max_area, static_cast<double>(b.area()));
  }
  const double min_area = ratio * max_area;
  boxes.erase(
    std::remove_if(boxes.begin(), boxes.end(),
      [min_area](const cv::Rect & b) {
        return static_cast<double>(b.area()) < min_area;
      }), boxes.end());
  return boxes;
}

// X 型验证（工具）：3x3 网格里中心格（两笔画交叉点）填充率最高，
// 且四角平均 >= 四边中平均（实心块/竖条/横条都不满足）
bool shapeCross(const cv::Mat & mask, const cv::Rect & box)
{
  const cv::Mat m = mask(box);
  const int rh = m.rows / 3, rw = m.cols / 3;
  if (rh <= 0 || rw <= 0) {return false;}
  double cell[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cell[i][j] = cv::mean(m(cv::Rect(j * rw, i * rh,
        j == 2 ? m.cols - 2 * rw : rw,
        i == 2 ? m.rows - 2 * rh : rh)))[0] / 255.0;
    }
  }
  double center = cell[1][1];
  double other_max = 0.0;
  double corners = 0.0, edges = 0.0;
  const int ci[3][4] = {{0, 0, 2, 2}, {0, 2, 0, 2}};   // 四角
  const int ei[3][4] = {{0, 1, 1, 2}, {1, 0, 2, 1}};   // 四边中
  for (int t = 0; t < 4; ++t) {
    corners += cell[ci[0][t]][ci[1][t]];
    edges += cell[ei[0][t]][ei[1][t]];
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == 1 && j == 1) {continue;}
      other_max = std::max(other_max, cell[i][j]);
    }
  }
  return center + 0.05 >= other_max && corners / 4.0 >= edges / 4.0;
}

// 倒 8 验证（仪器）：腐蚀后颈部断开成两瓣，且两瓣质心纵向分布。
// 紧凑块（蓝夹子等）腐蚀到消失也只有一块，被拒绝。
// 颈宽随曝光波动，腐蚀迭代扫 1..3，任一层分出 >=2 瓣即通过。
bool shapeEight(const cv::Mat & mask, const cv::Rect & box)
{
  const cv::Mat m = mask(box);
  const double base = cv::countNonZero(m);
  if (base < 100) {return false;}
  const cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
  for (int it = 1; it <= 3; ++it) {
    cv::Mat e;
    cv::erode(m, e, k, cv::Point(-1, -1), it);
    // 用像素计数（connectedComponentsWithStats）而非 contourArea：
    // 细长瓣的轮廓面积会被低估
    cv::Mat labels, stats, cents_mat;
    int n_labels = cv::connectedComponentsWithStats(e, labels, stats, cents_mat, 8);
    std::vector<cv::Point2d> cents;
    for (int i = 1; i < n_labels; ++i) {
      const double a = stats.at<int>(i, cv::CC_STAT_AREA);
      if (a < 0.03 * base) {continue;}
      cents.emplace_back(cents_mat.at<double>(i, 0), cents_mat.at<double>(i, 1));
    }
    if (cents.size() >= 2) {
      // 两瓣质心分离距离与目标尺寸成比例（透视不变）；贴纸悬挂弯曲/倾斜时
      // 倒8可能旋转（∞），方向不限
      const double min_sep = 0.3 * std::sqrt(base);
      for (size_t i = 0; i < cents.size(); ++i) {
        for (size_t j = i + 1; j < cents.size(); ++j) {
          const double dx = cents[i].x - cents[j].x;
          const double dy = cents[i].y - cents[j].y;
          if (std::sqrt(dx * dx + dy * dy) >= min_sep) {return true;}
        }
      }
    }
  }
  return false;
}
}  // namespace

RecognitionNode::RecognitionNode(const rclcpp::NodeOptions & options)
: Node("recognition_node", options)
{
  declare_parameter<std::string>("image_topic", "/video");
  declare_parameter<bool>("publish_annotated", true);
  declare_parameter<double>("min_fill_ratio", 0.30);
  // 每目标填充率下限（数组，缺省用 min_fill_ratio）：
  // 仪器框因双烧瓶+反光合并后 fill 只有 ~0.26，其把关者是倒8形状验证
  declare_parameter<std::vector<double>>("min_fills", {});
  declare_parameter<int>("merge_gap", 12);
  // 同帧同类相对过滤（0=禁用）：透视不变，替代绝对面积阈值
  declare_parameter<double>("relative_min_ratio", 0.20);
  // 调试：非空时每 30 帧保存一张标注图到该目录
  declare_parameter<std::string>("debug_dir", "");

  // 多目标参数（数组，按下标一一对应）
  declare_parameter<std::vector<std::string>>("target_names",
    std::vector<std::string>{"food"});
  declare_parameter<std::vector<int64_t>>("h_low", {42});
  declare_parameter<std::vector<int64_t>>("h_high", {58});
  declare_parameter<std::vector<int64_t>>("s_low", {35});
  declare_parameter<std::vector<int64_t>>("s_high", {255});
  declare_parameter<std::vector<int64_t>>("v_low", {70});
  declare_parameter<std::vector<int64_t>>("v_high", {255});
  declare_parameter<std::vector<bool>>("use_blackhat", {false});
  declare_parameter<std::vector<std::string>>("shape", {"none"});

  image_topic_ = get_parameter("image_topic").as_string();
  publish_annotated_ = get_parameter("publish_annotated").as_bool();
  relative_min_ratio_ = get_parameter("relative_min_ratio").as_double();
  debug_dir_ = get_parameter("debug_dir").as_string();

  auto names = get_parameter("target_names").as_string_array();
  auto h_lo = get_parameter("h_low").as_integer_array();
  auto h_hi = get_parameter("h_high").as_integer_array();
  auto s_lo = get_parameter("s_low").as_integer_array();
  auto s_hi = get_parameter("s_high").as_integer_array();
  auto v_lo = get_parameter("v_low").as_integer_array();
  auto v_hi = get_parameter("v_high").as_integer_array();
  auto blackhat = get_parameter("use_blackhat").as_bool_array();
  auto shapes = get_parameter("shape").as_string_array();
  auto fills = get_parameter("min_fills").as_double_array();
  const double fill_default = get_parameter("min_fill_ratio").as_double();

  const size_t n = names.size();
  auto at = [](const auto & v, size_t i, auto def) {
      return v.empty() ? def : v[std::min(i, v.size() - 1)];
    };
  for (size_t i = 0; i < n; ++i) {
    TargetCfg c;
    c.name = names[i];
    c.h_low = static_cast<int>(at(h_lo, i, int64_t{0}));
    c.h_high = static_cast<int>(at(h_hi, i, int64_t{179}));
    c.s_low = static_cast<int>(at(s_lo, i, int64_t{0}));
    c.s_high = static_cast<int>(at(s_hi, i, int64_t{255}));
    c.v_low = static_cast<int>(at(v_lo, i, int64_t{0}));
    c.v_high = static_cast<int>(at(v_hi, i, int64_t{255}));
    c.use_blackhat = at(blackhat, i, false);
    c.shape = at(shapes, i, std::string("none"));
    c.min_fill = at(fills, i, fill_default);
    targets_.push_back(c);
  }

  pub_detections_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "/recognition/detections", 10);
  if (publish_annotated_) {
    pub_annotated_ = create_publisher<sensor_msgs::msg::Image>(
      "/recognition/annotated", 10);
  }

  rclcpp::SubscriptionOptions sub_opt;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, qos,
    std::bind(&RecognitionNode::onImage, this, std::placeholders::_1),
    sub_opt);

  std::string log = "recognition_node ready: topic=" + image_topic_ + " targets:";
  for (const auto & t : targets_) {
    log += " " + t.name + (t.use_blackhat ? "[gray-on-white]" : "[h" +
      std::to_string(t.h_low) + "-" + std::to_string(t.h_high) + "]");
  }
  RCLCPP_INFO(get_logger(), "%s", log.c_str());
}

std::vector<cv::Rect> RecognitionNode::detectColor(
  const cv::Mat & hsv, const TargetCfg & cfg, int merge_gap) const
{
  cv::Mat mask;
  if (cfg.h_low <= cfg.h_high) {
    cv::inRange(hsv, cv::Scalar(cfg.h_low, cfg.s_low, cfg.v_low),
      cv::Scalar(cfg.h_high, cfg.s_high, cfg.v_high), mask);
  } else {  // H 环形区间（如红色跨 0）
    cv::Mat m1, m2;
    cv::inRange(hsv, cv::Scalar(0, cfg.s_low, cfg.v_low),
      cv::Scalar(cfg.h_high, cfg.s_high, cfg.v_high), m1);
    cv::inRange(hsv, cv::Scalar(cfg.h_low, cfg.s_low, cfg.v_low),
      cv::Scalar(179, cfg.s_high, cfg.v_high), m2);
    cv::bitwise_or(m1, m2, mask);
  }
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::Rect> boxes;
  for (const auto & c : contours) {
    const cv::Rect box = cv::boundingRect(c);
    const double area = cv::contourArea(c);
    if (area / static_cast<double>(box.area()) < cfg.min_fill) {continue;}
    // 形状验证放在合并前：单目标的倒8特征（颈+瓶身）比合并大框稳定；
    // 小反光块(base<100)和紧凑块（蓝夹子）在这里被自然排除
    if (cfg.shape == "eight" && !shapeEight(mask, box)) {continue;}
    boxes.push_back(box);
  }
  return relativeFilter(mergeNearby(boxes, merge_gap), relative_min_ratio_);
}

std::vector<cv::Rect> RecognitionNode::detectGrayOnWhite(
  const cv::Mat & hsv, const TargetCfg & cfg, int merge_gap) const
{
  std::vector<cv::Mat> ch;
  cv::split(hsv, ch);
  const cv::Mat & S = ch[1];
  const cv::Mat & V = ch[2];

  // 1. blackhat：白底上的暗结构（大片均匀灰区自动归零）
  const cv::Mat kbig = cv::getStructuringElement(
    cv::MORPH_RECT, {kBlackhatKernel, kBlackhatKernel});
  cv::Mat bh;
  cv::morphologyEx(V, bh, cv::MORPH_BLACKHAT, kbig);
  cv::Mat mask;
  cv::threshold(bh, mask, kBlackhatThresh, 255, cv::THRESH_BINARY);
  cv::Mat sat_hi;
  cv::inRange(S, cv::Scalar(80), cv::Scalar(255), sat_hi);
  mask.setTo(0, sat_hi);  // 只留低饱和暗结构

  const cv::Mat k5 = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
  const cv::Mat k7 = cv::getStructuringElement(cv::MORPH_RECT, {kCoreKernel, kCoreKernel});
  const cv::Mat k9 = cv::getStructuringElement(cv::MORPH_RECT, {9, 9});
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k5);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k7);
  // 2. 核心修剪：只留粗笔画（文字/细纹被腐蚀掉）
  cv::Mat core = mask.clone();
  cv::morphologyEx(core, core, cv::MORPH_OPEN, k7);
  cv::morphologyEx(core, core, cv::MORPH_CLOSE, k9);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(core, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::Mat white;
  cv::inRange(hsv, cv::Scalar(0, 0, 185), cv::Scalar(179, 70, 255), white);

  std::vector<cv::Rect> boxes;
  for (const auto & c : contours) {
    const cv::Rect box = cv::boundingRect(c);
    const double fill = cv::contourArea(c) / static_cast<double>(box.area());
    const double aspect = static_cast<double>(box.width) / box.height;
    if (fill < cfg.min_fill) {continue;}
    if (aspect < kCoreAspectMin || aspect > kCoreAspectMax) {continue;}
    // 3. 中性灰验证：图标 S 中位数要低（烤鸡带绿色调被排除）
    const double s_median = cv::mean(S(box), core(box))[0];
    if (s_median >= kCoreSMax) {continue;}
    // 4. 白衬底验证：外扩环带须有足量白像素（贴纸白纸）
    const cv::Rect ext(std::max(0, box.x - kRingPad),
      std::max(0, box.y - kRingPad),
      std::min(hsv.cols, box.x + box.width + kRingPad) - std::max(0, box.x - kRingPad),
      std::min(hsv.rows, box.y + box.height + kRingPad) - std::max(0, box.y - kRingPad));
    cv::Mat ring = cv::Mat::ones(ext.size(), CV_8U);
    cv::Rect inner(box.x - ext.x, box.y - ext.y, box.width, box.height);
    ring(inner).setTo(0);
    const double white_frac = cv::mean(white(ext), ring)[0] / 255.0;
    if (white_frac < kWhiteRingMin) {continue;}
    if (cfg.shape == "cross" && !shapeCross(core, box)) {continue;}
    boxes.push_back(box);
  }
  return relativeFilter(mergeNearby(boxes, merge_gap), relative_min_ratio_);
}

void RecognitionNode::onImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat bgr;
  try {
    bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "cv_bridge error: %s", e.what());
    return;
  }
  if (bgr.empty()) {return;}

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  const int merge_gap = get_parameter("merge_gap").as_int();

  vision_msgs::msg::Detection2DArray out;
  out.header = msg->header;

  cv::Mat annotated;
  if (publish_annotated_) {
    annotated = bgr.clone();
  }

  size_t target_idx = 0;
  for (const auto & cfg : targets_) {
    std::vector<cv::Rect> boxes = cfg.use_blackhat ?
      detectGrayOnWhite(hsv, cfg, merge_gap) :
      detectColor(hsv, cfg, merge_gap);

    const cv::Scalar & color = kPalette[target_idx % 6];
    for (const auto & box : boxes) {
      vision_msgs::msg::Detection2D det;
      det.header = msg->header;
      det.bbox.center.position.x = box.x + box.width / 2.0;
      det.bbox.center.position.y = box.y + box.height / 2.0;
      det.bbox.size_x = box.width;
      det.bbox.size_y = box.height;
      det.results.resize(1);
      det.results[0].hypothesis.class_id = cfg.name;
      det.results[0].hypothesis.score = 1.0f;
      out.detections.push_back(det);

      if (publish_annotated_) {
        cv::rectangle(annotated, box, color, 2);
        cv::putText(annotated, cfg.name,
          {box.x, std::max(0, box.y - 6)},
          cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
      }
    }
    ++target_idx;
  }

  if (publish_annotated_ && out.detections.empty()) {
    cv::putText(annotated, "no target", {20, 40},
      cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);
  }

  pub_detections_->publish(out);
  if (publish_annotated_) {
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
    pub_annotated_->publish(*out_msg);
    if (!debug_dir_.empty() && ++debug_count_ % 30 == 0) {
      cv::imwrite(debug_dir_ + "/annotated.png", annotated);
    }
  }
}

}  // namespace recognition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(recognition::RecognitionNode)
