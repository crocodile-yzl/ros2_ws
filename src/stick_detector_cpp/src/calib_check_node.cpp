// Depth <-> point cloud consistency calibration.
//
// Compares /depth/depth_raw (16UC1, millimetres) with the z of the organized
// point cloud pixel-by-pixel. If the projection / scale is wrong, the ratio
// z_cloud / depth is not 1.0 and/or varies across the image.
//
//   ratio = z_cloud / (depth_mm * 0.001)
//   - global median ~ 1.0      : conversion is consistent
//   - global median != 1.0     : systematic scale error -> apply cloud_scale
//   - strong tile-to-tile trend: intrinsics / lens distortion issue
//
// Prints a 4x4 grid of per-tile medians and a suggested cloud_scale.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace {

float median(std::vector<float>& v) {
  if (v.empty()) return 0.f;
  const size_t k = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

}  // namespace

class CalibCheckNode : public rclcpp::Node {
 public:
  CalibCheckNode() : Node("calib_check") {
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth/depth_raw");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/berxel_cloudpoint");
    tiles_ = declare_parameter<int>("tiles", 4);
    report_every_ = declare_parameter<int>("report_frames", 8);
    max_dt_ = declare_parameter<double>("max_dt", 0.15);

    auto qos = rclcpp::SensorDataQoS();
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos,
        std::bind(&CalibCheckNode::onCloud, this, std::placeholders::_1));
    sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, qos,
        std::bind(&CalibCheckNode::onDepth, this, std::placeholders::_1));
    tile_ratios_.resize(tiles_ * tiles_);
    RCLCPP_INFO(get_logger(),
                "calib_check up: %s + %s (tiles=%d, every %d frames)",
                cloud_topic_.c_str(), depth_topic_.c_str(), tiles_, report_every_);
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_cloud_ = msg;
    have_cloud_ = true;
  }

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!have_cloud_) return;
    if (msg->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "depth encoding is %s, expected 16UC1",
                           msg->encoding.c_str());
      return;
    }
    const auto& cloud = *last_cloud_;
    const int w = static_cast<int>(msg->width), h = static_cast<int>(msg->height);
    const double dt = std::fabs(
        (rclcpp::Time(msg->header.stamp) - rclcpp::Time(cloud.header.stamp)).seconds());
    if (static_cast<int>(cloud.width) != w || static_cast<int>(cloud.height) != h) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "size mismatch: depth=%dx%d enc=%s  vs  cloud=%ux%u (frame %s vs %s)",
          w, h, msg->encoding.c_str(), cloud.width, cloud.height,
          msg->header.frame_id.c_str(), cloud.header.frame_id.c_str());
      return;
    }
    if (dt > max_dt_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "stamp gap %.3fs > %.2f (depth %u.%09u vs cloud %u.%09u)", dt, max_dt_,
          msg->header.stamp.sec, msg->header.stamp.nanosec,
          cloud.header.stamp.sec, cloud.header.stamp.nanosec);
      return;
    }

    const uint16_t* depth = reinterpret_cast<const uint16_t*>(msg->data.data());
    const int step = static_cast<int>(cloud.point_step);
    const uint8_t* base = cloud.data.data();
    int oz = -1;
    for (const auto& f : cloud.fields)
      if (f.name == "z" && f.datatype == sensor_msgs::msg::PointField::FLOAT32)
        oz = static_cast<int>(f.offset);
    if (oz < 0) return;

    const int T = tiles_;
    std::vector<std::vector<float>> local(T * T);
    for (int v = 0; v < h; v += 2) {
      for (int u = 0; u < w; u += 2) {
        const uint16_t d = depth[static_cast<size_t>(v) * w + u];
        if (d == 0) continue;
        float z;
        std::memcpy(&z, base + (static_cast<size_t>(v) * w + u) * step + oz, 4);
        if (!std::isfinite(z) || z <= 0.01f) continue;
        const float dm = static_cast<float>(d) * 0.001f;
        const float r = z / dm;
        if (r < 0.2f || r > 5.0f) continue;
        const int tile = (v * T / h) * T + (u * T / w);
        local[tile].push_back(r);
      }
    }
    size_t valid_pairs = 0;
    for (int t = 0; t < T * T; ++t) {
      auto& dst = tile_ratios_[t];
      dst.insert(dst.end(), local[t].begin(), local[t].end());
      valid_pairs += local[t].size();
    }
    if (++frames_ % report_every_ == 0)
      report(valid_pairs);
    else if (valid_pairs == 0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "no valid pixel pairs this frame (depth zeros or cloud NaN)");

  }

  void report(size_t valid_pairs) {
    (void)valid_pairs;
    std::vector<float> all;
    std::string out = "\n";
    char line[256];
    double sum = 0.0;
    int cnt = 0;
    for (int r = 0; r < tiles_; ++r) {
      std::string row;
      for (int c = 0; c < tiles_; ++c) {
        auto& v = tile_ratios_[r * tiles_ + c];
        if (v.empty()) {
          row += "   --- ";
          continue;
        }
        const float m = median(v);
        row += "  ";
        char cell[24];
        std::snprintf(cell, sizeof(cell), "%.3f", m);
        row += cell;
        all.insert(all.end(), v.begin(), v.end());
        sum += m;
        ++cnt;
      }
      out += "    " + row + "\n";
    }
    if (all.empty()) {
      RCLCPP_INFO(get_logger(), "[calib] no valid depth/cloud pairs yet");
      return;
    }
    const float g = median(all);
    std::snprintf(line, sizeof(line),
                  "[calib] global ratio z_cloud/depth = %.4f (n=%zu px), "
                  "suggested cloud_scale = %.4f\n%s",
                  g, all.size(), g > 1e-6f ? 1.0f / g : 1.0f, out.c_str());
    RCLCPP_INFO(get_logger(), "%s", line);
    for (auto& v : tile_ratios_) v.clear();
  }

  std::string depth_topic_, cloud_topic_;
  int tiles_ = 4, report_every_ = 8;
  double max_dt_ = 0.15;
  int frames_ = 0;
  bool have_cloud_ = false;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  std::vector<std::vector<float>> tile_ratios_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibCheckNode>());
  rclcpp::shutdown();
  return 0;
}
