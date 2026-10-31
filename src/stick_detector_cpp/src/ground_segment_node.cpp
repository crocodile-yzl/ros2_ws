// Standalone ground segmentation node.
//
// Fits a ground plane from an organized or unorganized depth point cloud and
// publishes:
//   /ground/plane        -> shape_msgs/Plane (ax + by + cz + d = 0)
//   /ground/ground_cloud -> PointCloud2 (points on the plane)
//   /ground/non_ground   -> PointCloud2 (everything above the plane;
//                           all valid points while no plane is available)
//
// Two modes (same as stick_detector):
//   ground_plane_mode = auto   -> RANSAC fit, refreshed every N frames
//   ground_plane_mode = manual -> normal from camera_height + camera_pitch_deg
//
// The fitted normal is sanity-checked: it must be within max_slope of the
// camera's expected ground normal (from camera_pitch_deg), so a wall can not
// be mistaken for the floor.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "shape_msgs/msg/plane.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

using Vec3 = Eigen::Vector3f;

namespace {
struct Plane {
  Vec3 n = Vec3(0.f, 0.f, 1.f);
  float d = 0.f;
};

bool ransac_plane(const std::vector<Vec3>& pts, float thresh, int iters,
                  int sample, const Vec3& expected_normal, float min_cos,
                  float min_inlier_ratio,
                  std::mt19937& rng, Plane& out) {
  if (pts.size() < 100) return false;
  std::vector<Vec3> s;
  if (static_cast<int>(pts.size()) > sample) {
    s.reserve(sample);
    std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
    for (int i = 0; i < sample; ++i) s.push_back(pts[pick(rng)]);
  } else {
    s = pts;
  }
  const int m = static_cast<int>(s.size());
  const int stop_cnt = static_cast<int>(0.85f * m);
  int best_cnt = -1;
  Vec3 best_n = Vec3::Zero();
  float best_d = 0.f;
  std::uniform_int_distribution<int> pick3(0, m - 1);
  for (int it = 0; it < iters; ++it) {
    const Vec3& p0 = s[pick3(rng)];
    const Vec3& p1 = s[pick3(rng)];
    const Vec3& p2 = s[pick3(rng)];
    Vec3 n = (p1 - p0).cross(p2 - p0);
    const float norm = n.norm();
    if (norm < 1e-9f) continue;
    n /= norm;
    // A dominant wall must not hide a smaller ground plane. Score only
    // hypotheses compatible with the camera's expected ground direction.
    if (std::fabs(n.dot(expected_normal)) < min_cos) continue;
    if (n.dot(expected_normal) < 0.f) n = -n;
    const float d = n.dot(p0);
    // With a downward-facing normal, ground is on the positive side of the
    // camera origin. Reject ceilings and other surfaces above the camera.
    if (d <= 0.f) continue;
    int cnt = 0;
    for (int i = 0; i < m; ++i)
      if (std::fabs(n.dot(s[i]) - d) < thresh) ++cnt;
    if (cnt > best_cnt) {
      best_cnt = cnt;
      best_n = n;
      best_d = d;
      if (cnt >= stop_cnt) break;
    }
  }
  const int min_inliers = std::max(50, static_cast<int>(std::ceil(min_inlier_ratio * m)));
  if (best_cnt < min_inliers) return false;
  std::vector<Vec3> inl;
  inl.reserve(best_cnt);
  for (const auto& p : s)
    if (std::fabs(best_n.dot(p) - best_d) < thresh) inl.push_back(p);
  Vec3 c = Vec3::Zero();
  for (const auto& p : inl) c += p;
  c /= static_cast<float>(inl.size());
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : inl) cov += (p - c) * (p - c).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
  if (es.info() != Eigen::Success) return false;
  out.n = es.eigenvectors().col(0);       // smallest eigenvalue -> normal
  out.d = out.n.dot(c);
  if (out.n.dot(expected_normal) < 0.f) {
    out.n = -out.n;
    out.d = -out.d;
  }
  // Refinement can change the direction, so check the final normal too.
  if (out.n.dot(expected_normal) < min_cos || out.d <= 0.f) return false;
  int refined_cnt = 0;
  for (const auto& p : s)
    if (std::fabs(out.n.dot(p) - out.d) < thresh) ++refined_cnt;
  if (refined_cnt < min_inliers) return false;
  return true;
}

inline void append_xyz(std::vector<float>& buf, const Vec3& p) {
  buf.push_back(p.x());
  buf.push_back(p.y());
  buf.push_back(p.z());
}
}  // namespace

class GroundSegmentNode : public rclcpp::Node {
 public:
  GroundSegmentNode() : Node("ground_segmentation") {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/berxel_cloudpoint");
    mode_ = declare_parameter<std::string>("ground_plane_mode", "auto");
    camera_height_ = declare_parameter<double>("camera_height", 0.80);
    camera_pitch_deg_ = declare_parameter<double>("camera_pitch_deg", 0.0);
    thresh_ = declare_parameter<double>("ground_thresh", 0.02);
    iters_ = declare_parameter<int>("ground_iters", 1000);
    sample_ = declare_parameter<int>("ground_sample", 4000);
    refresh_ = declare_parameter<int>("ground_refresh", 30);
    max_slope_deg_ = declare_parameter<double>("max_slope_deg", 15.0);
    min_inlier_ratio_ = declare_parameter<double>("ground_min_inlier_ratio", 0.10);
    max_range_ = declare_parameter<double>("max_range", 3.0);
    stride_ = declare_parameter<int>("downsample_stride", 2);

    if ((mode_ != "auto" && mode_ != "manual") || refresh_ < 1 ||
        iters_ < 1 || sample_ < 100 || stride_ < 1 ||
        !std::isfinite(thresh_) || thresh_ <= 0.0 ||
        !std::isfinite(min_inlier_ratio_) || min_inlier_ratio_ <= 0.0 ||
        min_inlier_ratio_ > 1.0) {
      throw std::invalid_argument("Invalid ground segmentation parameters");
    }

    if (mode_ == "manual") {
      const double p = camera_pitch_deg_ * M_PI / 180.0;
      plane_.n = Vec3(0.f, static_cast<float>(std::cos(p)),
                      static_cast<float>(std::sin(p)));
      plane_.d = static_cast<float>(camera_height_);
      have_ = true;
    }

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos,
        std::bind(&GroundSegmentNode::onCloud, this, std::placeholders::_1));
    pub_plane_ = create_publisher<shape_msgs::msg::Plane>(
        "ground/plane", rclcpp::QoS(5));
    // RViz defaults to reliable subscriptions. A reliable publisher also
    // accepts best-effort subscribers, so both display settings can connect.
    const auto output_qos = rclcpp::QoS(5).reliable();
    pub_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "ground/ground_cloud", output_qos);
    pub_nonground_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "ground/non_ground", output_qos);
    RCLCPP_INFO(get_logger(), "ground_segmentation up (mode=%s)", mode_.c_str());
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    int ox = -1, oy = -1, oz = -1;
    for (const auto& f : msg->fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
      if (f.name == "x") ox = static_cast<int>(f.offset);
      else if (f.name == "y") oy = static_cast<int>(f.offset);
      else if (f.name == "z") oz = static_cast<int>(f.offset);
    }
    if (ox < 0 || oy < 0 || oz < 0 ||
        static_cast<size_t>(std::max({ox, oy, oz})) + sizeof(float) >
            msg->point_step) return;

    const int w = static_cast<int>(msg->width);
    const int h = static_cast<int>(msg->height);
    const int step = static_cast<int>(msg->point_step);
    const size_t row_step = msg->row_step;
    if (w <= 0 || h <= 0 || step <= 0 ||
        row_step < static_cast<size_t>(w) * step ||
        msg->data.size() < row_step * static_cast<size_t>(h)) {
      return;
    }

    const int s = std::max(1, stride_);
    std::vector<Vec3> pts;
    pts.reserve(static_cast<size_t>(w / s + 1) * (h / s + 1));
    const uint8_t* base = msg->data.data();
    for (int r = 0; r < h; r += s) {
      const uint8_t* row = base + static_cast<size_t>(r) * row_step;
      for (int c = 0; c < w; c += s) {
        const uint8_t* p = row + static_cast<size_t>(c) * step;
        float x, y, z;
        std::memcpy(&x, p + ox, 4);
        std::memcpy(&y, p + oy, 4);
        std::memcpy(&z, p + oz, 4);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
          continue;
        if (z <= 0.05f || z >= static_cast<float>(max_range_)) continue;
        pts.emplace_back(x, y, z);
      }
    }

    // estimate / refresh plane
    if (mode_ == "auto") {
      ++frame_;
      if (!have_ || (frame_ - 1) % refresh_ == 0) {
        Plane p;
        const double pitch = camera_pitch_deg_ * M_PI / 180.0;
        const Vec3 expected_normal(0.f, static_cast<float>(std::cos(pitch)),
                                   static_cast<float>(std::sin(pitch)));
        const float min_cos = static_cast<float>(
            std::cos(max_slope_deg_ * M_PI / 180.0));
        if (ransac_plane(pts, static_cast<float>(thresh_), iters_, sample_,
                         expected_normal, min_cos,
                         static_cast<float>(min_inlier_ratio_), rng_, p)) {
          plane_ = p;
          have_ = true;
        }
      }
    }
    if (!have_) {
      // Keep the sensor data visible even when the floor is out of view.
      // An empty ground cloud also clears any previous RViz display.
      std::vector<float> unclassified;
      unclassified.reserve(pts.size() * 3);
      for (const auto& p : pts) append_xyz(unclassified, p);
      publishCloud(msg->header, {}, *pub_ground_);
      publishCloud(msg->header, unclassified, *pub_nonground_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "No valid ground plane (%zu valid points); publishing unclassified "
          "points on ground/non_ground. Check floor visibility, "
          "camera_pitch_deg and max_slope_deg", pts.size());
      return;
    }

    // split
    std::vector<float> gbuf, ngbuf;
    gbuf.reserve(pts.size() * 3 / 2);
    ngbuf.reserve(pts.size() * 3 / 2);
    for (const auto& p : pts) {
      const float hh = plane_.d - plane_.n.dot(p);
      if (std::fabs(hh) < static_cast<float>(thresh_))
        append_xyz(gbuf, p);
      else if (hh > 0.f)
        append_xyz(ngbuf, p);
    }

    shape_msgs::msg::Plane pl;
    pl.coef[0] = plane_.n.x();
    pl.coef[1] = plane_.n.y();
    pl.coef[2] = plane_.n.z();
    pl.coef[3] = -plane_.d;                // ax+by+cz+d = 0
    pub_plane_->publish(pl);
    publishCloud(msg->header, gbuf, *pub_ground_);
    publishCloud(msg->header, ngbuf, *pub_nonground_);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "ground: n=(%.3f %.3f %.3f) d=%.3f m, %zu ground pts, %zu non-ground pts",
        plane_.n.x(), plane_.n.y(), plane_.n.z(), plane_.d,
        gbuf.size() / 3, ngbuf.size() / 3);
  }

  void publishCloud(const std_msgs::msg::Header& header,
                    const std::vector<float>& buf,
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(buf.size() / 3);
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.fields.resize(3);
    const char* names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      cloud.fields[i].name = names[i];
      cloud.fields[i].offset = 4 * i;
      cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[i].count = 1;
    }
    cloud.data.resize(buf.size() * sizeof(float));
    if (!buf.empty()) {
      std::memcpy(cloud.data.data(), buf.data(), cloud.data.size());
    }
    pub.publish(cloud);
  }

  std::string cloud_topic_, mode_;
  double camera_height_, camera_pitch_deg_, thresh_;
  int iters_, sample_, refresh_, stride_;
  double max_slope_deg_, max_range_, min_inlier_ratio_;
  int frame_ = 0;
  bool have_ = false;
  Plane plane_;
  std::mt19937 rng_{1234};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<shape_msgs::msg::Plane>::SharedPtr pub_plane_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nonground_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmentNode>());
  rclcpp::shutdown();
  return 0;
}
