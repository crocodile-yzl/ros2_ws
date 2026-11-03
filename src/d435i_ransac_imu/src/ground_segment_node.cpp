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
// expected ground normal, so a wall can not be mistaken for the floor.
//
// Expected normal source (use_imu_prior, default true):
//   - IMU: the accelerometer at rest measures +up (specific force), so the
//     gravity direction gives the ground normal directly. librealsense reports
//     an identity rotation between Depth and Gyro/Accel (only mm translation),
//     so the accelerometer axes match camera_depth_optical_frame and no
//     extrinsic conversion is needed. Low-pass filtered; falls back to the
//     config value when the IMU stream is missing or stale.
//   - config: normal built from camera_pitch_deg (previous behaviour).

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
#include "sensor_msgs/msg/imu.hpp"
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
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/d435i/points");
    mode_ = declare_parameter<std::string>("ground_plane_mode", "auto");
    camera_height_ = declare_parameter<double>("camera_height", 0.9);
    camera_pitch_deg_ = declare_parameter<double>("camera_pitch_deg", 34.0);
    thresh_ = declare_parameter<double>("ground_thresh", 0.02);
    iters_ = declare_parameter<int>("ground_iters", 1000);
    sample_ = declare_parameter<int>("ground_sample", 4000);
    refresh_ = declare_parameter<int>("ground_refresh", 30);
    max_slope_deg_ = declare_parameter<double>("max_slope_deg", 15.0);
    min_inlier_ratio_ = declare_parameter<double>("ground_min_inlier_ratio", 0.10);
    max_range_ = declare_parameter<double>("max_range", 3.0);
    stride_ = declare_parameter<int>("downsample_stride", 2);

    // --- IMU 重力先验 ---
    use_imu_prior_ = declare_parameter<bool>("use_imu_prior", true);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/d435i/accel/sample");
    gravity_tau_sec_ = declare_parameter<double>("gravity_tau_sec", 0.3);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.5);
    imu_pitch_offset_deg_ = declare_parameter<double>("imu_pitch_offset_deg", 0.0);
    imu_accel_min_ = declare_parameter<double>("imu_accel_min", 5.0);
    imu_accel_max_ = declare_parameter<double>("imu_accel_max", 14.0);

    if ((mode_ != "auto" && mode_ != "manual") || refresh_ < 1 ||
        iters_ < 1 || sample_ < 100 || stride_ < 1 ||
        !std::isfinite(thresh_) || thresh_ <= 0.0 ||
        !std::isfinite(min_inlier_ratio_) || min_inlier_ratio_ <= 0.0 ||
        min_inlier_ratio_ > 1.0 ||
        !std::isfinite(gravity_tau_sec_) || gravity_tau_sec_ <= 0.0 ||
        !std::isfinite(imu_timeout_sec_) || imu_timeout_sec_ <= 0.0 ||
        !std::isfinite(imu_accel_min_) || !std::isfinite(imu_accel_max_) ||
        imu_accel_min_ >= imu_accel_max_) {
      throw std::invalid_argument("Invalid ground segmentation parameters");
    }

    pitch_offset_rad_ = imu_pitch_offset_deg_ * M_PI / 180.0;
    // 滤波初值取配置先验，IMU 还没到时行为与原来完全一致
    up_ = -config_down();

    if (mode_ == "manual") {
      plane_.n = config_down();
      plane_.d = static_cast<float>(camera_height_);
      have_ = true;
    }

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos,
        std::bind(&GroundSegmentNode::onCloud, this, std::placeholders::_1));
    if (use_imu_prior_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_, qos,
          std::bind(&GroundSegmentNode::onImu, this, std::placeholders::_1));
    }
    pub_plane_ = create_publisher<shape_msgs::msg::Plane>(
        "ground/plane", rclcpp::QoS(5));
    // RViz defaults to reliable subscriptions. A reliable publisher also
    // accepts best-effort subscribers, so both display settings can connect.
    const auto output_qos = rclcpp::QoS(5).reliable();
    pub_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "ground/ground_cloud", output_qos);
    pub_nonground_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "ground/non_ground", output_qos);
    RCLCPP_INFO(get_logger(), "ground_segmentation up (mode=%s, imu_prior=%s%s%s)",
                mode_.c_str(), use_imu_prior_ ? "on" : "off",
                use_imu_prior_ ? ", topic=" : "", use_imu_prior_ ? imu_topic_.c_str() : "");
  }

 private:
  // 配置先验：由 camera_pitch_deg 构造“向下”的单位法向（与原有行为一致）
  Vec3 config_down() const {
    const double p = camera_pitch_deg_ * M_PI / 180.0;
    return Vec3(0.f, static_cast<float>(std::cos(p)),
                static_cast<float>(std::sin(p)));
  }

  // IMU 先验：静止时加速度计测到的是比力，方向朝上；取反即为地面法向。
  // imu_pitch_offset_deg 用于吃掉残余的安装/标定偏差（绕光学 x 轴旋转）。
  Vec3 imu_down() const {
    Vec3 down = -up_;
    if (pitch_offset_rad_ != 0.0) {
      const float c = static_cast<float>(std::cos(pitch_offset_rad_));
      const float s = static_cast<float>(std::sin(pitch_offset_rad_));
      down = Vec3(down.x(), c * down.y() - s * down.z(),
                  s * down.y() + c * down.z());
    }
    const float norm = down.norm();
    return norm > 1e-6f ? down / norm : config_down();
  }

  bool imu_fresh() const {
    if (!use_imu_prior_ || !have_up_) return false;
    return (this->now() - last_imu_time_).seconds() <= imu_timeout_sec_;
  }

  // 当前可用的先验法向：优先 IMU，没有则回退配置值
  Vec3 prior_down(bool* from_imu) const {
    if (imu_fresh()) {
      if (from_imu) *from_imu = true;
      return imu_down();
    }
    if (from_imu) *from_imu = false;
    return config_down();
  }

  // 重力低通：加速度计噪声约 mg 级，用一阶 EMA（时间常数 gravity_tau_sec）
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const Vec3 a(static_cast<float>(msg->linear_acceleration.x),
                 static_cast<float>(msg->linear_acceleration.y),
                 static_cast<float>(msg->linear_acceleration.z));
    const float norm = a.norm();
    // 明显加减速或冲击时扔掉（静止重力约 9.8，BMI085 未标定实测约 9.6）
    if (!std::isfinite(norm) || norm < static_cast<float>(imu_accel_min_) ||
        norm > static_cast<float>(imu_accel_max_)) {
      return;
    }

    const Vec3 unit = a / norm;               // 朝上
    const rclcpp::Time stamp = this->now();
    if (!have_up_) {
      up_ = unit;
      have_up_ = true;
      imu_samples_ = 1;
    } else {
      double dt = (stamp - last_imu_time_).seconds();
      if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5) dt = 1.0 / 200.0;
      const double alpha = dt / (gravity_tau_sec_ + dt);
      const Vec3 blended =
          up_ * static_cast<float>(1.0 - alpha) + unit * static_cast<float>(alpha);
      const float bnorm = blended.norm();
      if (bnorm > 1e-6f) up_ = blended / bnorm;
      ++imu_samples_;
    }
    last_imu_time_ = stamp;

    // 收敛后检查一次配置值与实测俯仰的偏差，提示用户更新参数
    if (!pitch_mismatch_reported_ && imu_samples_ > 100) {
      pitch_mismatch_reported_ = true;
      const Vec3 down = imu_down();
      const double imu_pitch =
          std::atan2(down.z(), down.y()) * 180.0 / M_PI;
      const double imu_roll =
          std::atan2(down.x(), std::hypot(down.y(), down.z())) * 180.0 / M_PI;
      RCLCPP_INFO(get_logger(),
                  "IMU gravity prior: pitch=%.1f deg, roll=%.1f deg, n=(%.3f %.3f %.3f)",
                  imu_pitch, imu_roll, down.x(), down.y(), down.z());
      if (std::fabs(imu_pitch - camera_pitch_deg_) > 5.0) {
        RCLCPP_WARN(get_logger(),
                    "IMU pitch %.1f deg vs camera_pitch_deg %.1f deg (diff %.1f deg); "
                    "the config value is only a fallback, consider updating it",
                    imu_pitch, camera_pitch_deg_,
                    std::fabs(imu_pitch - camera_pitch_deg_));
      }
    }
  }

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
        bool from_imu = false;
        const Vec3 expected_normal = prior_down(&from_imu);
        const float min_cos = static_cast<float>(
            std::cos(max_slope_deg_ * M_PI / 180.0));
        const bool ok = ransac_plane(pts, static_cast<float>(thresh_), iters_,
                                     sample_, expected_normal, min_cos,
                                     static_cast<float>(min_inlier_ratio_), rng_, p);
        if (ok) {
          plane_ = p;
          have_ = true;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "plane fit %s, prior=%s n=(%.3f %.3f %.3f)",
            ok ? "ok" : "failed", from_imu ? "imu" : "config",
            expected_normal.x(), expected_normal.y(), expected_normal.z());
      }
    } else {
      // manual：高度仍由 camera_height 给定，法向用重力方向替换静态俯仰假设
      if (imu_fresh()) {
        plane_.n = imu_down();
        plane_.d = static_cast<float>(camera_height_);
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

  // --- IMU 重力先验 ---
  bool use_imu_prior_ = true;
  std::string imu_topic_;
  double gravity_tau_sec_ = 0.3, imu_timeout_sec_ = 0.5, imu_pitch_offset_deg_ = 0.0;
  double imu_accel_min_ = 5.0, imu_accel_max_ = 14.0;
  double pitch_offset_rad_ = 0.0;
  Vec3 up_ = Vec3(0.f, 0.f, -1.f);          // 重力“朝上”方向（IMU/光学系）
  bool have_up_ = false;
  int imu_samples_ = 0;
  bool pitch_mismatch_reported_ = false;
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
