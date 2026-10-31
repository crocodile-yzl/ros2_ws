// Synthetic organized point cloud with road-block geometry, for testing.
//
// Camera 0.80 m above the floor, pitched 15 deg down:
//   floor + back wall + Π barrier (800 x 200 mm, tube dia. 40 mm) + box + pillar
// Same layout as the Python sim node, so results are directly comparable.

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace {
constexpr int W = 640;
constexpr int H = 400;
constexpr float FX = 570.f;
constexpr float FY = 570.f;
constexpr float CX = 320.f;
constexpr float CY = 200.f;
constexpr float H_CAM = 0.80f;
constexpr float PITCH_DEG = 15.f;

struct GroundPoint {
  float x, s, h;
};

Eigen::Vector3f g2c(const GroundPoint& p, float pitch) {
  const float cp = std::cos(pitch), sp = std::sin(pitch);
  return Eigen::Vector3f(p.x, (H_CAM - p.h) * cp - p.s * sp,
                         (H_CAM - p.h) * sp + p.s * cp);
}

void addGrid(std::vector<GroundPoint>& out, float x0, float x1, float dx,
             float s0, float s1, float ds, float h0, float h1, float dh) {
  for (float x = x0; x <= x1 + 1e-6f; x += dx)
    for (float s = s0; s <= s1 + 1e-6f; s += ds)
      for (float h = h0; h <= h1 + 1e-6f; h += dh) out.push_back({x, s, h});
}

// straight tube between two ground points
void addTube(std::vector<GroundPoint>& out, const GroundPoint& a,
             const GroundPoint& b, float r, float step, int n_theta) {
  Eigen::Vector3f pa(a.x, a.s, a.h), pb(b.x, b.s, b.h);
  Eigen::Vector3f axis = pb - pa;
  const float len = axis.norm();
  if (len < 1e-6f) return;
  axis /= len;
  Eigen::Vector3f tmp = std::fabs(axis.z()) < 0.9f ? Eigen::Vector3f(0, 0, 1)
                                                   : Eigen::Vector3f(1, 0, 0);
  Eigen::Vector3f e1 = axis.cross(tmp).normalized();
  Eigen::Vector3f e2 = axis.cross(e1);
  for (float t = 0.f; t <= len + 1e-6f; t += step) {
    for (int i = 0; i < n_theta; ++i) {
      const float th = 2.f * float(M_PI) * i / n_theta;
      Eigen::Vector3f p = pa + axis * t + r * (std::cos(th) * e1 + std::sin(th) * e2);
      out.push_back({p.x(), p.y(), p.z()});
    }
  }
}

void addDisk(std::vector<GroundPoint>& out, float cx, float cs, float r) {
  const float step = 0.006f;
  for (float x = -r; x <= r + 1e-6f; x += step)
    for (float s = -r; s <= r + 1e-6f; s += step)
      if (x * x + s * s <= r * r) out.push_back({cx + x, cs + s, 0.008f});
}

}  // namespace

class CloudSimNode : public rclcpp::Node {
 public:
  CloudSimNode() : Node("berxel_cloud_sim") {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "berxel_cloudpoint", rclcpp::SensorDataQoS());
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "depth/depth_raw", rclcpp::SensorDataQoS());
    depth_scale_ = declare_parameter<double>("sim_depth_scale", 1.0);
    z_scale_ = declare_parameter<double>("sim_cloud_scale", 1.0);
    buildScene();
    RCLCPP_INFO(get_logger(), "sim scene ready: %zu pts (floor + wall + barrier "
                              "800x200mm @2.6m + box + pillar)", scene_.size());
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&CloudSimNode::publish, this));
  }

 private:
  void buildScene() {
    std::vector<GroundPoint> gp;
    addGrid(gp, -1.5f, 1.5f, 0.014f, 1.0f, 3.2f, 0.014f, 0.f, 0.f, 1.f);   // floor
    addGrid(gp, -1.5f, 1.5f, 0.018f, 3.2f, 3.2f, 1.f, 0.f, 1.0f, 0.018f);   // wall
    addGrid(gp, -0.80f, -0.60f, 0.010f, 2.0f, 2.2f, 0.010f, 0.f, 0.20f, 0.015f);  // box
    addGrid(gp, 0.70f, 0.86f, 0.010f, 2.5f, 2.66f, 0.010f, 0.f, 1.20f, 0.03f);    // pillar

    const float S0 = 2.6f, half = 0.40f, top = 0.20f, R = 0.02f;
    addTube(gp, {-half, S0, top}, {half, S0, top}, R, 0.008f, 24);      // top bar
    addTube(gp, {-half, S0, top}, {-half, S0, R}, R, 0.008f, 24);       // left leg
    addTube(gp, {half, S0, top}, {half, S0, R}, R, 0.008f, 24);         // right leg
    addDisk(gp, -half, S0, 0.055f);
    addDisk(gp, half, S0, 0.055f);

    const float pitch = PITCH_DEG * float(M_PI) / 180.f;
    scene_.reserve(gp.size());
    for (const auto& p : gp) scene_.push_back(g2c(p, pitch));
  }

  void publish() {
    static std::mt19937 rng(7);
    static std::normal_distribution<float> noise(0.f, 0.003f);

    const size_t n = static_cast<size_t>(W) * H;
    std::vector<float> zbuf(n, std::numeric_limits<float>::infinity());
    std::vector<Eigen::Vector3f> pbuf(n, Eigen::Vector3f::Zero());
    std::vector<uint8_t> hit(n, 0);

    for (const auto& p0 : scene_) {
      Eigen::Vector3f p = p0;
      p.x() += noise(rng);
      p.y() += noise(rng);
      p.z() += noise(rng);
      if (p.z() <= 0.1f) continue;
      const int u = static_cast<int>(std::lround(CX + FX * p.x() / p.z()));
      const int v = static_cast<int>(std::lround(CY + FY * p.y() / p.z()));
      if (u < 0 || u >= W || v < 0 || v >= H) continue;
      const size_t k = static_cast<size_t>(v) * W + u;
      if (p.z() < zbuf[k]) {
        zbuf[k] = p.z();
        pbuf[k] = p;
        hit[k] = 1;
      }
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = "berxel_camera/berxel_camera_depth_optical_frame";
    msg.height = H;
    msg.width = W;
    msg.point_step = 16;
    msg.row_step = msg.point_step * W;
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.fields.resize(4);
    const char* names[4] = {"x", "y", "z", "rgb"};
    for (int i = 0; i < 4; ++i) {
      msg.fields[i].name = names[i];
      msg.fields[i].offset = 4 * i;
      msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[i].count = 1;
    }
    // ---- optional: publish the matching 16UC1 depth image (millimetres) ----
    {
      sensor_msgs::msg::Image img;
      img.header = msg.header;
      img.height = H;
      img.width = W;
      img.encoding = "16UC1";
      img.is_bigendian = false;
      img.step = W * 2;
      img.data.resize(n * 2);
      uint16_t* d = reinterpret_cast<uint16_t*>(img.data.data());
      for (size_t k = 0; k < n; ++k) {
        d[k] = hit[k] ? static_cast<uint16_t>(
            std::lround(zbuf[k] * 1000.0 * depth_scale_)) : 0;
      }
      depth_pub_->publish(img);
    }

    msg.data.resize(n * 16);
    float* dst = reinterpret_cast<float*>(msg.data.data());
    for (size_t k = 0; k < n; ++k) {
      if (hit[k]) {
        dst[4 * k + 0] = pbuf[k].x();
        dst[4 * k + 1] = pbuf[k].y();
        dst[4 * k + 2] = pbuf[k].z();
        dst[4 * k + 3] = 0.f;
      } else {
        dst[4 * k + 0] = std::numeric_limits<float>::quiet_NaN();
        dst[4 * k + 1] = std::numeric_limits<float>::quiet_NaN();
        dst[4 * k + 2] = std::numeric_limits<float>::quiet_NaN();
        dst[4 * k + 3] = 0.f;
      }
    }
    // cloud coordinates can be scaled independently to emulate calibration error
    if (z_scale_ != 1.0) {
      float* pf = reinterpret_cast<float*>(msg.data.data());
      for (size_t k = 0; k < n * 4; ++k) pf[k] *= static_cast<float>(z_scale_);
    }

    pub_->publish(msg);
  }

  std::vector<Eigen::Vector3f> scene_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  double depth_scale_ = 1.0, z_scale_ = 1.0;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudSimNode>());
  rclcpp::shutdown();
  return 0;
}
