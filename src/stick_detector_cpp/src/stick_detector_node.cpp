// Barrier / long-stick detector for organized depth point clouds.
//
// Pipeline (per frame):
//   1. parse PointCloud2 -> organized grid (stride-downsampled)
//   2. ROI box + ground-plane high-pass (removes floor / walls / ceiling)
//   3. connected components -> candidate clusters
//   4. optional per-cluster RANSAC plane removal (when the ground was not
//      already removed by the high-pass)
//   5. PCA features -> stick (elongated) or barrier (Π-shaped) classification
//   6. publish PoseArray / MarkerArray / target point cloud

#include "stick_detector_cpp/geometry.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using stickdet::Cloud;
using stickdet::Features;
using stickdet::Plane;
using stickdet::Vec3;

namespace {
struct Detection {
  Features f;
  bool is_barrier = false;
  float width = 0.f;
  float height = 0.f;
  Cloud pts;
};
}  // namespace

class StickDetectorNode : public rclcpp::Node {
 public:
  StickDetectorNode() : Node("stick_detector") {
    // ---- parameters (same names as the Python package) ----
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/berxel_cloudpoint");
    process_hz_ = declare_parameter<double>("process_hz", 10.0);
    stride_ = declare_parameter<int>("downsample_stride", 2);
    max_range_ = declare_parameter<double>("max_range", 5.0);
    min_px_ = declare_parameter<int>("min_cluster_px", 250);

    roi_enable_ = declare_parameter<bool>("roi_enable", true);
    roi_x_ = {declare_parameter<double>("roi_x_min", -1.5),
              declare_parameter<double>("roi_x_max", 1.5)};
    roi_y_ = {declare_parameter<double>("roi_y_min", -0.10),
              declare_parameter<double>("roi_y_max", 2.0)};
    roi_z_ = {declare_parameter<double>("roi_z_min", 0.20),
              declare_parameter<double>("roi_z_max", 5.0)};

    ground_roi_ = declare_parameter<bool>("ground_roi_enable", true);
    band_lo_ = declare_parameter<double>("ground_band_min", -0.03);
    band_hi_ = declare_parameter<double>("ground_band_max", 0.30);
    hp_enable_ = declare_parameter<bool>("ground_hp_enable", true);
    hp_min_ = declare_parameter<double>("ground_hp_min", 0.012);
    gmode_ = declare_parameter<std::string>("ground_plane_mode", "auto");
    camera_height_ = declare_parameter<double>("camera_height", 0.80);
    camera_pitch_deg_ = declare_parameter<double>("camera_pitch_deg", 15.0);
    grefresh_ = declare_parameter<int>("ground_plane_refresh", 30);
    if (grefresh_ < 1) grefresh_ = 1;
    gthresh_ = declare_parameter<double>("ground_plane_thresh", 0.02);
    gsample_ = declare_parameter<int>("ground_plane_sample", 4000);
    gfallback_ = declare_parameter<bool>("ground_plane_fallback_manual", true);

    plane_remove_ = declare_parameter<bool>("plane_remove", true);
    plane_thresh_ = declare_parameter<double>("plane_thresh", 0.012);
    plane_iters_ = declare_parameter<int>("plane_iters", 60);
    plane_sample_ = declare_parameter<int>("plane_sample", 6000);
    plane_min_px_ = declare_parameter<int>("plane_min_px", 2500);
    plane_min_ratio_ = declare_parameter<double>("plane_min_ratio", 0.35);

    stick_on_ = declare_parameter<bool>("stick_enable", false);
    min_len_ = declare_parameter<double>("min_length", 0.25);
    max_len_ = declare_parameter<double>("max_length", 3.0);
    min_elong_ = declare_parameter<double>("min_elongation", 5.0);
    max_radius_ = declare_parameter<double>("max_radius", 0.035);

    barrier_ = declare_parameter<bool>("barrier_enable", true);
    bar_w_ = {declare_parameter<double>("barrier_width_min", 0.40),
              declare_parameter<double>("barrier_width_max", 1.60)};
    bar_h_ = {declare_parameter<double>("barrier_height_min", 0.10),
              declare_parameter<double>("barrier_height_max", 0.45)};
    bar_top_frac_ = declare_parameter<double>("barrier_top_frac", 0.65);
    bar_min_elong_ = declare_parameter<double>("barrier_min_elongation", 3.0);
    bar_max_tilt_ = declare_parameter<double>("barrier_max_tilt", 0.45);

    publish_roi_cloud_ = declare_parameter<bool>("publish_roi_cloud", true);
    roi_box_marker_ = declare_parameter<bool>("roi_box_marker", true);
    cloud_scale_ = declare_parameter<double>("cloud_scale", 1.0);
    debug_log_ = declare_parameter<bool>("debug_log", false);
    profile_ = declare_parameter<bool>("profile", false);

    min_dt_ = process_hz_ > 0.0 ? 1.0 / process_hz_ : 0.0;
    if (gmode_ == "manual") {
      const double p = camera_pitch_deg_ * M_PI / 180.0;
      ground_.n = Vec3(0.f, static_cast<float>(std::cos(p)),
                       static_cast<float>(std::sin(p)));
      ground_.d = static_cast<float>(camera_height_);
      have_ground_ = true;
    }

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos,
        std::bind(&StickDetectorNode::onCloud, this, std::placeholders::_1));
    pub_poses_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "stick_detector/sticks", rclcpp::QoS(5));
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "stick_detector/markers", rclcpp::QoS(5));
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "stick_detector/stick_points", qos);
    pub_roi_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "stick_detector/roi_cloud", qos);

    RCLCPP_INFO(get_logger(),
                "stick_detector (C++) up: topic='%s' stride=%d ground_roi=%d "
                "hp=%d stick=%d barrier=%d",
                cloud_topic_.c_str(), stride_, static_cast<int>(ground_roi_),
                static_cast<int>(hp_enable_), static_cast<int>(stick_on_),
                static_cast<int>(barrier_));
  }

 private:
  using Clock = std::chrono::steady_clock;

  void tick(const char* key, Clock::time_point t0) {
    if (!profile_) return;
    const double ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    acc_[key] += ms;
    if (std::strcmp(key, "total") == 0) {
      ++prof_n_;
      if (prof_n_ % 5 == 0) {
        std::string s = "[profile]";
        for (const auto& kv : acc_) {
          char buf[64];
          std::snprintf(buf, sizeof(buf), " %s=%.1fms", kv.first.c_str(),
                        kv.second / prof_n_);
          s += buf;
        }
        RCLCPP_INFO(get_logger(), "%s", s.c_str());
        acc_.clear();
        prof_n_ = 0;
      }
    }
  }

  // ------------------------------------------------------------- cloud input
  bool parseCloud(const sensor_msgs::msg::PointCloud2& msg, Cloud& out, int& sw,
                  int& sh) {
    int ox = -1, oy = -1, oz = -1;
    for (const auto& f : msg.fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
      if (f.name == "x") ox = static_cast<int>(f.offset);
      else if (f.name == "y") oy = static_cast<int>(f.offset);
      else if (f.name == "z") oz = static_cast<int>(f.offset);
    }
    if (ox < 0 || oy < 0 || oz < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cloud needs x/y/z FLOAT32 fields");
      return false;
    }
    const int w = static_cast<int>(msg.width);
    const int h = static_cast<int>(msg.height);
    const int step = static_cast<int>(msg.point_step);
    if (h <= 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cloud is not organized (enable_ordered_pointcloud)");
      return false;
    }
    const size_t need = static_cast<size_t>(w) * h * step;
    if (msg.data.size() < need) return false;

    const uint8_t* base = msg.data.data();
    const int s = std::max(1, stride_);
    sw = (w + s - 1) / s;
    sh = (h + s - 1) / s;
    out.resize(static_cast<size_t>(sw) * sh);
    for (int r = 0; r < sh; ++r) {
      const uint8_t* row = base + static_cast<size_t>(r * s) * w * step;
      for (int c = 0; c < sw; ++c) {
        const uint8_t* p = row + static_cast<size_t>(c * s) * step;
        float x, y, z;
        std::memcpy(&x, p + ox, 4);
        std::memcpy(&y, p + oy, 4);
        std::memcpy(&z, p + oz, 4);
        if (cloud_scale_ != 1.0) {
          x *= static_cast<float>(cloud_scale_);
          y *= static_cast<float>(cloud_scale_);
          z *= static_cast<float>(cloud_scale_);
        }
        out[static_cast<size_t>(r) * sw + c] = Vec3(x, y, z);
      }
    }
    return true;
  }

  // ------------------------------------------------------------ ground plane
  bool updateGroundPlane(const Cloud& pts, const std::vector<uint8_t>& finite) {
    if (gmode_ == "manual") return have_ground_;
    ++frame_;
    if (have_ground_ && (frame_ % grefresh_) != 1) return true;

    Cloud sub;
    sub.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
      if (finite[i]) sub.push_back(pts[i]);
    if (sub.size() < 500) return have_ground_;

    Plane pl;
    std::vector<uint8_t> inl;
    if (!stickdet::ransac_plane(sub, static_cast<float>(gthresh_), 60, gsample_,
                                rng_, pl, inl))
      return have_ground_;
    if (pl.d < 0.f) {
      pl.n = -pl.n;
      pl.d = -pl.d;
    }
    double ratio = 0.0;
    for (auto v : inl) ratio += v;
    ratio /= static_cast<double>(inl.size());
    if (pl.d < 0.1f || pl.d > 3.0f || ratio < 0.15) return have_ground_;
    ground_ = pl;
    have_ground_ = true;
    return true;
  }

  // ------------------------------------------------------------- candidates
  struct Cluster {
    Cloud pts;
    std::vector<int> idx;  // flat pixel indices
  };

  std::vector<Cluster> groupClusters(const std::vector<int>& labels,
                                     int n_labels, const Cloud& pts) {
    std::vector<int> count(n_labels + 1, 0);
    for (int l : labels)
      if (l > 0) ++count[l];
    std::vector<int> offset(n_labels + 2, 0);
    for (int k = 1; k <= n_labels; ++k) offset[k + 1] = offset[k] + count[k];
    std::vector<int> cursor = offset;
    std::vector<int> idx(labels.size());
    for (size_t i = 0; i < labels.size(); ++i) {
      if (labels[i] > 0) {
        idx[cursor[labels[i]]] = static_cast<int>(i);
        ++cursor[labels[i]];
      }
    }
    std::vector<Cluster> out;
    out.reserve(n_labels);
    for (int k = 1; k <= n_labels; ++k) {
      if (count[k] < min_px_) continue;
      Cluster c;
      c.pts.reserve(count[k]);
      c.idx.reserve(count[k]);
      for (int i = offset[k]; i < offset[k + 1]; ++i) {
        c.pts.push_back(pts[idx[i]]);
        c.idx.push_back(idx[i]);
      }
      out.push_back(std::move(c));
    }
    return out;
  }

  bool isStick(const Features& f) const {
    return f.length >= min_len_ && f.length <= max_len_ &&
           f.elongation >= min_elong_ && f.radius <= max_radius_;
  }

  // Π-shaped barrier: horizontal top bar on two legs standing on the floor.
  bool detectBarrier(const Cloud& pts, Detection& det) {
    if (!have_ground_) return false;
    std::vector<float> hs(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
      hs[i] = ground_.d - ground_.n.dot(pts[i]);
    std::vector<float> tmp = hs;
    const float h_max = stickdet::percentile(tmp, 0.99f);
    tmp = hs;
    const float h_min = stickdet::percentile(tmp, 0.01f);
    if (h_max < bar_h_[0] || h_max > bar_h_[1]) return false;
    if (h_min > 0.08f) return false;  // not standing on the floor

    const float cut = static_cast<float>(bar_top_frac_) * h_max;
    Cloud top;
    top.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
      if (hs[i] >= cut) top.push_back(pts[i]);
    if (top.size() < 30) return false;

    const Features f = stickdet::stick_features(top);
    if (f.elongation < bar_min_elong_) return false;
    const float width = f.length;
    if (width < bar_w_[0] || width > bar_w_[1]) return false;
    if (std::fabs(f.axis.dot(ground_.n)) > bar_max_tilt_) return false;
    const float h_bar = ground_.d - ground_.n.dot(f.center);
    if (h_bar < 0.5f * h_max) return false;

    det.f = f;
    det.is_barrier = true;
    det.width = width;
    det.height = h_max;
    return true;
  }

  // ------------------------------------------------------------------ frame
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const auto now = Clock::now();
    if (min_dt_ > 0.0) {
      const double dt = std::chrono::duration<double>(now - last_t_).count();
      if (dt < min_dt_) return;
      last_t_ = now;
    } else {
      last_t_ = now;
    }

    Cloud pts;
    int w = 0, h = 0;
    cloud_frame_ = msg->header.frame_id;
    if (!parseCloud(*msg, pts, w, h)) return;
    tick("parse", now);

    const size_t n = pts.size();
    std::vector<uint8_t> finite(n), mask(n, 0);
    for (size_t i = 0; i < n; ++i) {
      const Vec3& p = pts[i];
      const bool ok = std::isfinite(p.x()) && std::isfinite(p.y()) &&
                      std::isfinite(p.z());
      finite[i] = ok ? 1 : 0;
      if (!ok) continue;
      if (p.z() <= 0.05f || p.z() >= static_cast<float>(max_range_)) continue;
      if (roi_enable_) {
        if (p.x() < roi_x_[0] || p.x() > roi_x_[1]) continue;
        if (p.y() < roi_y_[0] || p.y() > roi_y_[1]) continue;
        if (p.z() < roi_z_[0] || p.z() > roi_z_[1]) continue;
      }
      mask[i] = 1;
    }
    auto t_mask = Clock::now();
    tick("mask", t_mask);

    int n_before = 0;
    for (auto v : mask) n_before += v;
    bool ground_removed = false;
    if (ground_roi_ && !have_ground_ && gfallback_ && gmode_ != "manual") {
      const double pch = camera_pitch_deg_ * M_PI / 180.0;
      ground_.n = Vec3(0.f, static_cast<float>(std::cos(pch)),
                       static_cast<float>(std::sin(pch)));
      ground_.d = static_cast<float>(camera_height_);
      have_ground_ = true;
      RCLCPP_WARN(get_logger(),
                  "auto ground plane not found -> fallback to manual "
                  "(height=%.2fm pitch=%.1fdeg); check camera_height/"
                  "camera_pitch_deg", camera_height_, camera_pitch_deg_);
    }
    if (ground_roi_ && updateGroundPlane(pts, finite)) {
      const float lo = static_cast<float>(hp_enable_ ? hp_min_ : band_lo_);
      const float hi = static_cast<float>(band_hi_);
      for (size_t i = 0; i < n; ++i) {
        if (!mask[i]) continue;
        const float hh = ground_.d - ground_.n.dot(pts[i]);
        if (hh < lo || hh > hi) mask[i] = 0;
      }
      ground_removed = hp_enable_;
    }
    int n_after = 0;
    for (auto v : mask) n_after += v;
    if (debug_log_ && (roi_enable_ || ground_roi_)) {
      RCLCPP_INFO(get_logger(),
                  "ROI: %d -> %d px (-%.0f%%%s)", n_before, n_after,
                  100.0 * (1.0 - double(n_after) / std::max(n_before, 1)),
                  ground_removed ? ", ground removed" : "");
    }
    if (n_after < min_px_ * 4) return;
    auto t_ground = Clock::now();
    tick("ground", t_ground);

    int n_labels = 0;
    std::vector<int> labels = stickdet::connected_components(mask, h, w, n_labels);
    auto t_cc = Clock::now();
    tick("cluster", t_cc);

    std::vector<Cluster> cands = groupClusters(labels, n_labels, pts);
    auto t_plane = Clock::now();

    if (plane_remove_ && !ground_removed) {
      std::vector<Cluster> refined;
      for (auto& c : cands) {
        if (static_cast<int>(c.pts.size()) < plane_min_px_) {
          refined.push_back(std::move(c));
          continue;
        }
        Plane pl;
        std::vector<uint8_t> inl;
        if (!stickdet::ransac_plane(c.pts, static_cast<float>(plane_thresh_),
                                    plane_iters_, plane_sample_, rng_, pl, inl)) {
          refined.push_back(std::move(c));
          continue;
        }
        double ratio = 0.0;
        for (auto v : inl) ratio += v;
        ratio /= static_cast<double>(inl.size());
        if (ratio < plane_min_ratio_) {
          refined.push_back(std::move(c));
          continue;
        }
        std::vector<uint8_t> sub(mask.size(), 0);
        for (size_t k = 0; k < c.idx.size(); ++k)
          if (!inl[k]) sub[c.idx[k]] = 1;   // keep non-planar pixels
        int sub_labels = 0;
        std::vector<int> slab =
            stickdet::connected_components(sub, h, w, sub_labels);
        for (auto& s : groupClusters(slab, sub_labels, pts))
          refined.push_back(std::move(s));
      }
      cands.swap(refined);
    }
    tick("plane", t_plane);

    auto t_pca = Clock::now();
    std::vector<Detection> dets;
    std::string dbg;
    for (auto& c : cands) {
      const Features f = stickdet::stick_features(c.pts);
      if (debug_log_) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), " L=%.2f r=%.0fmm e=%.0f", f.length,
                      f.radius * 1000.f, f.elongation);
        dbg += buf;
      }
      if (stick_on_ && isStick(f)) {
        Detection d;
        d.f = f;
        d.pts = std::move(c.pts);
        dets.push_back(std::move(d));
        continue;
      }
      if (barrier_) {
        Detection d;
        if (detectBarrier(c.pts, d)) {
          d.pts = std::move(c.pts);
          dets.push_back(std::move(d));
        }
      }
    }
    if (debug_log_ && !dbg.empty())
      RCLCPP_INFO(get_logger(), "candidates(%zu):%s", cands.size(), dbg.c_str());
    tick("pca", t_pca);

    auto t_pub = Clock::now();
    publish(*msg, dets, pts, mask);
    tick("publish", t_pub);
    tick("total", now);

    if (!dets.empty()) {
      std::string s;
      int i = 0;
      for (const auto& d : dets) {
        ++i;
        char buf[160];
        if (d.is_barrier)
          std::snprintf(buf, sizeof(buf),
                        "#%d BARRIER W=%.2fm H=%.2fm c=(%.2f,%.2f,%.2f)", i,
                        d.width, d.height, d.f.center.x(), d.f.center.y(),
                        d.f.center.z());
        else
          std::snprintf(buf, sizeof(buf),
                        "#%d L=%.2fm r=%.0fmm e=%.0f c=(%.2f,%.2f,%.2f)", i,
                        d.f.length, d.f.radius * 1000.f, d.f.elongation,
                        d.f.center.x(), d.f.center.y(), d.f.center.z());
        if (!s.empty()) s += "; ";
        s += buf;
      }
      RCLCPP_INFO(get_logger(), "detected %zu target(s): %s", dets.size(),
                  s.c_str());
    }
  }

  // ---------------------------------------------------------------- publish
  void publish(const sensor_msgs::msg::PointCloud2& in,
               const std::vector<Detection>& dets, const Cloud& pts,
               const std::vector<uint8_t>& mask) {
    geometry_msgs::msg::PoseArray poses;
    poses.header = in.header;

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);

    Cloud all;
    int id = 0;
    for (const auto& d : dets) {
      ++id;
      geometry_msgs::msg::Pose pose;
      pose.position.x = d.f.center.x();
      pose.position.y = d.f.center.y();
      pose.position.z = d.f.center.z();
      const Eigen::Quaternionf q = stickdet::quat_x_to(d.f.axis);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      poses.poses.push_back(pose);

      visualization_msgs::msg::Marker line;
      line.header = in.header;
      line.ns = "sticks";
      line.id = id;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.03;
      line.color.r = 1.0f;
      line.color.g = 0.2f;
      line.color.b = 0.1f;
      line.color.a = 0.95f;
      line.pose.orientation.w = 1.0f;
      for (const auto& tip : {d.f.tip_a, d.f.tip_b}) {
        geometry_msgs::msg::Point p;
        p.x = tip.x();
        p.y = tip.y();
        p.z = tip.z();
        line.points.push_back(p);
      }
      ma.markers.push_back(line);

      if (!d.is_barrier) {
        visualization_msgs::msg::Marker cyl;
        cyl.header = in.header;
        cyl.ns = "stick_cylinders";
        cyl.id = id;
        cyl.type = visualization_msgs::msg::Marker::CYLINDER;
        cyl.action = visualization_msgs::msg::Marker::ADD;
        cyl.pose.position = pose.position;
        const Eigen::Quaternionf qz = stickdet::quat_z_to(d.f.axis);
        cyl.pose.orientation.x = qz.x();
        cyl.pose.orientation.y = qz.y();
        cyl.pose.orientation.z = qz.z();
        cyl.pose.orientation.w = qz.w();
        const float r_vis = std::max(d.f.radius, 0.008f);
        cyl.scale.x = cyl.scale.y = 2.f * r_vis;
        cyl.scale.z = d.f.length;
        cyl.color.r = 1.0f;
        cyl.color.a = 0.55f;
        ma.markers.push_back(cyl);
      }

      visualization_msgs::msg::Marker txt;
      txt.header = in.header;
      txt.ns = "stick_labels";
      txt.id = id;
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = d.f.center.x();
      txt.pose.position.y = d.f.center.y();
      txt.pose.position.z = d.f.center.z() + 0.06f;
      txt.pose.orientation.w = 1.0f;
      txt.scale.z = 0.05f;
      txt.color.r = txt.color.g = txt.color.b = 1.0f;
      txt.color.a = 0.95f;
      char buf[96];
      if (d.is_barrier)
        std::snprintf(buf, sizeof(buf), "barrier %d W=%.2fm H=%.2fm", id,
                      d.width, d.height);
      else
        std::snprintf(buf, sizeof(buf), "stick %d %.2fm", id, d.f.length);
      txt.text = buf;
      ma.markers.push_back(txt);

      all.insert(all.end(), d.pts.begin(), d.pts.end());
    }

    if (roi_box_marker_ && roi_enable_) {
      visualization_msgs::msg::Marker box;
      box.header = in.header;
      box.ns = "roi_box";
      box.id = 0;
      box.type = visualization_msgs::msg::Marker::LINE_LIST;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.scale.x = 0.006;
      box.color.r = 0.2f;
      box.color.g = 1.0f;
      box.color.b = 0.4f;
      box.color.a = 0.8f;
      box.pose.orientation.w = 1.0f;
      const float x0 = roi_x_[0], x1 = roi_x_[1];
      const float y0 = roi_y_[0], y1 = roi_y_[1];
      const float z0 = roi_z_[0], z1 = roi_z_[1];
      const float cx[8] = {x0, x1, x1, x0, x0, x1, x1, x0};
      const float cy[8] = {y0, y0, y1, y1, y0, y0, y1, y1};
      const float cz[8] = {z0, z0, z0, z0, z1, z1, z1, z1};
      const int edges[24] = {0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4,
                             0,4, 1,5, 2,6, 3,7};
      for (int i = 0; i < 24; ++i) {
        geometry_msgs::msg::Point p;
        p.x = cx[edges[i]];
        p.y = cy[edges[i]];
        p.z = cz[edges[i]];
        box.points.push_back(p);
      }
      ma.markers.push_back(box);
    }

    pub_poses_->publish(poses);
    pub_markers_->publish(ma);

    if (publish_roi_cloud_) {
      Cloud roi_pts;
      roi_pts.reserve(pts.size() / 4 + 16);
      for (size_t i = 0; i < pts.size() && i < mask.size(); ++i)
        if (mask[i]) roi_pts.push_back(pts[i]);
      if (!roi_pts.empty()) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = in.header;
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(roi_pts.size());
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
        cloud.data.resize(roi_pts.size() * 12);
        float* dst = reinterpret_cast<float*>(cloud.data.data());
        for (size_t i = 0; i < roi_pts.size(); ++i) {
          dst[3 * i + 0] = roi_pts[i].x();
          dst[3 * i + 1] = roi_pts[i].y();
          dst[3 * i + 2] = roi_pts[i].z();
        }
        pub_roi_cloud_->publish(cloud);
      }
    }

    if (!all.empty()) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = in.header;
      cloud.height = 1;
      cloud.width = static_cast<uint32_t>(all.size());
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
      cloud.data.resize(all.size() * 12);
      float* dst = reinterpret_cast<float*>(cloud.data.data());
      for (size_t i = 0; i < all.size(); ++i) {
        dst[3 * i + 0] = all[i].x();
        dst[3 * i + 1] = all[i].y();
        dst[3 * i + 2] = all[i].z();
      }
      pub_cloud_->publish(cloud);
    }
  }

  // ------------------------------------------------------------- parameters
  std::string cloud_topic_;
  double process_hz_ = 10.0;
  int stride_ = 2;
  double max_range_ = 5.0;
  int min_px_ = 250;

  bool roi_enable_ = true;
  std::array<double, 2> roi_x_{-1.5, 1.5}, roi_y_{-0.1, 2.0}, roi_z_{0.2, 5.0};

  bool ground_roi_ = true;
  double band_lo_ = -0.03, band_hi_ = 0.30;
  bool hp_enable_ = true;
  double hp_min_ = 0.012;
  std::string gmode_ = "auto";
  double camera_height_ = 0.8, camera_pitch_deg_ = 15.0;
  int grefresh_ = 30;
  double gthresh_ = 0.02;
  int gsample_ = 4000;

  bool plane_remove_ = true;
  double plane_thresh_ = 0.012;
  int plane_iters_ = 60, plane_sample_ = 6000, plane_min_px_ = 2500;
  double plane_min_ratio_ = 0.35;

  bool stick_on_ = false;
  double min_len_ = 0.25, max_len_ = 3.0, min_elong_ = 5.0, max_radius_ = 0.035;

  bool barrier_ = true;
  std::array<double, 2> bar_w_{0.4, 1.6}, bar_h_{0.1, 0.45};
  double bar_top_frac_ = 0.65, bar_min_elong_ = 3.0, bar_max_tilt_ = 0.45;

  bool publish_roi_cloud_ = true, roi_box_marker_ = true;
  bool gfallback_ = true;
  bool debug_log_ = false, profile_ = false;

  double cloud_scale_ = 1.0;
  std::string cloud_frame_;

  Plane ground_;
  bool have_ground_ = false;
  int frame_ = 0;
  std::mt19937 rng_{1234};

  double min_dt_ = 0.1;
  Clock::time_point last_t_ = Clock::now();

  std::map<std::string, double> acc_;
  int prof_n_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_poses_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_roi_cloud_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StickDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
