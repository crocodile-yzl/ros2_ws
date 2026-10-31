#pragma once
// Pure C++/Eigen geometry helpers for the barrier detector.
// Mirrors the Python reference implementation: run-based connected components,
// RANSAC plane fitting, PCA features.

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <random>
#include <vector>

namespace stickdet {

using Vec3 = Eigen::Vector3f;
using Cloud = std::vector<Vec3>;

struct Plane {
  Vec3 n = Vec3(0.f, 0.f, 1.f);
  float d = 0.f;  // plane: n . p = d
};

struct Features {
  Vec3 center = Vec3::Zero();
  Vec3 axis = Vec3(1.f, 0.f, 0.f);
  Vec3 tip_a = Vec3::Zero();
  Vec3 tip_b = Vec3::Zero();
  float length = 0.f;
  float radius = 0.f;
  float elongation = 0.f;
};

// ---------------------------------------------------------------- utilities
inline float percentile(std::vector<float>& v, float p) {
  if (v.empty()) return 0.f;
  size_t k = static_cast<size_t>(std::clamp(p, 0.f, 1.f) * (v.size() - 1) + 0.5f);
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

// ------------------------------------------------------- connected components
// 8-connectivity with 1 px tolerance (matches the Python implementation).
// Returns label image (0 = background) and the number of labels.
inline std::vector<int> connected_components(const std::vector<uint8_t>& mask,
                                             int h, int w, int& n_labels) {
  struct Run {
    int row, c0, c1;
  };
  std::vector<Run> runs;
  runs.reserve(4096);
  for (int r = 0; r < h; ++r) {
    const uint8_t* row = &mask[static_cast<size_t>(r) * w];
    int c = 0;
    while (c < w) {
      while (c < w && !row[c]) ++c;
      if (c >= w) break;
      int c0 = c;
      while (c < w && row[c]) ++c;
      runs.push_back({r, c0, c});
    }
  }
  std::vector<int> out(static_cast<size_t>(h) * w, 0);
  n_labels = 0;
  if (runs.empty()) return out;

  std::vector<int> parent(runs.size());
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&parent](int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };
  std::vector<int> row_start(h + 1, 0);
  for (const auto& ru : runs) row_start[ru.row + 1]++;
  for (int r = 0; r < h; ++r) row_start[r + 1] += row_start[r];

  for (int r = 1; r < h; ++r) {
    int prev_begin = row_start[r - 1], prev_end = row_start[r];
    int j = prev_begin;
    for (int k = row_start[r]; k < row_start[r + 1]; ++k) {
      while (j < prev_end && runs[j].c1 < runs[k].c0 - 1) ++j;
      for (int jj = j; jj < prev_end && runs[jj].c0 <= runs[k].c1 + 1; ++jj) {
        if (runs[jj].c1 >= runs[k].c0 - 1) {
          int a = find(k), b = find(jj);
          if (a != b) parent[a] = b;
        }
      }
    }
  }
  // compact roots -> labels 1..K
  std::vector<int> root_label(runs.size(), 0);
  std::vector<int> root_id(runs.size(), -1);
  int next = 0;
  for (size_t i = 0; i < runs.size(); ++i) {
    int r = find(static_cast<int>(i));
    if (root_id[r] < 0) root_id[r] = ++next;
    root_label[i] = root_id[r];
  }
  n_labels = next;
  for (size_t i = 0; i < runs.size(); ++i) {
    auto& ru = runs[i];
    int* dst = &out[static_cast<size_t>(ru.row) * w + ru.c0];
    const int lab = root_label[i];
    for (int x = ru.c0; x < ru.c1; ++x) dst[x - ru.c0] = lab;
  }
  return out;
}

// ------------------------------------------------------------- RANSAC plane
inline bool ransac_plane(const Cloud& pts, float thresh, int iters, int sample,
                         std::mt19937& rng, Plane& plane,
                         std::vector<uint8_t>& inliers) {
  if (pts.size() < 100) return false;
  Cloud s;
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
    const float d = n.dot(p0);
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
  if (best_cnt < 50) return false;

  // least-squares refine on inliers (3x3 covariance, no SVD of Nx3)
  Cloud inl;
  inl.reserve(best_cnt);
  for (const auto& p : s)
    if (std::fabs(best_n.dot(p) - best_d) < thresh) inl.push_back(p);
  if (inl.size() < 50) return false;
  Vec3 c = Vec3::Zero();
  for (const auto& p : inl) c += p;
  c /= static_cast<float>(inl.size());
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : inl) cov += (p - c) * (p - c).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
  Vec3 n = es.eigenvectors().col(0);  // smallest eigenvalue -> normal

  plane.n = n;
  plane.d = n.dot(c);
  inliers.assign(pts.size(), 0);
  for (size_t i = 0; i < pts.size(); ++i)
    inliers[i] = std::fabs(n.dot(pts[i]) - plane.d) < thresh ? 1 : 0;
  return true;
}

// ------------------------------------------------------------------ features
inline Features stick_features(const Cloud& pts) {
  Features f;
  if (pts.size() < 3) return f;
  Vec3 c = Vec3::Zero();
  for (const auto& p : pts) c += p;
  c /= static_cast<float>(pts.size());
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : pts) cov += (p - c) * (p - c).transpose();
  cov /= static_cast<float>(pts.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
  const Vec3 evals = es.eigenvalues();      // ascending
  f.axis = es.eigenvectors().col(2).normalized();
  f.center = c;
  const float l1 = std::max(evals[2], 1e-12f);
  const float l2 = std::max(evals[1], 1e-12f);
  f.elongation = std::sqrt(l1 / l2);

  std::vector<float> t(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) t[i] = f.axis.dot(pts[i] - c);
  std::vector<float> ts = t;
  const float lo = percentile(ts, 0.01f);
  ts = t;
  const float hi = percentile(ts, 0.99f);
  f.length = hi - lo;
  f.tip_a = c + f.axis * lo;
  f.tip_b = c + f.axis * hi;
  f.center = c + f.axis * (0.5f * (lo + hi));

  double sum2 = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    Vec3 d = (pts[i] - c) - f.axis * t[i];
    sum2 += d.squaredNorm();
  }
  f.radius = static_cast<float>(std::sqrt(sum2 / static_cast<double>(pts.size())));
  return f;
}

// quaternion rotating +z onto v (for CYLINDER markers)
inline Eigen::Quaternionf quat_z_to(const Vec3& v) {
  Vec3 u = v.normalized();
  float d = u.z();
  Eigen::Quaternionf q;
  if (d < -0.999999f) {
    q = Eigen::Quaternionf(0.f, 1.f, 0.f, 0.f);  // 180 deg about x
  } else {
    Vec3 c = Vec3(0.f, 0.f, 1.f).cross(u);
    q = Eigen::Quaternionf(1.f + d, c.x(), c.y(), c.z());
    q.normalize();
  }
  return q;
}

// quaternion rotating +x onto v (for PoseArray orientation)
inline Eigen::Quaternionf quat_x_to(const Vec3& v) {
  Vec3 u = v.normalized();
  float d = u.x();
  Eigen::Quaternionf q;
  if (d < -0.999999f) {
    q = Eigen::Quaternionf(0.f, 0.f, 0.f, 1.f);  // 180 deg about z
  } else {
    Vec3 c = Vec3(1.f, 0.f, 0.f).cross(u);
    q = Eigen::Quaternionf(1.f + d, c.x(), c.y(), c.z());
    q.normalize();
  }
  return q;
}

}  // namespace stickdet
