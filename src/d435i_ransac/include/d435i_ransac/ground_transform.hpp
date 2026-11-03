#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace d435i_ransac {
// Plane coefficients use the optical frame: n points toward the floor and
// n.dot(point) + offset = 0. The ground origin is below the camera.
inline bool ground_transform(const Eigen::Vector3d &normal, double offset,
                             Eigen::Isometry3d &result) {
  if (!normal.allFinite() || !std::isfinite(offset)) return false;
  const double norm = normal.norm();
  if (norm < 1e-9) return false;
  const Eigen::Vector3d down = normal / norm;
  const double height = -offset / norm;
  if (height <= 0.0 || down.y() <= 0.0) return false;
  const Eigen::Vector3d up = -down;
  Eigen::Vector3d forward = Eigen::Vector3d::UnitZ() - up.z() * up;
  if (forward.norm() < 1e-6) return false;
  forward.normalize();
  const Eigen::Vector3d left = up.cross(forward).normalized();
  result = Eigen::Isometry3d::Identity();
  result.linear().row(0) = forward.transpose();
  result.linear().row(1) = left.transpose();
  result.linear().row(2) = up.transpose();
  result.translation() = Eigen::Vector3d(0.0, 0.0, height);
  return true;
}
}  // namespace d435i_ransac
