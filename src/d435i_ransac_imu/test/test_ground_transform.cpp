#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "d435i_ransac_imu/ground_transform.hpp"

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  for (const double roll : {0.0, 0.18}) {
    const double pitch = 34.0 * std::acos(-1.0) / 180.0;
    const Eigen::Vector3d down = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) *
        Eigen::Vector3d(0, std::cos(pitch), std::sin(pitch));
    Eigen::Isometry3d transform;
    require(d435i_ransac_imu::ground_transform(3.0 * down, -2.7, transform), "Valid plane rejected");
    const Eigen::Vector3d floor = 0.9 * down;
    require(std::abs((transform * floor).z()) < 1e-9, "Floor must be at z=0");
    require(std::abs((transform * (floor - 0.2 * down)).z() - 0.2) < 1e-9,
            "Obstacle must be above the ground");
    const Eigen::Vector3d forward = Eigen::Vector3d::UnitZ() - down.z() * down;
    require((transform * (floor + forward)).x() > 0, "Camera forward must be ground +X");
    require((transform * (floor + Eigen::Vector3d::UnitX())).y() < 0,
            "Camera right must be ground -Y");
    require(std::abs(transform.linear().determinant() - 1) < 1e-9,
            "Transform must preserve handedness");
    require((transform * Eigen::Vector3d::Zero() - Eigen::Vector3d(0, 0, .9)).norm() < 1e-9,
            "Camera must be above ground origin");
  }
  Eigen::Isometry3d transform;
  require(!d435i_ransac_imu::ground_transform(Eigen::Vector3d::Zero(), -1, transform), "Zero normal accepted");
  require(!d435i_ransac_imu::ground_transform(Eigen::Vector3d::UnitY(), 1, transform), "Negative height accepted");
  require(!d435i_ransac_imu::ground_transform(-Eigen::Vector3d::UnitY(), -1, transform), "Reversed normal accepted");
  require(!d435i_ransac_imu::ground_transform(Eigen::Vector3d::UnitZ(), -1, transform), "Degenerate forward accepted");
  require(!d435i_ransac_imu::ground_transform(Eigen::Vector3d::UnitY(), std::numeric_limits<double>::quiet_NaN(), transform), "NaN accepted");
  std::cout << "Ground transform pitch, roll, handedness, obstacle height and invalid plane checks passed\n";
}
