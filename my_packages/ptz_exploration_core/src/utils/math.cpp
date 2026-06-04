#include "utils/math.hpp"

#include <algorithm>
#include <cmath>

namespace ptz_exploration_core::utils {

Eigen::Vector3d pixel_to_ray(double u, double v, const CameraModel &camera) {
  Eigen::Vector3d ray((u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy,
                      1.0);
  return ray.normalized();
}

Eigen::Vector3d rotate_vector(const Eigen::Vector3d &vec,
                              const geometry_msgs::msg::Quaternion &quat) {
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  Eigen::Matrix3d R = q.toRotationMatrix();
  return R * vec;
}

geometry_msgs::msg::Quaternion
invert_quaternion(const geometry_msgs::msg::Quaternion &quat) {
  geometry_msgs::msg::Quaternion inverted;
  inverted.w = quat.w;
  inverted.x = -quat.x;
  inverted.y = -quat.y;
  inverted.z = -quat.z;
  return inverted;
}

bool poses_similar(const gtsam::Pose3 &pose_a, const gtsam::Pose3 &pose_b,
                   double translation_threshold_m,
                   double rotation_threshold_rad) {
  if (!std::isfinite(translation_threshold_m) ||
      translation_threshold_m < 0.0 || !std::isfinite(rotation_threshold_rad) ||
      rotation_threshold_rad < 0.0) {
    return false;
  }

  const gtsam::Point3 t_a = pose_a.translation();
  const gtsam::Point3 t_b = pose_b.translation();

  const double dx = t_a.x() - t_b.x();
  const double dy = t_a.y() - t_b.y();
  const double dz = t_a.z() - t_b.z();
  const double translation_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (translation_distance > translation_threshold_m) {
    return false;
  }

  const gtsam::Rot3 relative_rotation =
      pose_a.rotation().between(pose_b.rotation());
  const double rotation_distance =
      gtsam::Rot3::Logmap(relative_rotation).norm();
  return rotation_distance <= rotation_threshold_rad;
}

bool covariance_to_ellipsoid(const Eigen::Matrix3d &cov3, double chi_squared,
                             double max_axis, double min_eigenvalue,
                             Eigen::Vector3d &half_axes,
                             Eigen::Quaterniond &orientation) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov3);
  if (eigensolver.info() != Eigen::Success) {
    return false;
  }

  Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
  Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();

  // Keep basis right-handed for valid quaternion conversion.
  if (eigenvectors.determinant() < 0.0) {
    eigenvectors.col(2) *= -1.0;
  }

  half_axes(0) = std::min(
      std::sqrt(chi_squared * std::max(eigenvalues(0), min_eigenvalue)),
      max_axis);
  half_axes(1) = std::min(
      std::sqrt(chi_squared * std::max(eigenvalues(1), min_eigenvalue)),
      max_axis);
  half_axes(2) = std::min(
      std::sqrt(chi_squared * std::max(eigenvalues(2), min_eigenvalue)),
      max_axis);

  orientation = Eigen::Quaterniond(eigenvectors);
  return true;
}

} // namespace ptz_exploration_core::utils
