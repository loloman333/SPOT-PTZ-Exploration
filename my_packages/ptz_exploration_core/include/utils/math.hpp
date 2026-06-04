#ifndef PTZ_EXPLORATION_CORE__UTILS__MATH_HPP_
#define PTZ_EXPLORATION_CORE__UTILS__MATH_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/quaternion.hpp>
#include <gtsam/geometry/Pose3.h>

namespace ptz_exploration_core::utils
{

// Camera intrinsics
struct CameraModel
{
  double fx, fy, cx, cy;
  int image_width;
  int image_height;
};

/**
 * @brief Project a pixel coordinate to a normalized 3D ray in camera frame.
 */
Eigen::Vector3d pixel_to_ray(double u, double v, const CameraModel & camera);

/**
 * @brief Rotate a 3D vector by a quaternion.
 */
Eigen::Vector3d rotate_vector(
  const Eigen::Vector3d & vec,
  const geometry_msgs::msg::Quaternion & quat);

/**
 * @brief Return quaternion conjugate (inverse for unit quaternions).
 */
geometry_msgs::msg::Quaternion invert_quaternion(
  const geometry_msgs::msg::Quaternion & quat);

/**
 * @brief Compare two poses using translation and rotation thresholds.
 */
bool poses_similar(
  const gtsam::Pose3 & pose_a,
  const gtsam::Pose3 & pose_b,
  double translation_threshold_m,
  double rotation_threshold_rad);

/**
 * @brief Convert 3x3 covariance to ellipsoid half-axes and orientation.
 */
bool covariance_to_ellipsoid(
  const Eigen::Matrix3d & cov3,
  double chi_squared,
  double max_axis,
  double min_eigenvalue,
  Eigen::Vector3d & half_axes,
  Eigen::Quaterniond & orientation);

}  // namespace ptz_exploration_core::utils

#endif  // PTZ_EXPLORATION_CORE__UTILS__MATH_HPP_
