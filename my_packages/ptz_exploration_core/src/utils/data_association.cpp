#include "utils/data_association.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace ptz_exploration_core {
namespace utils {

// Compute 2x3 bearing Jacobian (azimuth, elevation) wrt relative position
Eigen::Matrix<double, 2, 3>
compute_bearing_jacobian(const Eigen::Vector3d &relative_pos) {
  const double x = relative_pos.x();
  const double y = relative_pos.y();
  const double z = relative_pos.z();

  const double rho_xy_sq = x * x + y * y;
  const double rho_xy = std::sqrt(rho_xy_sq);
  const double r_sq = rho_xy_sq + z * z;
  // const double r = std::sqrt(r_sq);

  Eigen::Matrix<double, 2, 3> J;

  // ∂azimuth/∂(x,y,z)
  if (rho_xy_sq > 1e-12) {
    J(0, 0) = -y / rho_xy_sq;
    J(0, 1) = x / rho_xy_sq;
    J(0, 2) = 0.0;
  } else {
    // Singular at zenith/nadir
    J.row(0).setZero();
  }

  // ∂elevation/∂(x,y,z)
  if (r_sq > 1e-12 && rho_xy > 1e-12) {
    J(1, 0) = -x * z / (r_sq * rho_xy);
    J(1, 1) = -y * z / (r_sq * rho_xy);
    J(1, 2) = rho_xy / r_sq;
  } else {
    // Singular at origin or zenith/nadir
    J.row(1).setZero();
  }

  return J;
}

// Compute squared Mahalanobis distance between two bearing observations in 2D
// tangent space
double squaredMahalanobisDistance(const PolarObservation &o1,
                                  const PolarObservation &o2) {
  // Compute bearing difference in 3D, then project to 2D tangent space
  const Eigen::Vector3d v1 = o1.observation_direction.unitVector();
  const Eigen::Vector3d v2 = o2.observation_direction.unitVector();

  // Build tangent basis at v1
  Eigen::Vector3d basis_seed = (std::abs(v1.z()) < 0.9)
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitX();
  Eigen::Vector3d t1 = (basis_seed - basis_seed.dot(v1) * v1).normalized();
  Eigen::Vector3d t2 = v1.cross(t1).normalized();

  // Project 3D difference onto tangent plane
  const Eigen::Vector3d diff_3d = v2 - v1;
  Eigen::Vector2d diff_2d;
  diff_2d(0) = t1.dot(diff_3d);
  diff_2d(1) = t2.dot(diff_3d);

  // Combine 2D covariances (assuming independence)
  Eigen::Matrix2d combined_cov =
      o1.observation_direction_covariance + o2.observation_direction_covariance;

  // Solve using LDLT for numerical stability
  Eigen::LDLT<Eigen::Matrix2d> ldlt(combined_cov);
  if (ldlt.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Vector2d solved = ldlt.solve(diff_2d);
  if (ldlt.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }

  // Compute squared Mahalanobis distance
  double distance = diff_2d.dot(solved);

  return distance;
}

bool individualCompatibility(double D2_ij, int dof, double alpha) {
  // Using boost's chi-squared distribution to get the quantile (ppf in scipy)
  boost::math::chi_squared chi2_dist(dof);
  double threshold = boost::math::quantile(chi2_dist, alpha);

  // Check if the squared Mahalanobis distance is less than or equal to the
  // threshold
  return D2_ij <= threshold;
}

void icnn(
    const std::vector<std::shared_ptr<PolarObservation>> &new_observations,
    const std::vector<std::shared_ptr<Landmark>> &landmarks,
    const gtsam::Pose3 &camera_pose, const CameraModel &camera_model,
    const std::optional<
        std::reference_wrapper<const Eigen::Matrix<double, 6, 6>>> &pose_cov,
    double alpha, const Eigen::Matrix3d &min_landmark_covariance,
    std::vector<std::tuple<int, std::shared_ptr<PolarObservation>>>
        &paired_observations,
    std::vector<std::shared_ptr<PolarObservation>> &unpaired_observations) {
  paired_observations.clear();
  unpaired_observations.clear();

  // Track the best landmark match per observation.
  std::vector<double> min_distances(new_observations.size(),
                                    std::numeric_limits<double>::infinity());
  std::vector<int> best_match_indices(new_observations.size(), -1);

  for (size_t j = 0; j < landmarks.size(); ++j) {
    const auto &landmark = landmarks[j];
    if (!landmark) {
      continue;
    }

    Eigen::Vector3d relative_pos_camera;
    gtsam::Unit3 expected_direction_camera;
    if (!expected_observation_mean(camera_pose, camera_model, landmark,
                                   relative_pos_camera,
                                   expected_direction_camera)) {
      continue;
    }

    auto expected_observation = std::make_shared<PolarObservation>();
    expected_observation->observation_pose = camera_pose;
    expected_observation->observation_direction = expected_direction_camera;
    expected_observation->observation_direction_covariance =
        expected_observation_cov(landmark, relative_pos_camera,
                                 camera_pose.rotation(), pose_cov,
                                 min_landmark_covariance);

    // Perform data association using the ICNN algorithm.
    for (size_t i = 0; i < new_observations.size(); ++i) {
      const auto &new_observation = new_observations[i];
      if (!new_observation) {
        continue;
      }

      double distance =
          squaredMahalanobisDistance(*new_observation, *expected_observation);

      if (distance < min_distances[i] &&
          individualCompatibility(distance, 2, alpha)) // 2 DoF for bearing
      {
        min_distances[i] = distance;
        best_match_indices[i] = static_cast<int>(j);
      }
    }
  }

  for (size_t i = 0; i < new_observations.size(); ++i) {
    const auto &new_observation = new_observations[i];
    if (!new_observation) {
      continue;
    }

    if (best_match_indices[i] == -1) {
      // Unpaired observation
      unpaired_observations.push_back(new_observation);
    } else {
      // Paired observation
      paired_observations.push_back(
          std::make_tuple(best_match_indices[i], new_observation));
    }
  }
}

bool expected_observation_mean(const gtsam::Pose3 &camera_pose,
                               const CameraModel &camera_model,
                               const std::shared_ptr<Landmark> &landmark,
                               Eigen::Vector3d &relative_pos_camera,
                               gtsam::Unit3 &expected_direction_camera) {
  const Eigen::Vector3d relative_pos_world =
      landmark->position - camera_pose.translation();
  if (relative_pos_world.norm() < 1e-6)
    return false;

  // Transform relative position into camera frame to match measurement
  // convention.
  relative_pos_camera = camera_pose.rotation().inverse() * relative_pos_world;

  // Gate: reject landmarks behind camera.
  if (relative_pos_camera.z() <= 1e-6)
    return false;

  // Gate: pinhole projection must fall inside image bounds.
  const double u =
      camera_model.fx * (relative_pos_camera.x() / relative_pos_camera.z()) +
      camera_model.cx;
  const double v =
      camera_model.fy * (relative_pos_camera.y() / relative_pos_camera.z()) +
      camera_model.cy;

  if (u < 0.0 || u >= static_cast<double>(camera_model.image_width) ||
      v < 0.0 || v >= static_cast<double>(camera_model.image_height)) {
    return false;
  }

  expected_direction_camera =
      gtsam::Unit3(relative_pos_camera.x(), relative_pos_camera.y(),
                   relative_pos_camera.z());
  return true;
}

// Compute 2D bearing covariance for a single landmark (with optional pose
// covariance)
Eigen::Matrix2d expected_observation_cov(
    const std::shared_ptr<Landmark> &landmark,
    const Eigen::Vector3d &relative_pos_camera,
    const gtsam::Rot3 &camera_rotation,
    const std::optional<
        std::reference_wrapper<const Eigen::Matrix<double, 6, 6>>> &pose_cov,
    const Eigen::Matrix3d &min_landmark_covariance) {
  // Compute 2x3 bearing Jacobian wrt relative position in camera frame
  Eigen::Matrix<double, 2, 3> H = compute_bearing_jacobian(relative_pos_camera);

  // Rotate landmark covariance from world frame to camera frame
  Eigen::Matrix3d R_camera_from_world = Eigen::Map<const Eigen::Matrix3d>(
      camera_rotation.inverse().matrix().data());
  // Apply minimum covariance constraint for association (use max of actual and
  // minimum)
  Eigen::Matrix3d landmark_cov_for_association = landmark->covariance;
  for (int i = 0; i < 3; ++i) {
    landmark_cov_for_association(i, i) = std::max(
        landmark_cov_for_association(i, i), min_landmark_covariance(i, i));
  }
  Eigen::Matrix3d landmark_cov_camera = R_camera_from_world *
                                        landmark_cov_for_association *
                                        R_camera_from_world.transpose();

  // Propagate landmark position uncertainty: H * P_l * H^T (now in camera
  // frame)
  Eigen::Matrix2d cov = H * landmark_cov_camera * H.transpose();

  // Add pose contribution if provided
  if (pose_cov.has_value()) {
    // Pose covariance convention is [rot, trans] in camera-frame local
    // coordinates.
    const Eigen::Matrix<double, 6, 6> &P_pose_camera = pose_cov.value().get();

    // Relative position perturbation in camera frame:
    // δr_c ≈ -[r_c]_x δθ_c - δp_c
    const double rx = relative_pos_camera.x();
    const double ry = relative_pos_camera.y();
    const double rz = relative_pos_camera.z();
    Eigen::Matrix3d r_c_skew;
    r_c_skew << 0.0, -rz, ry, rz, 0.0, -rx, -ry, rx, 0.0;

    // Bearing Jacobian wrt pose perturbation [δθ_c, δp_c]
    // J_pose = H * [ -[r_c]_x  -I ]
    Eigen::Matrix<double, 2, 6> J_pose = Eigen::Matrix<double, 2, 6>::Zero();
    J_pose.block<2, 3>(0, 0) = H * (-r_c_skew);
    J_pose.block<2, 3>(0, 3) = -H;

    // Propagate full pose uncertainty (rotation + translation + cross terms)
    Eigen::Matrix2d cov_from_pose = J_pose * P_pose_camera * J_pose.transpose();

    // Add pose contribution to landmark covariance
    cov += cov_from_pose;
  }

  return cov;
}

std::tuple<bool, gtsam::Point3, double> triangulate_observations(
    const PolarObservation &obsA, const PolarObservation &obsB,
    double min_pose_baseline_m, double min_depth_m, double max_ray_distance_m,
    double min_parallax_threshold) {
  // Camera centers in world frame
  gtsam::Point3 originA_world = obsA.observation_pose.translation();
  gtsam::Point3 originB_world = obsB.observation_pose.translation();

  // GATE 0: Baseline Check
  if ((originA_world - originB_world).norm() < min_pose_baseline_m) {
    return {false, gtsam::Point3(), std::numeric_limits<double>::infinity()};
  }

  // Bearing rays in world frame: each camera-frame bearing is rotated by its
  // camera pose
  gtsam::Vector3 rayA_world = obsA.observation_pose.rotation() *
                              obsA.observation_direction.unitVector();
  gtsam::Vector3 rayB_world = obsB.observation_pose.rotation() *
                              obsB.observation_direction.unitVector();

  // Skew line intersection math (Closest Point of Approach)
  gtsam::Vector3 w0 = originA_world - originB_world;

  double a = rayA_world.dot(rayA_world);
  double b = rayA_world.dot(rayB_world);
  double c = rayB_world.dot(rayB_world);
  double d = rayA_world.dot(w0);
  double e = rayB_world.dot(w0);

  double denominator = a * c - b * b;

  // GATE 1: Parallax Check
  if (denominator < min_parallax_threshold) {
    return {false, gtsam::Point3(), std::numeric_limits<double>::infinity()};
  }

  // Calculate 's' and 't', which are the distances along ray 1 and ray 2 where
  // the closest approach occurs.
  double s = (b * e - c * d) / denominator;
  double t = (a * e - b * d) / denominator;

  // GATE 2: Cheirality (Behind-Camera) and min depth Check
  if (s <= min_depth_m || t <= min_depth_m) {
    return {false, gtsam::Point3(), std::numeric_limits<double>::infinity()};
  }

  // Calculate the specific 3D coordinates on both rays at the closest approach
  gtsam::Point3 P1 = originA_world + rayA_world * s;
  gtsam::Point3 P2 = originB_world + rayB_world * t;

  // Compute the midpoint between the two rays
  gtsam::Point3 triangulated_point = (P1 + P2) / 2.0;

  // The geometric error is the absolute physical distance between the rays
  double ray_distance = (P1 - P2).norm();

  // GATE 3: Intersection Error Check
  if (ray_distance > max_ray_distance_m) {
    return {false, gtsam::Point3(), ray_distance};
  }

  return {true, triangulated_point, ray_distance};
}

Eigen::Matrix3d
triangulate_observation_covariance(const PolarObservation &obsA,
                                   const PolarObservation &obsB) {

  // Camera centers in world frame
  gtsam::Point3 originA_world = obsA.observation_pose.translation();
  gtsam::Point3 originB_world = obsB.observation_pose.translation();

  // Observation directions are in each camera frame; rotate them to world frame
  // for triangulation geometry.
  const Eigen::Vector3d dirA_camera = obsA.observation_direction.unitVector();
  const Eigen::Vector3d dirB_camera = obsB.observation_direction.unitVector();
  gtsam::Vector3 rayA_world = obsA.observation_pose.rotation() * dirA_camera;
  gtsam::Vector3 rayB_world = obsB.observation_pose.rotation() * dirB_camera;

  // Tangent bases in camera frame (covariances are expressed in camera-frame
  // tangent space)
  Eigen::Vector3d basis_seed1 = (std::abs(dirA_camera.z()) < 0.9)
                                    ? Eigen::Vector3d::UnitZ()
                                    : Eigen::Vector3d::UnitX();
  Eigen::Vector3d u1_camera =
      (basis_seed1 - basis_seed1.dot(dirA_camera) * dirA_camera).normalized();
  Eigen::Vector3d v1_camera = dirA_camera.cross(u1_camera).normalized();

  Eigen::Vector3d basis_seed2 = (std::abs(dirB_camera.z()) < 0.9)
                                    ? Eigen::Vector3d::UnitZ()
                                    : Eigen::Vector3d::UnitX();
  Eigen::Vector3d u2_camera =
      (basis_seed2 - basis_seed2.dot(dirB_camera) * dirB_camera).normalized();
  Eigen::Vector3d v2_camera = dirB_camera.cross(u2_camera).normalized();

  // Precompute skew-line formula terms
  const gtsam::Vector3 w0 = originA_world - originB_world;
  const double a = rayA_world.dot(rayA_world);
  const double b = rayA_world.dot(rayB_world);
  const double c = rayB_world.dot(rayB_world);
  const double d = rayA_world.dot(w0);
  const double e = rayB_world.dot(w0);
  const double denom = a * c - b * b;

  if (std::abs(denom) < 1e-6) // Slightly safer threshold for parallel rays
  {
    return Eigen::Matrix3d::Identity() * 1e6;
  }

  const double s = (b * e - c * d) / denom;
  const double t = (a * e - b * d) / denom;

  // --- Deriving 3x1 Gradient Vectors w.r.t rayA_world ---
  const gtsam::Vector3 grad_denom_d1 =
      2.0 * c * rayA_world - 2.0 * b * rayB_world;

  const double num_s = b * e - c * d;
  const gtsam::Vector3 grad_num_s_d1 = e * rayB_world - c * w0;
  const gtsam::Vector3 grad_s_d1 =
      (denom * grad_num_s_d1 - num_s * grad_denom_d1) / (denom * denom);

  const double num_t = a * e - b * d;
  const gtsam::Vector3 grad_num_t_d1 =
      2.0 * e * rayA_world - b * w0 - d * rayB_world;
  const gtsam::Vector3 grad_t_d1 =
      (denom * grad_num_t_d1 - num_t * grad_denom_d1) / (denom * denom);

  // 3x3 Jacobian Matrix ∂P/∂d1
  const gtsam::Matrix3 dP_dd1_3d = 0.5 * (s * gtsam::Matrix3::Identity() +
                                          rayA_world * grad_s_d1.transpose() +
                                          rayB_world * grad_t_d1.transpose());

  // --- Deriving 3x1 Gradient Vectors w.r.t rayB_world ---
  const gtsam::Vector3 grad_denom_d2 =
      2.0 * a * rayB_world - 2.0 * b * rayA_world;

  const gtsam::Vector3 grad_num_s_d2 =
      b * w0 + e * rayA_world - 2.0 * d * rayB_world;
  const gtsam::Vector3 grad_s_d2 =
      (denom * grad_num_s_d2 - num_s * grad_denom_d2) / (denom * denom);

  const gtsam::Vector3 grad_num_t_d2 = a * w0 - d * rayA_world;
  const gtsam::Vector3 grad_t_d2 =
      (denom * grad_num_t_d2 - num_t * grad_denom_d2) / (denom * denom);

  // 3x3 Jacobian Matrix ∂P/∂d2
  const gtsam::Matrix3 dP_dd2_3d = 0.5 * (t * gtsam::Matrix3::Identity() +
                                          rayA_world * grad_s_d2.transpose() +
                                          rayB_world * grad_t_d2.transpose());

  // Rotate bearing covariances from camera frame to world frame after 2D->3D
  // expansion
  Eigen::Matrix3d RA = Eigen::Map<const Eigen::Matrix3d>(
      obsA.observation_pose.rotation().matrix().data());
  Eigen::Matrix3d RB = Eigen::Map<const Eigen::Matrix3d>(
      obsB.observation_pose.rotation().matrix().data());

  // Expand 2D bearing covariance to 3D in tangent space basis
  auto expand_2d_to_3d = [](const Eigen::Matrix2d &cov2d,
                            const Eigen::Vector3d &u,
                            const Eigen::Vector3d &v) -> Eigen::Matrix3d {
    Eigen::Matrix3d cov3d = Eigen::Matrix3d::Zero();
    cov3d += cov2d(0, 0) * u * u.transpose();
    cov3d += cov2d(0, 1) * u * v.transpose();
    cov3d += cov2d(1, 0) * v * u.transpose();
    cov3d += cov2d(1, 1) * v * v.transpose();
    return cov3d;
  };

  Eigen::Matrix3d cov_A_3d_camera = expand_2d_to_3d(
      obsA.observation_direction_covariance, u1_camera, v1_camera);
  Eigen::Matrix3d cov_B_3d_camera = expand_2d_to_3d(
      obsB.observation_direction_covariance, u2_camera, v2_camera);

  // Rotate to world frame: R_world_from_camera * Σ_camera *
  // R_world_from_camera^T
  Eigen::Matrix3d cov_A_3d_world = RA * cov_A_3d_camera * RA.transpose();
  Eigen::Matrix3d cov_B_3d_world = RB * cov_B_3d_camera * RB.transpose();

  // Propagate through Jacobians: Σ_P = J_A * Σ_A * J_A^T + J_B * Σ_B * J_B^T
  const Eigen::Matrix3d Cov_A_3d =
      dP_dd1_3d * cov_A_3d_world * dP_dd1_3d.transpose();
  const Eigen::Matrix3d Cov_B_3d =
      dP_dd2_3d * cov_B_3d_world * dP_dd2_3d.transpose();

  return (Cov_A_3d + Cov_B_3d);
}

} // namespace utils
} // namespace ptz_exploration_core