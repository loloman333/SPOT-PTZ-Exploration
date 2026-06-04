#ifndef PTZ_EXPLORATION_CORE__UTILS__DATA_ASSOCIATION_HPP_
#define PTZ_EXPLORATION_CORE__UTILS__DATA_ASSOCIATION_HPP_

#include <vector>
#include <memory>
#include <utility>
#include <cstdint>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Point3.h>
#include "utils/math.hpp"

namespace ptz_exploration_core
{
namespace utils
{

struct PolarObservation
{
    std::string class_name;
    double confidence;
    gtsam::Pose3 observation_pose;
    gtsam::Unit3 observation_direction;
    Eigen::Matrix2d observation_direction_covariance;  // 2x2 tangent-space (azimuth, elevation)
    double range; // Optional range measurement (-1 if unavailable)
    double range_covariance; // Optional range measurement uncertainty (m), used for gating and triangulation
};

struct Landmark
{
    uint32_t id;
    std::string class_name;
    double confidence;
    gtsam::Point3 position;
    Eigen::Matrix3d covariance;
};

/**
 * Compute 2x3 bearing Jacobian (azimuth, elevation) with respect to 3D relative position vector.
 * 
 * @param relative_pos Vector from observer to target (l - p)
 * @return 2x3 matrix: [∂az/∂v; ∂el/∂v]
 */
Eigen::Matrix<double, 2, 3> compute_bearing_jacobian(const Eigen::Vector3d &relative_pos);

/**
 * Compute squared Mahalanobis distance between two bearing observations in 2D tangent space.
 * 
 * @param o1 First observation
 * @param o2 Second observation
 * @return Squared Mahalanobis distance
 */
double squaredMahalanobisDistance(const PolarObservation &o1, const PolarObservation &o2);

/**
 * Check individual compatibility using chi-squared test.
 * 
 * @param D2_ij Squared Mahalanobis distance
 * @param dof Degrees of freedom (should be 2 for bearing)
 * @param alpha Confidence level quantile
 * @return true if compatible
 */
bool individualCompatibility(double D2_ij, int dof, double alpha);

/**
 * Individual Compatibility Nearest Neighbor (ICNN) data association.
 * 
 * @param new_observations New bearing observations
 * @param landmarks Landmarks to associate against
 * @param camera_pose Camera pose used to form expected observations
 * @param camera_model Pinhole camera model used for FoV gating
 * @param pose_cov Optional 6x6 pose covariance. If nullopt, uses landmark covariance only.
 * @param alpha Confidence level quantile
 * @param min_landmark_covariance Minimum landmark covariance constraint for association
 * @param paired_observations Output: vector of (landmark_index, observation) pairs
 * @param unpaired_observations Output: vector of observations with no match
 */
void icnn(
    const std::vector<std::shared_ptr<PolarObservation>> &new_observations, 
     const std::vector<std::shared_ptr<Landmark>> &landmarks,
     const gtsam::Pose3 &camera_pose,
     const CameraModel &camera_model,
     const std::optional<std::reference_wrapper<const Eigen::Matrix<double, 6, 6>>> &pose_cov,
    double alpha,
    const Eigen::Matrix3d &min_landmark_covariance,
    std::vector<std::tuple<int, std::shared_ptr<PolarObservation>>> &paired_observations,
    std::vector<std::shared_ptr<PolarObservation>> &unpaired_observations);

/**
 * Compute 2D bearing covariance for a single landmark.
 * 
 * @param landmark Landmark with position and covariance (in world frame)
 * @param relative_pos_camera Vector from camera to landmark in camera frame
 * @param world_from_camera_rotation Camera pose rotation (camera to world transform).
 *        The implementation internally uses its inverse for world->camera covariance rotation.
 * @param pose_cov Optional 6x6 pose covariance ([rot, trans]) in camera-frame local coordinates.
 *        If provided, adds observer rotation + position uncertainty (including cross terms).
 * @param min_landmark_covariance Minimum landmark covariance constraint for association
 * @return 2x2 covariance in bearing tangent space (camera frame)
 */
Eigen::Matrix2d expected_observation_cov(
    const std::shared_ptr<Landmark> &landmark,
    const Eigen::Vector3d &relative_pos_camera,
    const gtsam::Rot3 &world_from_camera_rotation,
    const std::optional<std::reference_wrapper<const Eigen::Matrix<double, 6, 6>>> &pose_cov,
    const Eigen::Matrix3d &min_landmark_covariance);

/**
 * Compute expected bearing mean for a single landmark and apply pinhole FoV gating.
 * 
 * @param camera_pose Camera pose
 * @param camera_model Pinhole camera model used for image-plane FoV gate
 * @param landmark Landmark to project
 * @param relative_pos_camera Output relative position from camera to landmark in camera frame
 * @param expected_direction_camera Output expected bearing direction in camera frame
 * @return true if landmark is in front of camera and projects into image bounds
 */
bool expected_observation_mean(
    const gtsam::Pose3 &camera_pose,
    const CameraModel &camera_model,
    const std::shared_ptr<Landmark> &landmark,
    Eigen::Vector3d &relative_pos_camera,
    gtsam::Unit3 &expected_direction_camera);

/**
 * Triangulate a 3D landmark position from two bearing observations.
 * 
 * @param obsA First bearing observation with pose
 * @param obsB Second bearing observation with pose
 * @param min_pose_baseline_m Minimum translation between observation poses (m)
 * @param min_depth_m Minimum forward depth along each ray from its camera origin (m)
 * @param max_ray_distance_m Maximum allowed ray intersection error (m)
 * @param min_parallax_threshold Minimum parallax denominator threshold
 * @return Tuple of (success, triangulated_point, ray_distance)
 *         - success: true if triangulation passed all gates
 *         - triangulated_point: midpoint of closest ray approach
 *         - ray_distance: geometric error (distance between rays)
 */
std::tuple<bool, gtsam::Point3, double> triangulate_observations(
    const PolarObservation &obsA,
    const PolarObservation &obsB,
    double min_pose_baseline_m,
    double min_depth_m,
    double max_ray_distance_m,
    double min_parallax_threshold);

/**
 * Compute 3D position covariance for a triangulated landmark by propagating
 * bearing observation uncertainties through analytical Jacobians.
 * 
 * Uses first-order error propagation with information matrix fusion:
 * - Computes analytical Jacobians ∂P/∂bearing for each observation
 * - Maps 2D bearing covariances to 3D position space via chain rule
 * - Combines information matrices: I_total = I_A^-1 + I_B^-1
 * - Returns final covariance: Cov = I_total^-1
 * 
 * @param obsA First bearing observation with 2x2 tangent-space covariance
 * @param obsB Second bearing observation with 2x2 tangent-space covariance
 * @return 3x3 position covariance matrix in world frame
 */
Eigen::Matrix3d triangulate_observation_covariance(
    const PolarObservation &obsA,
    const PolarObservation &obsB);

} // namespace utils
} // namespace ptz_exploration_core

#endif // PTZ_EXPLORATION_CORE__UTILS__DATA_ASSOCIATION_HPP_
