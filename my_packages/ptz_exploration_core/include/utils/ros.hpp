#ifndef PTZ_EXPLORATION_CORE__UTILS__ROS_HPP_
#define PTZ_EXPLORATION_CORE__UTILS__ROS_HPP_

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include <gtsam/geometry/Pose3.h>
#include <octomap/OcTree.h>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <map>
#include <functional>

#include "utils/math.hpp"

namespace ptz_exploration_core::utils
{

struct DetectionClassification
{
  float confidence;
  std::string class_name;
};

/**
 * @brief Extract camera name from ROS frame ID
 * @param frame_id Frame identifier string
 * @param cameras List of valid camera names to match against
 * @return Matched camera name, or empty string if not found
 */
std::string extract_camera_from_frame(
  const std::string & frame_id,
  const std::vector<std::string> & cameras);

/**
 * @brief Replace placeholder patterns in a string
 * @param pattern String with placeholders like {robot} and {camera}
 * @param robot_name Robot name to substitute for {robot}
 * @param camera_name Camera name to substitute for {camera}
 * @return Evaluated string with substitutions applied
 */
std::string evaluate_pattern(
  const std::string & pattern,
  const std::string & robot_name,
  const std::string & camera_name);

/**
 * @brief Initialize camera_info subscribers for configured cameras.
 *
 * Creates one subscription per camera using @p camera_info_pattern and stores it in
 * @p cam_info_subs. The callback receives both the message and the resolved camera name.
 */
void initialize_camera_info_subscribers(
  rclcpp::Node & node,
  const std::vector<std::string> & cameras,
  const std::string & camera_info_pattern,
  const std::string & robot_name,
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> & cam_info_subs,
  std::function<void(const sensor_msgs::msg::CameraInfo::SharedPtr, const std::string &)> callback);

/**
 * @brief Update camera model cache from CameraInfo.
 *
 * Always writes latest intrinsics/resolution for given camera.
 */
void cache_camera_model_from_info(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg,
  const std::string & camera_name,
  std::map<std::string, CameraModel> & camera_models,
  const rclcpp::Logger & logger);

/**
 * @brief Create a DELETEALL marker for clearing previous visualization state.
 */
visualization_msgs::msg::Marker make_delete_all_marker(
  const std::string & frame_id,
  const rclcpp::Time & stamp);

/**
 * @brief Create a SPHERE_LIST marker for landmark point centers.
 */
visualization_msgs::msg::Marker make_landmark_points_marker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double point_scale_m,
  float r,
  float g,
  float b,
  float a);

/**
 * @brief Create an oriented covariance ellipsoid marker (type SPHERE).
 */
visualization_msgs::msg::Marker make_covariance_ellipsoid_marker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  int marker_id,
  double px,
  double py,
  double pz,
  double qx,
  double qy,
  double qz,
  double qw,
  double scale_x,
  double scale_y,
  double scale_z,
  float r,
  float g,
  float b,
  double alpha);

/**
 * @brief Convert a TF transform message into a gtsam::Pose3.
 */
gtsam::Pose3 tf_to_pose(const geometry_msgs::msg::TransformStamped & tf);

/**
 * @brief Get the highest-confidence class hypothesis from a 2D detection.
 */
DetectionClassification best_detection_classification(
  const vision_msgs::msg::Detection2D & detection);

/**
 * @brief Apply octomap update gates (time + pose delta) and deserialize into OcTree.
 *
 * If a gate blocks the update, returns nullptr. On successful update, updates
 * @p last_octomap_tf and @p has_last_octomap_pose and returns the new OcTree.
 */
std::shared_ptr<octomap::OcTree> deserialize_octomap_msg(
  const octomap_msgs::msg::Octomap & msg,
  tf2_ros::Buffer & tf_buffer,
  const std::string & world_frame,
  const std::string & base_frame,
  int tf_lookup_timeout_ms,
  double octomap_min_translation_m,
  double octomap_min_rotation_rad,
  const rclcpp::Duration & octomap_update_interval,
  geometry_msgs::msg::TransformStamped & last_octomap_tf,
  bool & has_last_octomap_pose,
  const rclcpp::Time & current_time,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

/**
 * @brief Create a debug arrow marker for a bearing vector in base frame.
 */
visualization_msgs::msg::Marker make_bearing_arrow_marker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  int marker_id,
  const geometry_msgs::msg::Point & p0,
  const geometry_msgs::msg::Point & p1,
  double shaft_diameter,
  double head_diameter,
  double head_length,
  float r,
  float g,
  float b,
  float a,
  const rclcpp::Duration & lifetime);

/**
 * @brief Create bearing debug markers (main arrow + 4 covariance edge arrows).
 *
 * Computes p0 and p1 from camera origin and ray direction. Colors determined by octomap hit.
 * The 4 edge arrows represent the 95% confidence interval edges in the 2D tangent space
 * using chi-squared scaling (default: 5.991 for 2 DoF).
 */
std::vector<visualization_msgs::msg::Marker> create_bearing_debug_markers(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  int marker_id_base,
  const Eigen::Vector3d & cam_origin,
  const Eigen::Vector3d & ray_map,
  const octomap::point3d & hit_point,
  bool hit,
  double sigma_azimuth_rad,
  double sigma_elevation_rad,
  double shaft_diameter,
  double head_diameter,
  double head_length,
  const rclcpp::Duration & lifetime,
  double confidence_chi_squared_2d = 5.991,
  float covariance_alpha = 0.35f);

}  // namespace ptz_exploration_core::utils

#endif  // PTZ_EXPLORATION_CORE__UTILS__ROS_HPP_
