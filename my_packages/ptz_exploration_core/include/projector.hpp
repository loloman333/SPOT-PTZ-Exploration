#ifndef PTZ_EXPLORATION_CORE__PROJECTOR_HPP_
#define PTZ_EXPLORATION_CORE__PROJECTOR_HPP_

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <ptz_exploration_core/msg/detection_measurement.hpp>
#include <ptz_exploration_core/msg/detection_measurement_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <utils/math.hpp>

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <octomap/octomap.h>
#include <string>
#include <vector>

namespace ptz_exploration_core {

class Projector : public rclcpp::Node {
public:
  Projector();
  virtual ~Projector() = default;

private:
  // Callbacks
  void cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg,
                         const std::string &camera_name);
  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void
  detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  // Parameters
  std::string robot_name_;
  std::vector<std::string> cameras_;
  std::string camera_info_pattern_;
  std::string world_frame_;
  std::string base_frame_;
  std::string detections_topic_;
  std::string octomap_topic_;
  std::string detection_measurements_topic_;
  std::string detection_debug_markers_topic_;
  int tf_lookup_timeout_ms_;
  double bearing_norm_epsilon_;
  double max_ray_distance_;
  double bbox_center_confidence_multiplier_;
  double min_bearing_sigma_rad_;
  double range_sigma_multiplier_;
  double min_range_sigma_m_;
  double octomap_min_translation_m_;
  double octomap_min_rotation_rad_;
  rclcpp::Duration octomap_update_interval_{rclcpp::Duration(0, 0)};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Octomap
  std::shared_ptr<octomap::OcTree> latest_octomap_;
  geometry_msgs::msg::TransformStamped last_octomap_tf_;
  bool has_last_octomap_pose_{false};

  // Publishers
  rclcpp::Publisher<ptz_exploration_core::msg::DetectionMeasurementArray>::
      SharedPtr measurement_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      debug_marker_pub_;

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
      detection_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  // Camera intrinsics cache
  std::map<std::string, utils::CameraModel> camera_models_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> cam_info_subs_;
};

} // namespace ptz_exploration_core

#endif // PTZ_EXPLORATION_CORE__PROJECTOR_HPP_
