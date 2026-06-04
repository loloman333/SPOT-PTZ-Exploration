#ifndef PTZ_EXPLORATION_CORE__GAZER_HPP_
#define PTZ_EXPLORATION_CORE__GAZER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <ptz_exploration_core/msg/landmark_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <utils/math.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include <octomap/OcTree.h>

namespace ptz_exploration_core {

class Gazer : public rclcpp::Node {
public:
  Gazer();

private:
  // Callbacks
  void target_callback(
      const geometry_msgs::msg::PointStamped::SharedPtr target_point);
  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void landmarks_callback(
      const ptz_exploration_core::msg::LandmarkArray::SharedPtr msg);
  void
  detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void ptz_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void ptz_settled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg,
                            const std::string &camera_name);

  // Services / scan helpers
  bool confirm_landmarks();
  void adjust_zoom_to_active_landmark();
  void handle_scan_service(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_confirm_service(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Gaze helpers
  bool landmark_is_visible(const ptz_exploration_core::msg::Landmark &landmark,
                           const octomap::OcTree &octomap,
                           const geometry_msgs::msg::Point &robot_point) const;
  void publish_target_point(const geometry_msgs::msg::Point &point);
  void publish_ptz_command(double pan, double tilt, double zoom);
  void command_home_pose();
  bool wait_until_home();
  bool get_latest_ptz_state_snapshot(geometry_msgs::msg::Point &state);
  bool ptz_state_is_home(const geometry_msgs::msg::Point &state) const;
  bool
  select_largest_detection(const vision_msgs::msg::Detection2DArray &detections,
                           vision_msgs::msg::Detection2D &best_detection) const;
  double compute_max_zoom_for_full_bbox_in_frame(
      const vision_msgs::msg::Detection2D &detection,
      const utils::CameraModel &cam_model, double reference_zoom) const;
  // Core
  std::tuple<double, double, double>
  compute_ptz_cmd_from_point(const geometry_msgs::msg::Point &point) const;

  // Parameters
  std::string ptz_cmd_topic_;
  std::string target_point_topic_;
  std::string octomap_topic_;
  std::string ptz_state_topic_;
  std::string ptz_settled_topic_;
  std::string landmarks_topic_;
  std::string detections_topic_;
  std::string camera_info_pattern_;
  std::string camera_;
  std::string robot_name_;
  std::string ptz_origin_frame_;
  std::string gaze_frame_;
  std::string world_frame_;
  std::string base_frame_;
  int tf_lookup_timeout_ms_;
  double octomap_min_translation_m_;
  double octomap_min_rotation_rad_;
  rclcpp::Duration octomap_update_interval_{rclcpp::Duration(0, 0)};
  double landmark_dismiss_radius_m_;
  int scan_num_stops_;
  double scan_dwell_time_s_;
  double scan_home_tolerance_rad_;
  double scan_home_tolerance_zoom_;
  double scan_zoom_{1.0};
  double confirm_threshold_;
  int turning_direction_;
  double home_pan_offset_rad_;
  double home_tilt_offset_rad_;
  double home_zoom_offset_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Octomap
  std::shared_ptr<octomap::OcTree> latest_octomap_;
  geometry_msgs::msg::TransformStamped last_octomap_tf_;
  bool has_last_octomap_pose_{false};
  ptz_exploration_core::msg::LandmarkArray::SharedPtr latest_landmarks_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
  std::map<std::string, utils::CameraModel> camera_models_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> cam_info_subs_;
  std::mutex mutex_;

  // Active auto target
  double current_zoom_{1.0};

  size_t next_landmark_index_{0};

  // PTZ state
  geometry_msgs::msg::Point latest_ptz_state_;
  bool latest_ptz_settled_{false};
  bool has_latest_ptz_state_{false};

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      clicked_point_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<ptz_exploration_core::msg::LandmarkArray>::SharedPtr
      landmarks_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
      detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ptz_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ptz_settled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      clicked_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ptz_cmd_pub_;

  rclcpp::TimerBase::SharedPtr zoom_adjust_timer_;

  // Callback groups (prevent long service callbacks from starving
  // subscriptions)
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;

  // Service servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_service_;
};

} // namespace ptz_exploration_core

#endif // PTZ_EXPLORATION_CORE__GAZER_HPP_
