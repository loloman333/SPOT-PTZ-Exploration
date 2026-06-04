#ifndef PTZ_EXPLORATION_CORE__OPTIMIZER_HPP_
#define PTZ_EXPLORATION_CORE__OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <map>
#include <memory>
#include <optional>
#include <ptz_exploration_core/msg/detection_measurement_array.hpp>
#include <ptz_exploration_core/msg/landmark_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <set>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "utils/data_association.hpp"
#include "utils/math.hpp"

namespace ptz_exploration_core {

class Optimizer : public rclcpp::Node {
public:
  using PolarObservation = utils::PolarObservation;
  using Landmark = utils::Landmark;
  using PoseCovariance = Eigen::Matrix<double, 6, 6>;

  Optimizer();

private:
  // Core optimization
  void init();
  void handle_measurements(
      const std::vector<std::shared_ptr<PolarObservation>> new_observations,
      const gtsam::Pose3 &current_pose, const utils::CameraModel &camera_model);
  std::vector<std::shared_ptr<PolarObservation>>
  polar_observations_from_measurement_msg(
      const ptz_exploration_core::msg::DetectionMeasurementArray::SharedPtr msg,
      const gtsam::Pose3 &current_pose);
  int add_pose_to_graph(const gtsam::Pose3 &pose);
  void add_observations_to_graph(
      const std::vector<std::tuple<int, std::shared_ptr<PolarObservation>>>
          &observations,
      int pose_key);
  void handle_unpaired_observations(
      const std::vector<std::shared_ptr<PolarObservation>>
          &unpaired_observations,
      const gtsam::Pose3 &current_pose, const utils::CameraModel &camera_model,
      const std::optional<std::reference_wrapper<const PoseCovariance>>
          &pose_cov);
  void update_candidate_landmarks(
      const std::vector<std::shared_ptr<PolarObservation>>
          &unpaired_observations);
  int add_landmark_to_graph(const std::shared_ptr<Landmark> &landmark);
  void optimize_graph();
  void export_factor_graph_snapshot(const gtsam::Values &estimates);
  void publish_landmark_markers(const rclcpp::Time &stamp);
  void publish_landmarks(const rclcpp::Time &stamp);

  // Callbacks
  void measurement_callback(
      const ptz_exploration_core::msg::DetectionMeasurementArray::SharedPtr
          msg);
  void cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg,
                         const std::string &camera_name);

  // Parameters: frames and timing
  std::string robot_name_;
  std::string world_frame_;
  std::string landmarks_topic_;
  double tf_timeout_s_;

  // Parameters: optimizer and association
  double pose_prior_pos_sigma_m_;
  double pose_prior_rot_sigma_rad_;
  double isam_relinearize_threshold_;
  int isam_relinearize_skip_;
  bool export_factor_graph_enabled_;
  std::string factor_graph_export_path_;
  double data_association_alpha_;
  int candidate_confirmation_threshold_;
  std::vector<std::string> cameras_;
  std::string camera_info_pattern_;
  double similar_pose_translation_threshold_m_;
  double similar_pose_rotation_threshold_rad_;
  double similar_time_threshold_;

  // Parameters: triangulation
  double triangulation_min_pose_baseline_m_;
  double triangulation_min_depth_m_;
  double triangulation_max_ray_distance_m_;
  double triangulation_min_parallax_threshold_;

  // Parameters: visualization
  double viz_confidence_chi_squared_;
  double viz_max_ellipse_axis_m_;
  double viz_covariance_min_eigenvalue_;
  double min_covariance_radius_m_;
  Eigen::Matrix3d min_landmark_covariance_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Optimizer state
  bool initialized_;
  uint64_t pose_index_;
  uint64_t landmark_index_;

  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::noiseModel::Diagonal::shared_ptr pose_prior_noise_;
  PoseCovariance default_camera_uncertainty_;
  gtsam::NonlinearFactorGraph pending_factors_;
  gtsam::Values pending_values_;

  // Measurement and landmark state
  std::vector<std::shared_ptr<PolarObservation>> pending_measurements_;
  std::vector<std::shared_ptr<Landmark>> confirmed_landmarks_;
  std::vector<std::shared_ptr<Landmark>> candidate_landmarks_;
  std::map<int, std::vector<std::shared_ptr<PolarObservation>>>
      candidate_landmark_supporters_map_;

  // Last processed transform per camera frame
  std::map<std::string, geometry_msgs::msg::TransformStamped>
      last_processed_measurement_tf_map_;

  // PTZ camera tracking: previous focal lengths and zoom change detection
  std::map<std::string, double>
      previous_focal_length_; // focal_x from previous CameraInfo
  std::map<std::string, bool>
      camera_info_changed_; // true if zoom detected since last measurement

  // Subscribers
  rclcpp::Subscription<ptz_exploration_core::msg::DetectionMeasurementArray>::
      SharedPtr measurement_sub_;
  std::map<std::string, utils::CameraModel> camera_models_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> cam_info_subs_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      landmark_marker_pub_;
  rclcpp::Publisher<ptz_exploration_core::msg::LandmarkArray>::SharedPtr
      landmarks_pub_;

private:
  // Helper structs for factor graph export
  struct ExportPoseNode {
    int id;
    double x;
    double y;
    double theta;
  };

  struct ExportLandmarkNode {
    int id;
    double x;
    double y;
  };

  struct ExportEdge {
    std::string from_type;
    int from_id;
    std::string to_type;
    int to_id;
    std::string factor_type;
  };

  // Helper functions for JSON export
  static std::string escape_json_string(const std::string &input);
  static std::string factor_type_name(const gtsam::NonlinearFactor &factor);
};

} // namespace ptz_exploration_core

#endif // PTZ_EXPLORATION_CORE__OPTIMIZER_HPP_
