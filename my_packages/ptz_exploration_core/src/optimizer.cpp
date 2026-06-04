#include "optimizer.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/exceptions.h>
#include <visualization_msgs/msg/marker.hpp>

#include "utils/data_association.hpp"
#include "utils/math.hpp"
#include "utils/ros.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

namespace ptz_exploration_core {

namespace {
constexpr double kChiSquared99Percent3D = 11.344866730144373;
}

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

std::string Optimizer::escape_json_string(const std::string &input) {
  std::ostringstream output;
  for (const char c : input) {
    switch (c) {
    case '\\':
      output << "\\\\";
      break;
    case '"':
      output << "\\\"";
      break;
    case '\b':
      output << "\\b";
      break;
    case '\f':
      output << "\\f";
      break;
    case '\n':
      output << "\\n";
      break;
    case '\r':
      output << "\\r";
      break;
    case '\t':
      output << "\\t";
      break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
               << static_cast<int>(static_cast<unsigned char>(c)) << std::dec
               << std::setfill(' ');
      } else {
        output << c;
      }
      break;
    }
  }
  return output.str();
}

std::string Optimizer::factor_type_name(const gtsam::NonlinearFactor &factor) {
  const auto key_count = factor.keys().size();
  if (key_count == 1) {
    return "prior";
  }
  if (key_count == 2) {
    return "binary";
  }
  return "other";
}

Optimizer::Optimizer()
    : Node("optimizer"), initialized_(false), pose_index_(0),
      landmark_index_(0) {
  // Declare parameters
  // Core frames / TF
  this->declare_parameter<std::string>("robot_name", "spot");
  this->declare_parameter<std::string>("world_frame", "map");
  this->declare_parameter<std::string>("landmarks_topic", "/landmarks");

  // Triangulation
  // Triangulation: minimum pose baseline (translation) required between
  // observations (m).
  this->declare_parameter<double>("triangulation_min_pose_baseline_m", 0.5);
  // Triangulation: minimum valid forward depth along each observation ray (m).
  this->declare_parameter<double>("triangulation_min_depth_m", 0.0);
  // Triangulation: maximum allowed ray intersection error (m).
  this->declare_parameter<double>("triangulation_max_ray_distance_m", 0.5);
  // Triangulation: minimum parallax threshold (denominator in skew line math).
  this->declare_parameter<double>("triangulation_min_parallax_threshold",
                                  0.001);

  // Association
  // ICNN compatibility quantile alpha (chi-squared CDF value).
  this->declare_parameter<double>("data_association_alpha", 0.7f);
  // Number of compatible observations required before promoting a tentative
  // landmark.
  this->declare_parameter<int>("candidate_confirmation_threshold", 2);
  // CameraInfo subscriptions for ICNN expected-observation FoV gating.
  this->declare_parameter<std::vector<std::string>>(
      "cameras", std::vector<std::string>{""});
  this->declare_parameter<std::string>("camera_info_pattern", "");

  // Message filtering
  // Pose similarity threshold (m) for skipping near-duplicate measurement
  // updates.
  this->declare_parameter<double>("similar_pose_translation_threshold_m", 0.5);
  // Pose rotation similarity threshold (rad) for skipping near-duplicate
  // measurement updates.
  this->declare_parameter<double>("similar_pose_rotation_threshold_rad", 0.05);
  // Timestamp similarity threshold (s) for skipping near-duplicate camera
  // messages.
  this->declare_parameter<double>("similar_time_threshold", 0.5);

  // Optimizer (graph / solver)
  this->declare_parameter<double>("pose_prior_pos_sigma_m", 0.0001);
  this->declare_parameter<double>("pose_prior_rot_sigma_rad", 0.0001);
  this->declare_parameter<double>("min_covariance_radius_m", 0.5);
  this->declare_parameter<bool>("export_factor_graph", true);
  this->declare_parameter<std::string>(
      "factor_graph_export_path",
      "/home/appuser/ros2_ws/data/factor_graph/current_run.json");
  // iSAM2 relinearization threshold.
  this->declare_parameter<double>("isam_relinearize_threshold", 0.1);
  // iSAM2 relinearization skip interval in updates.
  this->declare_parameter<int>("isam_relinearize_skip", 1);

  // Get parameters
  // Core frames / TF
  this->robot_name_ = this->get_parameter("robot_name").as_string();
  this->world_frame_ = this->get_parameter("world_frame").as_string();
  this->landmarks_topic_ = this->get_parameter("landmarks_topic").as_string();
  this->tf_timeout_s_ = 0.1;

  // Visualization
  this->viz_confidence_chi_squared_ =
      5.991; // 95% chi-squared for 2D confidence ellipse scaling
  this->viz_max_ellipse_axis_m_ =
      100.0; // Maximum ellipsoid axis length for RViz clipping
  this->viz_covariance_min_eigenvalue_ =
      1e-6; // Numerical stability floor for covariance eigendecomposition

  // Triangulation
  this->triangulation_min_pose_baseline_m_ =
      this->get_parameter("triangulation_min_pose_baseline_m").as_double();
  this->triangulation_min_depth_m_ =
      this->get_parameter("triangulation_min_depth_m").as_double();
  this->triangulation_max_ray_distance_m_ =
      this->get_parameter("triangulation_max_ray_distance_m").as_double();
  this->triangulation_min_parallax_threshold_ =
      this->get_parameter("triangulation_min_parallax_threshold").as_double();

  // Association
  this->data_association_alpha_ =
      this->get_parameter("data_association_alpha").as_double();
  this->candidate_confirmation_threshold_ =
      this->get_parameter("candidate_confirmation_threshold").as_int();
  this->cameras_ = this->get_parameter("cameras").as_string_array();
  this->camera_info_pattern_ =
      this->get_parameter("camera_info_pattern").as_string();

  // Message filtering
  this->similar_pose_translation_threshold_m_ =
      this->get_parameter("similar_pose_translation_threshold_m").as_double();
  this->similar_pose_rotation_threshold_rad_ =
      this->get_parameter("similar_pose_rotation_threshold_rad").as_double();
  this->similar_time_threshold_ =
      this->get_parameter("similar_time_threshold").as_double();

  // Optimizer (graph / solver)
  this->pose_prior_pos_sigma_m_ =
      this->get_parameter("pose_prior_pos_sigma_m").as_double();
  this->pose_prior_rot_sigma_rad_ =
      this->get_parameter("pose_prior_rot_sigma_rad").as_double();
  this->min_covariance_radius_m_ =
      this->get_parameter("min_covariance_radius_m").as_double();
  this->export_factor_graph_enabled_ =
      this->get_parameter("export_factor_graph").as_bool();
  this->factor_graph_export_path_ =
      this->get_parameter("factor_graph_export_path").as_string();
  this->isam_relinearize_threshold_ =
      this->get_parameter("isam_relinearize_threshold").as_double();
  this->isam_relinearize_skip_ =
      this->get_parameter("isam_relinearize_skip").as_int();

  // Initialize TF
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  // Initialize optimizer internals
  this->init();

  RCLCPP_INFO(this->get_logger(), "Optimizer initialized for robot: %s",
              this->robot_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Factor graph JSON export: %s (%s)",
              this->export_factor_graph_enabled_ ? "enabled" : "disabled",
              this->factor_graph_export_path_.c_str());

  // Subscribe to detection measurements
  this->measurement_sub_ = this->create_subscription<
      ptz_exploration_core::msg::DetectionMeasurementArray>(
      "/detection_measurements", 10,
      std::bind(&Optimizer::measurement_callback, this, std::placeholders::_1));

  utils::initialize_camera_info_subscribers(
      *this, this->cameras_, this->camera_info_pattern_, this->robot_name_,
      this->cam_info_subs_,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg,
             const std::string &camera) {
        this->cam_info_callback(msg, camera);
      });

  // Publisher for optimized landmark markers with covariance
  this->landmark_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/landmark_markers", 10);

  // Publisher for optimized landmarks (id, class, confidence, position)
  this->landmarks_pub_ =
      this->create_publisher<ptz_exploration_core::msg::LandmarkArray>(
          this->landmarks_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Optimizer node ready");
}

void Optimizer::init() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = this->isam_relinearize_threshold_;
  params.relinearizeSkip = this->isam_relinearize_skip_;

  this->isam_ = std::make_unique<gtsam::ISAM2>(params);

  gtsam::Vector6 pose_sigmas;
  pose_sigmas << this->pose_prior_rot_sigma_rad_,
      this->pose_prior_rot_sigma_rad_, this->pose_prior_rot_sigma_rad_,
      this->pose_prior_pos_sigma_m_, this->pose_prior_pos_sigma_m_,
      this->pose_prior_pos_sigma_m_;

  this->pose_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas);
  this->default_camera_uncertainty_.setZero();
  this->default_camera_uncertainty_.diagonal() =
      pose_sigmas.array().square().matrix();

  const double min_covariance_variance =
      (this->min_covariance_radius_m_ * this->min_covariance_radius_m_) /
      kChiSquared99Percent3D;
  this->min_landmark_covariance_.setIdentity();
  this->min_landmark_covariance_ *= min_covariance_variance;

  this->pose_index_ = 0;
  this->landmark_index_ = 0;
  this->pending_factors_.resize(0);
  this->pending_values_.clear();
  this->initialized_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Initialized factor graph and iSAM2 optimizer");
}

void Optimizer::cam_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg,
    const std::string &camera_name) {
  // Always refresh camera model (PTZ intrinsics can change with zoom).
  utils::cache_camera_model_from_info(msg, camera_name, this->camera_models_,
                                      this->get_logger());

  // Detect zoom level change by focal length delta.
  const double current_focal_length = msg->k[0]; // fx in K
  const auto prev_it = this->previous_focal_length_.find(camera_name);

  if (prev_it != this->previous_focal_length_.end()) {
    if (prev_it->second != current_focal_length) {
      this->camera_info_changed_[camera_name] = true;
      RCLCPP_DEBUG(
          this->get_logger(),
          "Zoom change detected for camera '%s': focal_length %.2f → %.2f",
          camera_name.c_str(), prev_it->second, current_focal_length);
    }
  }

  // Cache latest focal length.
  this->previous_focal_length_[camera_name] = current_focal_length;

  // Keep subscriber active (no unsubscribe).
}

void Optimizer::measurement_callback(
    const ptz_exploration_core::msg::DetectionMeasurementArray::SharedPtr msg) {
  if (msg->measurements.empty()) {
    return;
  }
  const std::string camera_frame = msg->header.frame_id;

  // Resolve camera model for this frame from camera_info subscriptions.
  const std::string camera =
      utils::extract_camera_from_frame(camera_frame, this->cameras_);
  if (camera.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Cannot resolve camera name from frame '%s'",
                         camera_frame.c_str());
    return;
  }

  const auto cam_it = this->camera_models_.find(camera);
  if (cam_it == this->camera_models_.end()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Camera model not ready for '%s'. Waiting for CameraInfo.",
        camera.c_str());
    return;
  }
  const utils::CameraModel &association_camera_model = cam_it->second;
  const bool zoom_changed = this->camera_info_changed_[camera];

  // Discard messages that are too close in time
  const auto prev_it =
      this->last_processed_measurement_tf_map_.find(camera_frame);
  if (prev_it != this->last_processed_measurement_tf_map_.end()) {
    const auto &prev_tf = prev_it->second;
    const double dt_s = std::abs(
        (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_tf.header.stamp))
            .seconds());
    if (dt_s < this->similar_time_threshold_) {
      return;
    }
  }

  // Get camera pose for this measurement batch
  geometry_msgs::msg::TransformStamped current_tf;
  try {
    current_tf = this->tf_buffer_->lookupTransform(
        this->world_frame_, camera_frame, msg->header.stamp,
        rclcpp::Duration::from_seconds(this->tf_timeout_s_));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "TF lookup failed for camera '%s' at t=%d.%d: %s",
                camera_frame.c_str(), msg->header.stamp.sec,
                msg->header.stamp.nanosec, ex.what());
    return;
  }

  // Discard messages with similar camera pose to last processed message for
  // this camera
  const gtsam::Pose3 current_camera_pose = utils::tf_to_pose(current_tf);

  if (prev_it != this->last_processed_measurement_tf_map_.end()) {
    const gtsam::Pose3 prev_pose = utils::tf_to_pose(prev_it->second);
    // Bypass pose gate if zoom changed (to allow re-observation at same pose
    // with new intrinsics)
    if (!zoom_changed &&
        utils::poses_similar(prev_pose, current_camera_pose,
                             this->similar_pose_translation_threshold_m_,
                             this->similar_pose_rotation_threshold_rad_)) {
      return;
    }
  }

  // Update last processed TF for this camera
  this->last_processed_measurement_tf_map_[camera_frame] = current_tf;

  // Convert measurements to internal PolarObservation representation
  auto new_observations =
      polar_observations_from_measurement_msg(msg, current_camera_pose);

  // Handle measurements: data association, graph update, optimization
  this->handle_measurements(new_observations, current_camera_pose,
                            association_camera_model);

  // Clear zoom-change gate after first processed batch at new zoom.
  if (zoom_changed) {
    this->camera_info_changed_[camera] = false;
  }
}

void Optimizer::handle_measurements(
    const std::vector<std::shared_ptr<PolarObservation>> new_observations,
    const gtsam::Pose3 &camera_pose, const utils::CameraModel &camera_model) {
  if (!initialized_ || !isam_) {
    RCLCPP_WARN(this->get_logger(),
                "Optimizer not initialized. Dropping measurement batch.");
    return;
  }

  // Match new observations against existing landmarks using ICNN data
  // association
  std::vector<std::tuple<int, std::shared_ptr<PolarObservation>>>
      paired_observations;
  std::vector<std::shared_ptr<PolarObservation>> unpaired_observations;
  utils::icnn(new_observations, this->confirmed_landmarks_, camera_pose,
              camera_model, std::cref(this->default_camera_uncertainty_),
              this->data_association_alpha_, this->min_landmark_covariance_,
              paired_observations, unpaired_observations);

  // RCLCPP_INFO(this->get_logger(), "Paired observations: %zu, Unpaired
  // observations: %zu", paired_observations.size(),
  // unpaired_observations.size());

  // Add camera pose prior and all paired bearing observations to graph
  if (!paired_observations.empty()) {
    for (const auto &entry : paired_observations) {
      const int landmark_index = std::get<0>(entry);
      const auto &observation = std::get<1>(entry);

      this->confirmed_landmarks_[landmark_index]->confidence =
          std::max(this->confirmed_landmarks_[landmark_index]->confidence,
                   observation->confidence);
    }

    int pose_key = this->add_pose_to_graph(camera_pose);
    this->add_observations_to_graph(paired_observations, pose_key);
  }

  // Handle unpaired observations
  if (!unpaired_observations.empty()) {
    this->handle_unpaired_observations(
        unpaired_observations, camera_pose, camera_model,
        std::cref(this->default_camera_uncertainty_));
  }

  if (!this->pending_factors_.empty()) {
    this->optimize_graph();
  }

  const auto stamp = this->now();
  this->publish_landmark_markers(stamp);
  this->publish_landmarks(stamp);
}

void Optimizer::publish_landmarks(const rclcpp::Time &stamp) {
  ptz_exploration_core::msg::LandmarkArray landmarks_msg;
  landmarks_msg.header.stamp = stamp;
  landmarks_msg.header.frame_id = this->world_frame_;
  landmarks_msg.landmarks.reserve(this->confirmed_landmarks_.size());

  for (const auto &landmark : this->confirmed_landmarks_) {
    if (!landmark) {
      continue;
    }

    ptz_exploration_core::msg::Landmark msg_landmark;
    msg_landmark.id = landmark->id;
    msg_landmark.class_name = landmark->class_name;
    msg_landmark.confidence =
        std::clamp(static_cast<float>(landmark->confidence), 0.0f, 1.0f);
    msg_landmark.position.x = landmark->position.x();
    msg_landmark.position.y = landmark->position.y();
    msg_landmark.position.z = landmark->position.z();
    msg_landmark.covariance = {
        landmark->covariance(0, 0), landmark->covariance(0, 1),
        landmark->covariance(0, 2), landmark->covariance(1, 0),
        landmark->covariance(1, 1), landmark->covariance(1, 2),
        landmark->covariance(2, 0), landmark->covariance(2, 1),
        landmark->covariance(2, 2)};
    landmarks_msg.landmarks.push_back(msg_landmark);
  }

  std::sort(landmarks_msg.landmarks.begin(), landmarks_msg.landmarks.end(),
            [](const ptz_exploration_core::msg::Landmark &a,
               const ptz_exploration_core::msg::Landmark &b) {
              return a.confidence > b.confidence;
            });

  this->landmarks_pub_->publish(landmarks_msg);
}

void Optimizer::handle_unpaired_observations(
    const std::vector<std::shared_ptr<Optimizer::PolarObservation>>
        &unpaired_observations,
    const gtsam::Pose3 &camera_pose, const utils::CameraModel &camera_model,
    const std::optional<
        std::reference_wrapper<const Eigen::Matrix<double, 6, 6>>> &pose_cov) {
  // Match observations against candidate landmarks using ICNN data association
  std::vector<std::tuple<int, std::shared_ptr<PolarObservation>>>
      paired_candidate_observations;
  std::vector<std::shared_ptr<PolarObservation>> still_unpaired_observations;
  utils::icnn(unpaired_observations, this->candidate_landmarks_, camera_pose,
              camera_model, pose_cov, this->data_association_alpha_,
              this->min_landmark_covariance_, paired_candidate_observations,
              still_unpaired_observations);

  // Mark candidate landmarks with enough supporters for promotion
  std::set<int> promote_candidate_indices;
  for (const auto &entry : paired_candidate_observations) {
    const int candidate_index = std::get<0>(entry);
    const auto &observation = std::get<1>(entry);

    if (candidate_index < 0 ||
        candidate_index >=
            static_cast<int>(this->candidate_landmarks_.size())) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid candidate landmark index %d in paired observations",
                   candidate_index);
      assert(false && "Candidate landmark index out of bounds");
    }

    this->candidate_landmark_supporters_map_[candidate_index].push_back(
        observation);
    int num_supporters = static_cast<int>(
        candidate_landmark_supporters_map_[candidate_index].size());

    this->candidate_landmarks_[candidate_index]->confidence =
        std::max(this->candidate_landmarks_[candidate_index]->confidence,
                 observation->confidence);

    if (num_supporters >= this->candidate_confirmation_threshold_) {
      promote_candidate_indices.insert(candidate_index);
    }
  }

  // Promote candidates to confirmed landmarks and add to graph
  for (auto it = promote_candidate_indices.rbegin();
       it != promote_candidate_indices.rend(); ++it) {
    const int candidate_index = *it;
    auto &candidate = this->candidate_landmarks_[candidate_index];

    // Add landmark to graph
    int new_landmark_index = this->add_landmark_to_graph(candidate);

    // Add all supporting observations to graph
    for (const auto &obs :
         candidate_landmark_supporters_map_[candidate_index]) {
      int pose_key = this->add_pose_to_graph(obs->observation_pose);
      this->add_observations_to_graph(
          {std::make_tuple(new_landmark_index, obs)}, pose_key);
    }

    // Clean up candidate landmark and its supporters
    this->candidate_landmarks_.erase(this->candidate_landmarks_.begin() +
                                     candidate_index);
    this->candidate_landmark_supporters_map_.erase(candidate_index);
  }

  // Triangulate remaining unpaired observations with pending measurements to
  // create new candidates
  this->update_candidate_landmarks(still_unpaired_observations);
}

void Optimizer::update_candidate_landmarks(
    const std::vector<std::shared_ptr<Optimizer::PolarObservation>>
        &unpaired_observations) {
  // For each unpaired observation, try to triangulate with pending measurements
  // Keep only the best match (smallest ray distance error)
  for (size_t obs_idx = 0; obs_idx < unpaired_observations.size(); ++obs_idx) {
    const auto &unpaired_obs = unpaired_observations[obs_idx];
    double min_error = std::numeric_limits<double>::infinity();
    std::shared_ptr<PolarObservation> best_pending_obs;
    size_t best_pending_idx = 0;
    bool found_match = false;
    gtsam::Point3 best_triangulated_pt;

    // Find best pairing with pending measurements
    for (size_t j = 0; j < this->pending_measurements_.size(); ++j) {
      const auto &pending_obs = this->pending_measurements_[j];

      // Triangulate camera-frame bearings from two viewpoints and check error
      auto [success, triangulated_pt, ray_distance] =
          utils::triangulate_observations(
              *unpaired_obs, *pending_obs,
              this->triangulation_min_pose_baseline_m_,
              this->triangulation_min_depth_m_,
              this->triangulation_max_ray_distance_m_,
              this->triangulation_min_parallax_threshold_);

      // Keep pairing with smallest error
      if (success && ray_distance < min_error) {
        min_error = ray_distance;
        best_pending_obs = pending_obs;
        best_pending_idx = j;
        best_triangulated_pt = triangulated_pt;
        found_match = true;
      }
    }

    // If best match found, create candidate landmark and remove consumed
    // observations
    if (found_match && best_pending_obs) {
      // Create new candidate landmark with covariance from bearing uncertainty
      // propagation
      auto new_candidate = std::make_shared<Landmark>();
      new_candidate->id = 0;
      new_candidate->position = best_triangulated_pt;
      new_candidate->class_name = unpaired_obs->class_name;

      new_candidate->covariance = utils::triangulate_observation_covariance(
          *unpaired_obs, *best_pending_obs);

      new_candidate->confidence =
          std::max(unpaired_obs->confidence, best_pending_obs->confidence);

      this->candidate_landmarks_.push_back(new_candidate);
      int new_candidate_idx = this->candidate_landmarks_.size() - 1;
      candidate_landmark_supporters_map_[new_candidate_idx].push_back(
          unpaired_obs);
      candidate_landmark_supporters_map_[new_candidate_idx].push_back(
          best_pending_obs);

      // Remove consumed pending measurement
      this->pending_measurements_.erase(this->pending_measurements_.begin() +
                                        best_pending_idx);
    } else {
      // No good match found, add observation to pending for future
      // triangulation
      this->pending_measurements_.push_back(unpaired_obs);
    }
  }
}

int Optimizer::add_pose_to_graph(const gtsam::Pose3 &pose) {
  const int pose_key = static_cast<int>(this->pose_index_++);
  const gtsam::Key x_key = X(pose_key);

  this->pending_factors_.add(
      gtsam::PriorFactor<gtsam::Pose3>(x_key, pose, this->pose_prior_noise_));

  if (this->pending_values_.exists(x_key)) {
    this->pending_values_.update(x_key, pose);
  } else {
    this->pending_values_.insert(x_key, pose);
  }

  return pose_key;
}

void Optimizer::add_observations_to_graph(
    const std::vector<std::tuple<int, std::shared_ptr<PolarObservation>>>
        &observations,
    int pose_key) {
  const gtsam::Key x_key = X(pose_key);

  for (const auto &entry : observations) {
    const int landmark_index = std::get<0>(entry);
    const auto &obs = std::get<1>(entry);

    if (!obs) {
      continue;
    }

    if (landmark_index < 0 ||
        landmark_index >= static_cast<int>(this->confirmed_landmarks_.size())) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid landmark index %d in paired observations",
                   landmark_index);
      assert(false && "Landmark index out of bounds");
    }

    // Use 2x2 covariance directly
    Eigen::Matrix2d cov2 = obs->observation_direction_covariance;

    auto bearing_noise = gtsam::noiseModel::Gaussian::Covariance(cov2);
    const gtsam::Unit3 &bearing = obs->observation_direction;

    if (obs->range >= 0.0 && obs->range_covariance > 0.0) {
      gtsam::Vector3 sigmas;
      sigmas << std::sqrt(cov2(0, 0)), std::sqrt(cov2(1, 1)),
          std::sqrt(obs->range_covariance);
      auto bearing_range_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

      this->pending_factors_.add(
          gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3,
                                    double>(x_key, L(landmark_index), bearing,
                                            obs->range, bearing_range_noise));
    } else {
      this->pending_factors_.add(
          gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>(
              x_key, L(landmark_index), bearing, bearing_noise));
    }
  }
}

int Optimizer::add_landmark_to_graph(
    const std::shared_ptr<Landmark> &landmark) {
  if (!landmark) {
    return -1;
  }

  const int landmark_key = static_cast<int>(this->landmark_index_++);
  const gtsam::Key l_key = L(landmark_key);

  landmark->id = static_cast<uint32_t>(landmark_key);

  this->confirmed_landmarks_.push_back(landmark);

  if (this->pending_values_.exists(l_key)) {
    this->pending_values_.update(l_key, landmark->position);
  } else {
    this->pending_values_.insert(l_key, landmark->position);
  }

  auto landmark_noise =
      gtsam::noiseModel::Gaussian::Covariance(landmark->covariance);
  this->pending_factors_.add(gtsam::PriorFactor<gtsam::Point3>(
      l_key, landmark->position, landmark_noise));

  return landmark_key;
}

void Optimizer::optimize_graph() {
  // Assert that we have data to optimize
  assert(!this->pending_factors_.empty() &&
         "optimize_graph called with empty pending_factors_");
  assert(!this->pending_values_.empty() &&
         "optimize_graph called with empty pending_values_");

  try {
    // Perform iSAM2 incremental optimization
    this->isam_->update(this->pending_factors_, this->pending_values_);
    this->isam_->update();
    this->isam_->update();

    // Get full optimized estimates
    gtsam::Values estimates = this->isam_->calculateEstimate();

    // Update confirmed landmarks with optimized positions and covariances
    for (size_t i = 0; i < this->confirmed_landmarks_.size(); ++i) {
      const gtsam::Key landmark_key = L(static_cast<int>(i));
      if (estimates.exists(landmark_key)) {
        this->confirmed_landmarks_[i]->position =
            estimates.at<gtsam::Point3>(landmark_key);

        // Retrieve and store marginal covariance
        try {
          gtsam::Matrix marginal_cov =
              this->isam_->marginalCovariance(landmark_key);
          Eigen::Matrix3d cov3;
          cov3 << marginal_cov(0, 0), marginal_cov(0, 1), marginal_cov(0, 2),
              marginal_cov(1, 0), marginal_cov(1, 1), marginal_cov(1, 2),
              marginal_cov(2, 0), marginal_cov(2, 1), marginal_cov(2, 2);

          this->confirmed_landmarks_[i]->covariance = cov3;
        } catch (const std::exception &ex) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to get covariance for landmark %zu: %s", i,
                      ex.what());
          this->confirmed_landmarks_[i]->covariance =
              this->min_landmark_covariance_;
        }
      }
    }

    // Clear pending data after successful optimization
    this->pending_factors_.resize(0);
    this->pending_values_.clear();

    if (this->export_factor_graph_enabled_) {
      this->export_factor_graph_snapshot(estimates);
    }

    // RCLCPP_INFO(this->get_logger(), "Optimization successful: %zu landmarks",
    // this->confirmed_landmarks_.size());
  } catch (const std::exception &ex) {
    // Keep pending factors/values to retry with next measurement batch
    RCLCPP_ERROR(this->get_logger(),
                 "Optimization failed: %s. Keeping pending data for retry.",
                 ex.what());
  }
}

void Optimizer::export_factor_graph_snapshot(const gtsam::Values &estimates) {
  try {
    std::filesystem::path export_path(this->factor_graph_export_path_);
    if (export_path.has_parent_path()) {
      std::filesystem::create_directories(export_path.parent_path());
    }

    std::vector<ExportPoseNode> pose_nodes;
    std::vector<ExportLandmarkNode> landmark_nodes;
    pose_nodes.reserve(estimates.size());
    landmark_nodes.reserve(estimates.size());

    for (const auto key : estimates.keys()) {
      const gtsam::Symbol symbol(key);
      if (symbol.chr() == 'x') {
        const gtsam::Pose3 pose = estimates.at<gtsam::Pose3>(key);
        pose_nodes.push_back(ExportPoseNode{
            static_cast<int>(symbol.index()), pose.translation().x(),
            pose.translation().y(), pose.rotation().yaw()});
      } else if (symbol.chr() == 'l') {
        const gtsam::Point3 point = estimates.at<gtsam::Point3>(key);
        landmark_nodes.push_back(ExportLandmarkNode{
            static_cast<int>(symbol.index()), point.x(), point.y()});
      }
    }

    std::sort(pose_nodes.begin(), pose_nodes.end(),
              [](const auto &lhs, const auto &rhs) { return lhs.id < rhs.id; });
    std::sort(landmark_nodes.begin(), landmark_nodes.end(),
              [](const auto &lhs, const auto &rhs) { return lhs.id < rhs.id; });

    std::vector<ExportEdge> edges;
    const auto &factors = this->isam_->getFactorsUnsafe();
    for (const auto &factor : factors) {
      if (!factor) {
        continue;
      }

      const auto keys = factor->keys();
      if (keys.size() != 2) {
        continue;
      }

      const gtsam::Symbol first_symbol(keys[0]);
      const gtsam::Symbol second_symbol(keys[1]);
      if (!((first_symbol.chr() == 'x' || first_symbol.chr() == 'l') &&
            (second_symbol.chr() == 'x' || second_symbol.chr() == 'l'))) {
        continue;
      }

      edges.push_back(
          ExportEdge{first_symbol.chr() == 'x' ? "pose" : "landmark",
                     static_cast<int>(first_symbol.index()),
                     second_symbol.chr() == 'x' ? "pose" : "landmark",
                     static_cast<int>(second_symbol.index()),
                     Optimizer::factor_type_name(*factor)});
    }

    std::sort(edges.begin(), edges.end(), [](const auto &lhs, const auto &rhs) {
      if (lhs.from_type != rhs.from_type) {
        return lhs.from_type < rhs.from_type;
      }
      if (lhs.from_id != rhs.from_id) {
        return lhs.from_id < rhs.from_id;
      }
      if (lhs.to_type != rhs.to_type) {
        return lhs.to_type < rhs.to_type;
      }
      return lhs.to_id < rhs.to_id;
    });

    std::ofstream out(export_path, std::ios::trunc);
    if (!out.is_open()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open factor graph export file: %s",
                   export_path.c_str());
      return;
    }

    out << std::setprecision(17);
    out << "{\n";
    out << "  \"schema\": \"ptz_exploration_core.factor_graph.v1\",\n";
    out << "  \"robot_name\": \""
        << Optimizer::escape_json_string(this->robot_name_) << "\",\n";
    out << "  \"world_frame\": \""
        << Optimizer::escape_json_string(this->world_frame_) << "\",\n";
    out << "  \"generated_at_ns\": " << this->now().nanoseconds() << ",\n";

    out << "  \"poses\": [\n";
    for (size_t i = 0; i < pose_nodes.size(); ++i) {
      const auto &pose = pose_nodes[i];
      out << "    {\"type\": \"pose\", \"id\": " << pose.id
          << ", \"x\": " << pose.x << ", \"y\": " << pose.y
          << ", \"theta\": " << pose.theta << "}";
      out << (i + 1 < pose_nodes.size() ? ",\n" : "\n");
    }
    out << "  ],\n";

    out << "  \"landmarks\": [\n";
    for (size_t i = 0; i < landmark_nodes.size(); ++i) {
      const auto &landmark = landmark_nodes[i];
      out << "    {\"type\": \"landmark\", \"id\": " << landmark.id
          << ", \"x\": " << landmark.x << ", \"y\": " << landmark.y << "}";
      out << (i + 1 < landmark_nodes.size() ? ",\n" : "\n");
    }
    out << "  ],\n";

    out << "  \"edges\": [\n";
    for (size_t i = 0; i < edges.size(); ++i) {
      const auto &edge = edges[i];
      out << "    {\"from\": {\"type\": \"" << edge.from_type
          << "\", \"id\": " << edge.from_id << "}, \"to\": {\"type\": \""
          << edge.to_type << "\", \"id\": " << edge.to_id
          << "}, \"factor_type\": \"" << edge.factor_type << "\"}";
      out << (i + 1 < edges.size() ? ",\n" : "\n");
    }
    out << "  ]\n";
    out << "}\n";

    out.flush();

    if (!out.good()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed while writing factor graph export file: %s",
                   export_path.c_str());
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Wrote factor graph snapshot to %s",
                 export_path.c_str());
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to export factor graph snapshot: %s", ex.what());
  }
}

std::vector<std::shared_ptr<Optimizer::PolarObservation>>
Optimizer::polar_observations_from_measurement_msg(
    const ptz_exploration_core::msg::DetectionMeasurementArray::SharedPtr msg,
    const gtsam::Pose3 &camera_pose) {
  std::vector<std::shared_ptr<PolarObservation>> observations;

  for (const auto &measurement : msg->measurements) {
    const Eigen::Vector3d direction(measurement.bearing_unit.x,
                                    measurement.bearing_unit.y,
                                    measurement.bearing_unit.z);

    if (direction.norm() <= std::numeric_limits<double>::epsilon()) {
      RCLCPP_WARN(this->get_logger(),
                  "Dropping measurement with near-zero bearing vector");
      assert(false && "Invalid measurement: zero bearing vector");
    }

    auto obs = std::make_shared<PolarObservation>();
    obs->class_name = measurement.class_name;
    obs->confidence = measurement.confidence;
    obs->observation_pose = camera_pose;
    obs->observation_direction =
        gtsam::Unit3(direction.x(), direction.y(), direction.z());

    // Build 2x2 covariance in azimuth-elevation space from message
    Eigen::Matrix2d cov2d = Eigen::Matrix2d::Zero();
    cov2d(0, 0) = measurement.sigma_azimuth_rad * measurement.sigma_azimuth_rad;
    cov2d(1, 1) =
        measurement.sigma_elevation_rad * measurement.sigma_elevation_rad;
    obs->observation_direction_covariance = cov2d;

    if (measurement.measurement_type ==
        ptz_exploration_core::msg::DetectionMeasurement::BEARING_RANGE) {
      obs->range = measurement.range_m;
      obs->range_covariance =
          measurement.sigma_range_m * measurement.sigma_range_m;
    } else {
      obs->range = -1.0;
      obs->range_covariance = -1.0;
    }

    observations.push_back(obs);
  }

  return observations;
}

void Optimizer::publish_landmark_markers(const rclcpp::Time &stamp) {
  visualization_msgs::msg::MarkerArray marker_array;

  // Marker containing confirmed landmark centers as small red spheres (0.15m
  // diameter).
  visualization_msgs::msg::Marker points_marker =
      utils::make_landmark_points_marker(this->world_frame_, stamp, 0.15, 1.0f,
                                         0.0f, 0.0f, 1.0f);

  // Marker containing candidate landmark centers as small purple spheres (0.15m
  // diameter).
  visualization_msgs::msg::Marker candidate_points_marker =
      utils::make_landmark_points_marker(this->world_frame_, stamp, 0.15, 0.60f,
                                         0.0f, 0.80f, 1.0f);
  candidate_points_marker.ns = "candidate_landmark_points";
  candidate_points_marker.id = 1;

  // Clear previous covariance markers to avoid stale ellipsoids.
  marker_array.markers.push_back(
      utils::make_delete_all_marker(this->world_frame_, stamp));

  // Start covariance markers at id 1000 to avoid collision with point markers.
  int covariance_marker_id = 1000;
  int candidate_covariance_marker_id = 5000;

  for (size_t landmark_index = 0;
       landmark_index < this->confirmed_landmarks_.size(); ++landmark_index) {
    const auto &landmark = this->confirmed_landmarks_[landmark_index];
    const gtsam::Point3 &p = landmark->position;

    const double confidence =
        std::clamp(static_cast<double>(landmark->confidence), 0.0, 1.0);
    const float landmark_r = static_cast<float>(1.0 - confidence);
    const float landmark_g = static_cast<float>(confidence);
    const float landmark_b = 0.0f;

    // Add landmark point
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    points_marker.points.push_back(point);

    std_msgs::msg::ColorRGBA point_color;
    point_color.r = landmark_r;
    point_color.g = landmark_g;
    point_color.b = landmark_b;
    point_color.a = 1.0f;
    points_marker.colors.push_back(point_color);

    // Get covariance from landmark (already computed during optimization)
    const Eigen::Matrix3d &cov3 = landmark->covariance;

    // Full 3D covariance ellipsoid as a single oriented SPHERE marker.
    {

      Eigen::Vector3d half_axes;
      Eigen::Quaterniond q_ellipsoid;
      if (!utils::covariance_to_ellipsoid(
              cov3, this->viz_confidence_chi_squared_,
              this->viz_max_ellipse_axis_m_,
              this->viz_covariance_min_eigenvalue_, half_axes, q_ellipsoid)) {
        continue;
      }

      // RViz SPHERE marker is stretched and rotated into the covariance
      // ellipsoid.
      marker_array.markers.push_back(utils::make_covariance_ellipsoid_marker(
          this->world_frame_, stamp, covariance_marker_id++, p.x(), p.y(),
          p.z(), q_ellipsoid.x(), q_ellipsoid.y(), q_ellipsoid.z(),
          q_ellipsoid.w(), 2.0 * half_axes(0), 2.0 * half_axes(1),
          2.0 * half_axes(2), landmark_r, landmark_g, landmark_b, 0.20));
    }
  }

  marker_array.markers.push_back(points_marker);

  for (size_t landmark_index = 0;
       landmark_index < this->candidate_landmarks_.size(); ++landmark_index) {
    const auto &landmark = this->candidate_landmarks_[landmark_index];
    const gtsam::Point3 &p = landmark->position;

    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    candidate_points_marker.points.push_back(point);

    const Eigen::Matrix3d &cov3 = landmark->covariance;

    Eigen::Vector3d half_axes;
    Eigen::Quaterniond q_ellipsoid;
    if (!utils::covariance_to_ellipsoid(cov3, this->viz_confidence_chi_squared_,
                                        this->viz_max_ellipse_axis_m_,
                                        this->viz_covariance_min_eigenvalue_,
                                        half_axes, q_ellipsoid)) {
      continue;
    }

    auto candidate_cov_marker = utils::make_covariance_ellipsoid_marker(
        this->world_frame_, stamp, candidate_covariance_marker_id++, p.x(),
        p.y(), p.z(), q_ellipsoid.x(), q_ellipsoid.y(), q_ellipsoid.z(),
        q_ellipsoid.w(), 2.0 * half_axes(0), 2.0 * half_axes(1),
        2.0 * half_axes(2), 0.60f, 0.0f, 0.80f, 0.20);

    candidate_cov_marker.ns = "candidate_landmark_cov_3d";
    candidate_cov_marker.color.a = 0.20;
    marker_array.markers.push_back(candidate_cov_marker);
  }

  marker_array.markers.push_back(candidate_points_marker);

  this->landmark_marker_pub_->publish(marker_array);
}

} // namespace ptz_exploration_core

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ptz_exploration_core::Optimizer>());
  rclcpp::shutdown();
  return 0;
}
