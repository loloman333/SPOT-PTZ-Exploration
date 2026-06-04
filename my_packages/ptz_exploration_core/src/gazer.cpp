#include "gazer.hpp"

#include "utils/ros.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>
#include <vision_msgs/msg/detection2_d.hpp>

namespace ptz_exploration_core {
namespace {
double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
} // namespace

Gazer::Gazer() : rclcpp::Node("gazer") {
  this->service_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->sensor_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  this->declare_parameter("robot_name", "spot");
  this->declare_parameter("camera", "ptz_cam");
  this->declare_parameter("camera_info_pattern", "/{camera}/raw/camera_info");
  this->declare_parameter("ptz_cmd_topic", "/ptz_cmd");
  this->declare_parameter("ptz_state_topic", "/ptz_state");
  this->declare_parameter("ptz_settled_topic", "/ptz_settled");
  this->declare_parameter("target_point_topic", "/clicked_point");
  this->declare_parameter("octomap_topic", "/octomap_binary");
  this->declare_parameter("landmarks_topic", "/landmarks");
  this->declare_parameter("detections_topic", "/detections");
  this->declare_parameter("ptz_origin_frame", "ptz_cam_origin_link");
  this->declare_parameter("gaze_frame", "map");
  this->declare_parameter("world_frame", "map");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter<double>("octomap_min_translation_m", 0.1);
  this->declare_parameter<double>("octomap_min_rotation_rad", 0.09);
  this->declare_parameter<double>("octomap_update_interval", 1.0);
  this->declare_parameter<double>("landmark_dismiss_radius_m", 1.0);
  this->declare_parameter<int>("scan_num_stops", 12);
  this->declare_parameter<double>("scan_dwell_time_s", 1.0);
  this->declare_parameter<double>("scan_home_tolerance_rad", 0.05);
  this->declare_parameter<double>("scan_home_tolerance_zoom", 0.05);
  this->declare_parameter<double>("scan_zoom", 1.0);
  this->declare_parameter<double>("confirm_threshold", 1.0);
  this->declare_parameter<int>("turning_direction", 1);
  this->declare_parameter<double>("home_pan_offset_rad", 0.0);
  this->declare_parameter<double>("home_tilt_offset_rad", 0.0);
  this->declare_parameter<double>("home_zoom_offset", 0.0);

  this->robot_name_ = this->get_parameter("robot_name").as_string();
  this->camera_ = this->get_parameter("camera").as_string();
  this->camera_info_pattern_ =
      this->get_parameter("camera_info_pattern").as_string();
  this->ptz_cmd_topic_ = this->get_parameter("ptz_cmd_topic").as_string();
  this->ptz_state_topic_ = this->get_parameter("ptz_state_topic").as_string();
  this->ptz_settled_topic_ =
      this->get_parameter("ptz_settled_topic").as_string();
  this->target_point_topic_ =
      this->get_parameter("target_point_topic").as_string();
  this->octomap_topic_ = this->get_parameter("octomap_topic").as_string();
  this->landmarks_topic_ = this->get_parameter("landmarks_topic").as_string();
  this->detections_topic_ = this->get_parameter("detections_topic").as_string();
  this->ptz_origin_frame_ = this->get_parameter("ptz_origin_frame").as_string();
  this->gaze_frame_ = this->get_parameter("gaze_frame").as_string();
  this->world_frame_ = this->get_parameter("world_frame").as_string();
  this->base_frame_ = this->get_parameter("base_frame").as_string();
  this->octomap_min_translation_m_ =
      this->get_parameter("octomap_min_translation_m").as_double();
  this->octomap_min_rotation_rad_ =
      this->get_parameter("octomap_min_rotation_rad").as_double();
  this->octomap_update_interval_ = rclcpp::Duration::from_seconds(
      this->get_parameter("octomap_update_interval").as_double());
  this->landmark_dismiss_radius_m_ =
      this->get_parameter("landmark_dismiss_radius_m").as_double();
  this->scan_num_stops_ = this->get_parameter("scan_num_stops").as_int();
  this->scan_dwell_time_s_ =
      this->get_parameter("scan_dwell_time_s").as_double();
  this->scan_home_tolerance_rad_ =
      this->get_parameter("scan_home_tolerance_rad").as_double();
  this->scan_home_tolerance_zoom_ =
      this->get_parameter("scan_home_tolerance_zoom").as_double();
  this->scan_zoom_ = this->get_parameter("scan_zoom").as_double();
  this->confirm_threshold_ =
      this->get_parameter("confirm_threshold").as_double();
  this->turning_direction_ = this->get_parameter("turning_direction").as_int();
  this->home_pan_offset_rad_ =
      this->get_parameter("home_pan_offset_rad").as_double();
  this->home_tilt_offset_rad_ =
      this->get_parameter("home_tilt_offset_rad").as_double();
  this->home_zoom_offset_ = this->get_parameter("home_zoom_offset").as_double();

  if (this->turning_direction_ != -1 && this->turning_direction_ != 1) {
    RCLCPP_WARN(
        this->get_logger(),
        "Invalid 'turning_direction'=%d. Expected -1 or 1. Falling back to 1.",
        this->turning_direction_);
    this->turning_direction_ = 1;
  }

  this->tf_lookup_timeout_ms_ = 100;
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = this->sensor_cb_group_;

  this->clicked_point_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          this->target_point_topic_, 10,
          std::bind(&Gazer::target_callback, this, std::placeholders::_1),
          sub_opts);

  this->ptz_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      this->ptz_cmd_topic_, 10);

  this->clicked_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          this->target_point_topic_, 10);

  this->octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      this->octomap_topic_, 10,
      std::bind(&Gazer::octomap_callback, this, std::placeholders::_1),
      sub_opts);

  this->landmarks_sub_ =
      this->create_subscription<ptz_exploration_core::msg::LandmarkArray>(
          this->landmarks_topic_, 10,
          std::bind(&Gazer::landmarks_callback, this, std::placeholders::_1),
          sub_opts);

  this->detections_sub_ =
      this->create_subscription<vision_msgs::msg::Detection2DArray>(
          this->detections_topic_, 10,
          std::bind(&Gazer::detections_callback, this, std::placeholders::_1),
          sub_opts);

  this->ptz_state_sub_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
          this->ptz_state_topic_, 10,
          std::bind(&Gazer::ptz_state_callback, this, std::placeholders::_1),
          sub_opts);

  this->ptz_settled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      this->ptz_settled_topic_, 10,
      std::bind(&Gazer::ptz_settled_callback, this, std::placeholders::_1),
      sub_opts);

  utils::initialize_camera_info_subscribers(
      *this, std::vector<std::string>{this->camera_},
      this->camera_info_pattern_, this->robot_name_, this->cam_info_subs_,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg,
             const std::string &camera) {
        this->camera_info_callback(msg, camera);
      });

  this->zoom_adjust_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250), [this]() { /* timer disabled */ });

  this->scan_service_ = this->create_service<std_srvs::srv::Trigger>(
      "scan_environment",
      std::bind(&Gazer::handle_scan_service, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, this->service_cb_group_);

  this->confirm_service_ = this->create_service<std_srvs::srv::Trigger>(
      "confirm_landmarks",
      std::bind(&Gazer::handle_confirm_service, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, this->service_cb_group_);

  RCLCPP_INFO(this->get_logger(), "Gazer node started");
}

void Gazer::handle_scan_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;

  RCLCPP_INFO(this->get_logger(), "scan_environment started: scanning %d stops",
              this->scan_num_stops_);

  // 1) Go home first.
  (void)this->wait_until_home();

  if (!rclcpp::ok()) {
    response->success = false;
    response->message = "scan aborted: shutdown";
    return;
  }

  // 2) Calculate evenly distributed pan angles around the circle,
  // starting at home (pan=0) to avoid an initial 180-degree jump.
  const int num_stops = std::max(1, this->scan_num_stops_);
  const double dwell_time_s = std::max(0.1, this->scan_dwell_time_s_);

  std::vector<double> pan_angles;
  for (int i = 0; i < num_stops; ++i) {
    const double pan = 2.0 * M_PI * i / static_cast<double>(num_stops);
    pan_angles.push_back(pan);
  }

  // 3) Visit each stop: command, wait until settled, dwell.
  for (const double pan : pan_angles) {
    if (!rclcpp::ok()) {
      response->success = false;
      response->message = "scan aborted: shutdown";
      return;
    }

    // Command position (use configured scan zoom) with offset applied
    this->publish_ptz_command(pan + this->home_pan_offset_rad_,
                              0.0 + this->home_tilt_offset_rad_,
                              this->scan_zoom_ + this->home_zoom_offset_);

    // Wait until settled at this position
    const auto settle_start = this->now();
    bool settled = false;

    while (rclcpp::ok()) {
      bool is_settled = false;
      {
        std::scoped_lock<std::mutex> lock(this->mutex_);
        is_settled = this->latest_ptz_settled_;
      }

      geometry_msgs::msg::Point current_state;
      const bool has_state = this->get_latest_ptz_state_snapshot(current_state);
      const double target_pan = pan + this->home_pan_offset_rad_;
      const double target_tilt = 0.0 + this->home_tilt_offset_rad_;
      const double target_zoom = this->scan_zoom_ + this->home_zoom_offset_;

      const bool at_target =
          has_state &&
          std::abs(wrap_angle(current_state.x - target_pan)) <=
              this->scan_home_tolerance_rad_ &&
          std::abs(current_state.y - target_tilt) <=
              this->scan_home_tolerance_rad_ &&
          std::abs(current_state.z - target_zoom) <=
              this->scan_home_tolerance_zoom_;

      if (is_settled && at_target) {
        settled = true;
        break;
      }

      // Timeout: 10 seconds per stop
      if ((this->now() - settle_start).seconds() > 10.0) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (settled) {
      RCLCPP_DEBUG(this->get_logger(), "Settled at pan=%.3f rad", pan);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to settle at pan=%.3f rad within timeout", pan);
    }

    // Dwell at this position
    const auto dwell_start = this->now();
    while (rclcpp::ok() &&
           (this->now() - dwell_start).seconds() < dwell_time_s) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // 4) Return home and ensure settled.
  this->command_home_pose();
  (void)this->wait_until_home();

  RCLCPP_INFO(this->get_logger(),
              "scan_environment finished: PTZ back at home [0,0,1]");

  response->success = true;
  response->message = "scan complete";
}

void Gazer::handle_confirm_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;

  const bool ok = this->confirm_landmarks();
  response->success = ok;
  response->message = ok ? "landmark confirmed" : "no landmark confirmed";
}

void Gazer::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg,
    const std::string &camera_name) {
  std::scoped_lock<std::mutex> lock(this->mutex_);
  utils::cache_camera_model_from_info(msg, camera_name, this->camera_models_,
                                      this->get_logger());
}

void Gazer::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  auto octree = utils::deserialize_octomap_msg(
      *msg, *this->tf_buffer_, this->world_frame_, this->base_frame_,
      this->tf_lookup_timeout_ms_, this->octomap_min_translation_m_,
      this->octomap_min_rotation_rad_, this->octomap_update_interval_,
      this->last_octomap_tf_, this->has_last_octomap_pose_, this->now(),
      this->get_logger(), *this->get_clock());

  if (!octree) {
    return;
  }

  std::scoped_lock<std::mutex> lock(this->mutex_);
  this->latest_octomap_ = octree;
}

void Gazer::landmarks_callback(
    const ptz_exploration_core::msg::LandmarkArray::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(this->mutex_);
  this->latest_landmarks_ = msg;
}

void Gazer::detections_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
  if (this->camera_.empty()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No PTZ camera configured in 'camera'. Dropping detections.");
    return;
  }

  const std::string &ptz_camera = this->camera_;
  if (msg->header.frame_id.find(ptz_camera) == std::string::npos) {
    return;
  }

  std::scoped_lock<std::mutex> lock(this->mutex_);
  this->latest_detections_ = msg;
}

void Gazer::ptz_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < 3) {
    return;
  }

  geometry_msgs::msg::Point state;
  state.x = msg->position[0];
  state.y = msg->position[1];
  state.z = msg->position[2];

  std::scoped_lock<std::mutex> lock(this->mutex_);
  this->latest_ptz_state_ = state;
  this->has_latest_ptz_state_ = true;
}

void Gazer::ptz_settled_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(this->mutex_);
  this->latest_ptz_settled_ = msg->data;
}

void Gazer::publish_ptz_command(double pan, double tilt, double zoom) {
  geometry_msgs::msg::Point ptz_cmd;
  ptz_cmd.x = pan;
  ptz_cmd.y = tilt;
  ptz_cmd.z = zoom;
  this->ptz_cmd_pub_->publish(ptz_cmd);
}

void Gazer::command_home_pose() {
  // Publish home command with offset applied to physical frame
  this->publish_ptz_command(0.0 + this->home_pan_offset_rad_,
                            0.0 + this->home_tilt_offset_rad_,
                            1.0 + this->home_zoom_offset_);
}

bool Gazer::wait_until_home() {
  bool commanded_home = false;

  while (rclcpp::ok()) {
    geometry_msgs::msg::Point state;
    const bool has_state = this->get_latest_ptz_state_snapshot(state);

    if (has_state && this->ptz_state_is_home(state)) {
      return true;
    }

    bool is_settled = false;
    {
      std::scoped_lock<std::mutex> lock(this->mutex_);
      is_settled = this->latest_ptz_settled_;
    }

    if (commanded_home && is_settled) {
      return true;
    }

    if (!commanded_home) {
      this->command_home_pose();
      commanded_home = true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;
}

bool Gazer::ptz_state_is_home(const geometry_msgs::msg::Point &state) const {
  return std::abs(wrap_angle(state.x - this->home_pan_offset_rad_)) <=
             this->scan_home_tolerance_rad_ &&
         std::abs(state.y - this->home_tilt_offset_rad_) <=
             this->scan_home_tolerance_rad_ &&
         std::abs(state.z - (1.0 + this->home_zoom_offset_)) <=
             this->scan_home_tolerance_zoom_;
}

bool Gazer::get_latest_ptz_state_snapshot(geometry_msgs::msg::Point &state) {
  std::scoped_lock<std::mutex> lock(this->mutex_);
  state = this->latest_ptz_state_;
  return this->has_latest_ptz_state_;
}

void Gazer::publish_target_point(const geometry_msgs::msg::Point &point) {
  geometry_msgs::msg::PointStamped target_point;
  target_point.header.frame_id = this->world_frame_;
  target_point.header.stamp = this->now();
  target_point.point = point;
  this->clicked_point_pub_->publish(target_point);
}

bool Gazer::landmark_is_visible(
    const ptz_exploration_core::msg::Landmark &landmark,
    const octomap::OcTree &octomap,
    const geometry_msgs::msg::Point &robot_point) const {
  const double dx = landmark.position.x - robot_point.x;
  const double dy = landmark.position.y - robot_point.y;
  const double dz = landmark.position.z - robot_point.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (distance < 1e-6) {
    return true;
  }

  const double ray_limit = distance - this->landmark_dismiss_radius_m_;
  if (ray_limit <= 0.0) {
    return true;
  }

  const octomap::point3d start(robot_point.x, robot_point.y, robot_point.z);
  const octomap::point3d direction(dx / distance, dy / distance, dz / distance);
  octomap::point3d hit_point;
  const bool hit =
      octomap.castRay(start, direction, hit_point, true, ray_limit);
  return !hit;
}

bool Gazer::select_largest_detection(
    const vision_msgs::msg::Detection2DArray &detections,
    vision_msgs::msg::Detection2D &best_detection) const {
  bool found = false;
  double max_area = -1.0;

  for (const auto &detection : detections.detections) {
    const double bbox_area = std::max(0.0, detection.bbox.size_x) *
                             std::max(0.0, detection.bbox.size_y);

    if (!found || bbox_area > max_area) {
      max_area = bbox_area;
      best_detection = detection;
      found = true;
    }
  }

  return found;
}

double Gazer::compute_max_zoom_for_full_bbox_in_frame(
    const vision_msgs::msg::Detection2D &detection,
    const utils::CameraModel &cam_model, double reference_zoom) const {
  if (cam_model.image_width <= 1 || cam_model.image_height <= 1) {
    return std::clamp(reference_zoom, 1.0, 30.0);
  }

  // Model: image coordinates scale around optical center as zoom changes.
  // This handles off-center detections too.
  const double fit_margin = 0.7;
  const double image_w = static_cast<double>(cam_model.image_width);
  const double image_h = static_cast<double>(cam_model.image_height);

  const double border_margin_x = 0.5 * (1.0 - fit_margin) * image_w;
  const double border_margin_y = 0.5 * (1.0 - fit_margin) * image_h;

  const double cx = cam_model.cx;
  const double cy = cam_model.cy;

  const double bbox_center_x = detection.bbox.center.position.x;
  const double bbox_center_y = detection.bbox.center.position.y;
  const double half_w = 0.5 * detection.bbox.size_x;
  const double half_h = 0.5 * detection.bbox.size_y;

  const double left = bbox_center_x - half_w;
  const double right = bbox_center_x + half_w;
  const double top = bbox_center_y - half_h;
  const double bottom = bbox_center_y + half_h;

  // If detection touches (or nearly touches) image border,
  // treat bbox as potentially clipped and do not request further zoom-in.
  // This prevents over-zoom when detector reports truncated boxes.
  const double edge_eps_px = 10.0;
  const bool touches_image_border = left <= edge_eps_px || top <= edge_eps_px ||
                                    right >= (image_w - edge_eps_px) ||
                                    bottom >= (image_h - edge_eps_px);
  if (touches_image_border) {
    return std::clamp(reference_zoom, 1.0, 30.0);
  }

  const double min_x = border_margin_x;
  const double max_x = image_w - border_margin_x;
  const double min_y = border_margin_y;
  const double max_y = image_h - border_margin_y;

  double scale_limit = std::numeric_limits<double>::infinity();
  const double eps = 1e-6;

  if (left < cx - eps) {
    scale_limit =
        std::min(scale_limit, (cx - min_x) / std::max(eps, cx - left));
  }
  if (right > cx + eps) {
    scale_limit =
        std::min(scale_limit, (max_x - cx) / std::max(eps, right - cx));
  }
  if (top < cy - eps) {
    scale_limit = std::min(scale_limit, (cy - min_y) / std::max(eps, cy - top));
  }
  if (bottom > cy + eps) {
    scale_limit =
        std::min(scale_limit, (max_y - cy) / std::max(eps, bottom - cy));
  }

  if (!std::isfinite(scale_limit) || scale_limit <= 0.0) {
    scale_limit = 1.0;
  }

  const double max_zoom = reference_zoom * scale_limit;

  return std::clamp(max_zoom, 1.0, 30.0);
}

bool Gazer::confirm_landmarks() {
  // Go home first, like in scan_service.
  (void)this->wait_until_home();

  if (!rclcpp::ok()) {
    return false;
  }

  std::shared_ptr<octomap::OcTree> octomap;
  ptz_exploration_core::msg::LandmarkArray::SharedPtr landmarks_msg;
  geometry_msgs::msg::TransformStamped tf_world_from_ptz;

  {
    std::scoped_lock<std::mutex> lock(this->mutex_);
    octomap = this->latest_octomap_;
    landmarks_msg = this->latest_landmarks_;
  }

  if (!octomap) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "No octomap available yet for landmark confirmation");
    return false;
  }

  if (!landmarks_msg || landmarks_msg->landmarks.empty()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No landmarks available yet for landmark confirmation");
    return false;
  }

  try {
    tf_world_from_ptz = this->tf_buffer_->lookupTransform(
        this->world_frame_, this->ptz_origin_frame_, tf2::TimePointZero,
        std::chrono::milliseconds(this->tf_lookup_timeout_ms_));
  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Failed to get PTZ pose for landmark confirmation: %s",
                         e.what());
    return false;
  }

  geometry_msgs::msg::Point ptz_point;
  ptz_point.x = tf_world_from_ptz.transform.translation.x;
  ptz_point.y = tf_world_from_ptz.transform.translation.y;
  ptz_point.z = tf_world_from_ptz.transform.translation.z;

  const auto &landmarks = landmarks_msg->landmarks;
  if (landmarks.empty()) {
    return false;
  }

  const size_t start_index = [&]() {
    std::scoped_lock<std::mutex> lock(this->mutex_);
    if (this->next_landmark_index_ >= landmarks.size()) {
      this->next_landmark_index_ = 0;
    }
    return this->next_landmark_index_;
  }();

  const double confirm_time_s = 6.0;
  bool confirmed_any = false;

  const double settle_timeout_s = 6.0;

  const auto wait_until_settled_at_pose =
      [&](double pan, double tilt, double zoom,
          double timeout_s) -> std::pair<bool, rclcpp::Time> {
    auto start_time = this->now();
    auto settle_time = this->now();
    bool saw_unsettled = false;

    while (rclcpp::ok()) {
      bool is_settled = false;
      {
        std::scoped_lock<std::mutex> lock(this->mutex_);
        is_settled = this->latest_ptz_settled_;
      }

      if (!is_settled) {
        saw_unsettled = true;
      } else {
        geometry_msgs::msg::Point current_state;
        const bool has_state =
            this->get_latest_ptz_state_snapshot(current_state);
        const bool already_at_target =
            has_state &&
            std::abs(wrap_angle(current_state.x - pan)) <=
                this->scan_home_tolerance_rad_ &&
            std::abs(current_state.y - tilt) <=
                this->scan_home_tolerance_rad_ &&
            std::abs(current_state.z - zoom) <= this->scan_home_tolerance_zoom_;

        if (saw_unsettled || already_at_target) {
          return {true, this->now()};
        }
      }

      // Timeout guard
      if ((this->now() - start_time).seconds() > timeout_s) {
        RCLCPP_WARN(this->get_logger(),
                    "Timed out waiting to settle at pan=%.3f tilt=%.3f "
                    "zoom=%.3f after %.1fs",
                    pan, tilt, zoom, timeout_s);
        return {false, this->now()};
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return {false, settle_time};
  };

  for (size_t offset = 0; offset < landmarks.size(); ++offset) {
    const size_t index = (start_index + offset) % landmarks.size();
    const auto &landmark = landmarks[index];

    if (landmark.confidence >= this->confirm_threshold_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Skipping landmark '%s' (id=%u): confidence=%.2f >= "
                   "confirm_threshold=%.2f",
                   landmark.class_name.c_str(), landmark.id,
                   landmark.confidence, this->confirm_threshold_);
      continue;
    }

    if (!this->landmark_is_visible(landmark, *octomap, ptz_point)) {
      continue;
    }

    // Gaze at landmark in PTZ frame.
    geometry_msgs::msg::PointStamped landmark_point_world;
    landmark_point_world.header.frame_id = this->world_frame_;
    landmark_point_world.header.stamp = this->now();
    landmark_point_world.point = landmark.position;

    geometry_msgs::msg::PointStamped landmark_point_ptz;
    try {
      const auto tf = this->tf_buffer_->lookupTransform(
          this->ptz_origin_frame_, landmark_point_world.header.frame_id,
          tf2::TimePointZero,
          std::chrono::milliseconds(this->tf_lookup_timeout_ms_));
      tf2::doTransform(landmark_point_world, landmark_point_ptz, tf);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(this->get_logger(),
                  "confirm_landmarks: failed to transform landmark '%s' into "
                  "PTZ frame: %s",
                  landmark.class_name.c_str(), e.what());
      continue;
    }

    const auto [target_pan, target_tilt, target_zoom] =
        this->compute_ptz_cmd_from_point(landmark_point_ptz.point);
    this->publish_ptz_command(target_pan, target_tilt, target_zoom);
    this->publish_target_point(landmark.position);

    std::string class_name = landmark.class_name;
    const std::array<double, 3> zoom_candidates = {1.0, 2.0, 4.0};
    bool found_detection = false;
    vision_msgs::msg::Detection2D selected_detection;
    double selected_zoom = 1.0;

    for (const double attempt_zoom : zoom_candidates) {
      this->publish_ptz_command(target_pan, target_tilt, attempt_zoom);
      const auto [settled_ok, settle_time] = wait_until_settled_at_pose(
          target_pan, target_tilt, attempt_zoom, settle_timeout_s);

      if (!settled_ok) {
        // Timed out waiting to settle for this zoom candidate; try next
        // candidate.
        continue;
      }

      // After settle, wait at least 1s before checking detection.
      std::this_thread::sleep_for(std::chrono::seconds(1));

      vision_msgs::msg::Detection2DArray::SharedPtr detections;
      {
        std::scoped_lock<std::mutex> lock(this->mutex_);
        detections = this->latest_detections_;
      }

      if (!detections || detections->detections.empty() ||
          detections->header.frame_id.find(this->camera_) ==
              std::string::npos) {
        continue;
      }

      const rclcpp::Time detection_time(detections->header.stamp);

      if (detection_time <= settle_time) {
        continue;
      }

      vision_msgs::msg::Detection2D best_detection;
      if (!this->select_largest_detection(*detections, best_detection)) {
        continue;
      }

      found_detection = true;
      selected_detection = best_detection;
      selected_zoom = attempt_zoom;
      break;
    }

    if (!found_detection) {
      RCLCPP_INFO(this->get_logger(),
                  "No detection for '%s' after zoom ladder; skipping",
                  class_name.c_str());
      continue;
    }

    // Detection found: compute final zoom from bbox size, command it, and
    // settle before dwell.
    double final_zoom = selected_zoom;
    {
      utils::CameraModel cam_model;
      bool has_cam_model = false;
      {
        std::scoped_lock<std::mutex> lock(this->mutex_);
        auto it = this->camera_models_.find(this->camera_);
        if (it != this->camera_models_.end()) {
          cam_model = it->second;
          has_cam_model = true;
        }
      }

      if (has_cam_model && cam_model.image_width > 1 &&
          cam_model.image_height > 1) {
        final_zoom = this->compute_max_zoom_for_full_bbox_in_frame(
            selected_detection, cam_model, selected_zoom);
      }
    }

    if (std::abs(final_zoom - selected_zoom) >
        this->scan_home_tolerance_zoom_) {
      this->publish_ptz_command(target_pan, target_tilt, final_zoom);
      const auto [final_settled_ok, final_settle_time] =
          wait_until_settled_at_pose(target_pan, target_tilt, final_zoom,
                                     settle_timeout_s);
      if (!final_settled_ok) {
        RCLCPP_WARN(this->get_logger(),
                    "Final zoom settle timed out for landmark '%s' (id=%u)",
                    class_name.c_str(), landmark.id);
      }
    }

    const auto dwell_start = this->now();
    while (rclcpp::ok() &&
           (this->now() - dwell_start).seconds() < confirm_time_s) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    this->publish_ptz_command(target_pan, target_tilt, 1.0);
    (void)wait_until_settled_at_pose(target_pan, target_tilt, 1.0,
                                     settle_timeout_s);

    {
      std::scoped_lock<std::mutex> lock(this->mutex_);
      this->next_landmark_index_ = (index + 1) % landmarks.size();
      this->current_zoom_ = 1.0;
    }

    RCLCPP_INFO(this->get_logger(),
                "Confirmed landmark '%s' (id=%u, confidence=%.2f)",
                landmark.class_name.c_str(), landmark.id, landmark.confidence);
    confirmed_any = true;
  }

  // Always return PTZ to home [0,0,1] and wait until settled (bounded) before
  // leaving service.
  this->command_home_pose();
  (void)wait_until_settled_at_pose(0.0, 0.0, 1.0, settle_timeout_s);

  return confirmed_any;
}

void Gazer::target_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr target_point) {
  try {
    const auto tf = this->tf_buffer_->lookupTransform(
        this->ptz_origin_frame_, target_point->header.frame_id,
        tf2::TimePointZero);

    geometry_msgs::msg::PointStamped transformed_point;
    tf2::doTransform(*target_point, transformed_point, tf);

    const auto [target_pan, target_tilt, target_zoom_factor] =
        this->compute_ptz_cmd_from_point(transformed_point.point);

    this->publish_ptz_command(target_pan, target_tilt, target_zoom_factor);

    RCLCPP_DEBUG(this->get_logger(),
                 "Published PTZ CMD: Pan=%.2f, Tilt=%.2f, Zoom=%.2fx",
                 target_pan, target_tilt, target_zoom_factor);
  } catch (const tf2::TransformException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to gaze at point: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to gaze at point: %s", e.what());
  }
}

std::tuple<double, double, double> Gazer::compute_ptz_cmd_from_point(
    const geometry_msgs::msg::Point &point) const {
  const double x = point.x;
  const double y = point.y;
  const double z = point.z;

  const double target_pan =
      static_cast<double>(this->turning_direction_) * std::atan2(y, x);
  const double target_tilt = static_cast<double>(this->turning_direction_) *
                             -std::atan2(z, std::sqrt(x * x + y * y));

  return {target_pan, target_tilt, 1.0};
}

} // namespace ptz_exploration_core

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ptz_exploration_core::Gazer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
