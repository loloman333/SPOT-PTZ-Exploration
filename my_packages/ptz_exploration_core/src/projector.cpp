#include "projector.hpp"
#include "utils/math.hpp"
#include "utils/ros.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

namespace ptz_exploration_core {

Projector::Projector() : rclcpp::Node("projector") {
  // Declare parameters
  this->declare_parameter("robot_name", "spot");
  this->declare_parameter("cameras", std::vector<std::string>{""});
  this->declare_parameter("camera_info_pattern", "");
  // Global/world frame used for octomap ray casting and TF lookups.
  this->declare_parameter("world_frame", "map");
  // Robot base frame used by octomap update gating (TF pose-delta check).
  this->declare_parameter("base_frame", "base_link");
  // Input/output topic names.
  this->declare_parameter("detections_topic", "/detections");
  this->declare_parameter("octomap_topic", "/octomap_full");
  this->declare_parameter("detection_measurements_topic",
                          "/detection_measurements");
  this->declare_parameter("detection_debug_markers_topic",
                          "/detection_debug_markers");

  // Minimum vector norm guard when deriving bearing from hit point.
  this->declare_parameter("max_ray_distance", 100.0);

  // Multiplier applied to bbox size for the center-confidence region used in
  // bearing uncertainty.
  this->declare_parameter("bbox_center_confidence_multiplier", 2.0);

  // Minimum bearing uncertainty floor (radians) combined with bbox-derived
  // uncertainty.
  this->declare_parameter("min_bearing_sigma_rad", 0.01);

  // Proportional range uncertainty model: sigma_range = multiplier * range.
  this->declare_parameter("range_sigma_multiplier", 0.1);

  // Minimum range uncertainty floor (meters) combined with range-derived
  // uncertainty.
  this->declare_parameter("min_range_sigma_m", 0.05);

  // Octomap update gating.
  this->declare_parameter<double>("octomap_min_translation_m", 0.1); // 10 cm
  this->declare_parameter<double>("octomap_min_rotation_rad",
                                  0.09);                           // ~5 degrees
  this->declare_parameter<double>("octomap_update_interval", 1.0); // seconds

  // Read parameters
  this->robot_name_ = this->get_parameter("robot_name").as_string();
  this->cameras_ = this->get_parameter("cameras").as_string_array();
  this->camera_info_pattern_ =
      this->get_parameter("camera_info_pattern").as_string();
  this->world_frame_ = this->get_parameter("world_frame").as_string();
  this->base_frame_ = this->get_parameter("base_frame").as_string();
  this->detections_topic_ = this->get_parameter("detections_topic").as_string();
  this->octomap_topic_ = this->get_parameter("octomap_topic").as_string();
  this->detection_measurements_topic_ =
      this->get_parameter("detection_measurements_topic").as_string();
  this->detection_debug_markers_topic_ =
      this->get_parameter("detection_debug_markers_topic").as_string();
  this->max_ray_distance_ = this->get_parameter("max_ray_distance").as_double();
  this->bbox_center_confidence_multiplier_ =
      this->get_parameter("bbox_center_confidence_multiplier").as_double();
  this->min_bearing_sigma_rad_ =
      this->get_parameter("min_bearing_sigma_rad").as_double();
  this->range_sigma_multiplier_ =
      this->get_parameter("range_sigma_multiplier").as_double();
  this->min_range_sigma_m_ =
      this->get_parameter("min_range_sigma_m").as_double();
  this->octomap_min_translation_m_ =
      this->get_parameter("octomap_min_translation_m").as_double();
  this->octomap_min_rotation_rad_ =
      this->get_parameter("octomap_min_rotation_rad").as_double();
  this->octomap_update_interval_ = rclcpp::Duration::from_seconds(
      this->get_parameter("octomap_update_interval").as_double());

  // Initialize TF buffer
  this->tf_lookup_timeout_ms_ = 100.0;
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  // Initialize publishers
  this->measurement_array_pub_ = this->create_publisher<
      ptz_exploration_core::msg::DetectionMeasurementArray>(
      this->detection_measurements_topic_, 10);
  this->debug_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          this->detection_debug_markers_topic_, 10);

  // Initialize subscribers
  this->detection_sub_ =
      this->create_subscription<vision_msgs::msg::Detection2DArray>(
          this->detections_topic_, 10,
          std::bind(&Projector::detection_callback, this,
                    std::placeholders::_1));

  this->octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      this->octomap_topic_, 10,
      std::bind(&Projector::octomap_callback, this, std::placeholders::_1));

  utils::initialize_camera_info_subscribers(
      *this, this->cameras_, this->camera_info_pattern_, this->robot_name_,
      this->cam_info_subs_,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg,
             const std::string &camera) {
        this->cam_info_callback(msg, camera);
      });

  RCLCPP_INFO(this->get_logger(), "Projector initialized with robot_name=%s",
              this->robot_name_.c_str());
}

void Projector::cam_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg,
    const std::string &camera_name) {
  utils::cache_camera_model_from_info(msg, camera_name, this->camera_models_,
                                      this->get_logger());
}

void Projector::octomap_callback(
    const octomap_msgs::msg::Octomap::SharedPtr msg) {
  auto octree = utils::deserialize_octomap_msg(
      *msg, *this->tf_buffer_, this->world_frame_, this->base_frame_,
      this->tf_lookup_timeout_ms_, this->octomap_min_translation_m_,
      this->octomap_min_rotation_rad_, this->octomap_update_interval_,
      this->last_octomap_tf_, this->has_last_octomap_pose_, this->now(),
      this->get_logger(), *this->get_clock());

  if (octree) {
    this->latest_octomap_ = octree;
  }
}

void Projector::detection_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
  if (!this->latest_octomap_) {
    RCLCPP_WARN(this->get_logger(), "No octomap available yet");
    return;
  }

  const auto &camera_frame = msg->header.frame_id;
  const auto &detection_stamp = msg->header.stamp;

  // Extract camera name from frame_id
  std::string camera =
      utils::extract_camera_from_frame(camera_frame, this->cameras_);
  if (camera.empty() ||
      this->camera_models_.find(camera) == this->camera_models_.end()) {
    RCLCPP_WARN(this->get_logger(),
                "Unknown camera or camera model not ready: %s",
                camera_frame.c_str());
    return;
  }

  // Get camera pose in map frame (for ray cast)
  geometry_msgs::msg::TransformStamped tf_map_from_camera;
  // Wait (bounded) until the exact-time transform becomes available.
  // This preserves exact stamping for projection while avoiding
  // repeated "extrapolation into the future" warnings when TF
  // lags the sensor by a few milliseconds.
  const int wait_step_ms = 5;
  int waited_ms = 0;
  bool tf_ready = false;

  while (waited_ms < this->tf_lookup_timeout_ms_) {
    try {
      if (this->tf_buffer_->canTransform(
              this->world_frame_, camera_frame, detection_stamp,
              std::chrono::milliseconds(wait_step_ms))) {
        tf_ready = true;
        break;
      }
    } catch (const tf2::TransformException &e) {
      // ignore and retry until timeout
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_step_ms));
    waited_ms += wait_step_ms;
  }

  if (!tf_ready) {
    // Final attempt to produce a meaningful error message and respect
    // the configured lookup timeout.
    try {
      tf_map_from_camera = this->tf_buffer_->lookupTransform(
          this->world_frame_, camera_frame, detection_stamp,
          std::chrono::milliseconds(this->tf_lookup_timeout_ms_));
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(this->get_logger(),
                  "TF lookup failed after waiting %d ms: %s", waited_ms,
                  e.what());
      return;
    }
  } else {
    // Perform the actual lookup; canTransform indicated the transform
    // exists at the requested stamp so a short lookup timeout is fine.
    try {
      tf_map_from_camera = this->tf_buffer_->lookupTransform(
          this->world_frame_, camera_frame, detection_stamp,
          std::chrono::milliseconds(1));
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed after canTransform: %s",
                  e.what());
      return;
    }
  }

  Eigen::Vector3d cam_origin(tf_map_from_camera.transform.translation.x,
                             tf_map_from_camera.transform.translation.y,
                             tf_map_from_camera.transform.translation.z);

  const auto &cam_model = this->camera_models_[camera];

  ptz_exploration_core::msg::DetectionMeasurementArray measurement_array_msg;
  measurement_array_msg.header.stamp = detection_stamp;
  measurement_array_msg.header.frame_id = camera_frame;

  visualization_msgs::msg::MarkerArray marker_array_msg;

  // Process each detection
  for (size_t i = 0; i < msg->detections.size(); ++i) {
    const auto &detection = msg->detections[i];
    double u = detection.bbox.center.position.x;
    double v = detection.bbox.center.position.y;

    // If detection bbox touches any image border, skip to avoid unreliable
    // bearing uncertainty estimates.

    // // X border detection
    // if (detection.bbox.center.position.x - detection.bbox.size_x / 2.0 <=
    // cam_model.image_width * 0.025 ||
    //     detection.bbox.center.position.x + detection.bbox.size_x / 2.0 >=
    //     cam_model.image_width * 0.975 || detection.bbox.center.position.y -
    //     detection.bbox.size_y / 2.0 <= cam_model.image_height * 0.025 ||
    //     detection.bbox.center.position.y + detection.bbox.size_y / 2.0 >=
    //     cam_model.image_height * 0.975)
    // {
    //     // RCLCPP_WARN(this->get_logger(), "Skipping detection %zu due to
    //     border proximity", i); continue;
    // }

    // Compute anisotropic bearing uncertainty from the bbox center-confidence
    // region.
    const double sigma_azimuth_bbox =
        ((detection.bbox.size_x * this->bbox_center_confidence_multiplier_) /
         (4.0 * cam_model.fx));
    const double sigma_elevation_bbox =
        ((detection.bbox.size_y * this->bbox_center_confidence_multiplier_) /
         (4.0 * cam_model.fy));

    // Combine with minimum floor uncertainty.
    const double sigma_azimuth =
        std::sqrt(sigma_azimuth_bbox * sigma_azimuth_bbox +
                  this->min_bearing_sigma_rad_ * this->min_bearing_sigma_rad_);
    const double sigma_elevation =
        std::sqrt(sigma_elevation_bbox * sigma_elevation_bbox +
                  this->min_bearing_sigma_rad_ * this->min_bearing_sigma_rad_);

    // Project pixel to 3D ray in camera frame
    Eigen::Vector3d ray = utils::pixel_to_ray(u, v, cam_model);

    // Transform ray direction to map frame (for raycast)
    Eigen::Vector3d ray_map =
        utils::rotate_vector(ray, tf_map_from_camera.transform.rotation);
    ray_map.normalize();

    // Cast ray through octomap
    octomap::point3d start(cam_origin(0), cam_origin(1), cam_origin(2));
    octomap::point3d direction(ray_map(0), ray_map(1), ray_map(2));

    octomap::point3d hit_point;
    bool hit = this->latest_octomap_->castRay(start, direction, hit_point, true,
                                              this->max_ray_distance_);

    // Fill measurement message
    const auto classification = utils::best_detection_classification(detection);
    ptz_exploration_core::msg::DetectionMeasurement measurement_msg;
    measurement_msg.header.stamp = detection_stamp;
    measurement_msg.header.frame_id = camera_frame;
    measurement_msg.class_name = classification.class_name;
    measurement_msg.confidence = classification.confidence;
    measurement_msg.sigma_azimuth_rad = static_cast<float>(sigma_azimuth);
    measurement_msg.sigma_elevation_rad = static_cast<float>(sigma_elevation);

    // Bearing-only in camera frame
    measurement_msg.measurement_type =
        ptz_exploration_core::msg::DetectionMeasurement::BEARING_ONLY;
    measurement_msg.range_m = std::numeric_limits<float>::quiet_NaN();
    measurement_msg.bearing_unit.x = ray(0);
    measurement_msg.bearing_unit.y = ray(1);
    measurement_msg.bearing_unit.z = ray(2);

    if (hit) {
      measurement_msg.measurement_type =
          ptz_exploration_core::msg::DetectionMeasurement::BEARING_RANGE;
      measurement_msg.range_m = static_cast<float>((hit_point - start).norm());
      const double sigma_range_model =
          this->range_sigma_multiplier_ *
          static_cast<double>(measurement_msg.range_m);
      measurement_msg.sigma_range_m = static_cast<float>(
          std::sqrt(sigma_range_model * sigma_range_model +
                    this->min_range_sigma_m_ * this->min_range_sigma_m_));
    }

    measurement_array_msg.measurements.push_back(measurement_msg);

    const auto debug_markers = utils::create_bearing_debug_markers(
        world_frame_, detection_stamp, "detection_bearings_" + camera,
        static_cast<int>(i) * 10, cam_origin, ray_map, hit_point, hit,
        static_cast<double>(measurement_msg.sigma_azimuth_rad),
        static_cast<double>(measurement_msg.sigma_elevation_rad),
        0.04, // shaft diameter
        0.08, // head diameter
        0.10, // head length
        rclcpp::Duration::from_seconds(1.0),
        5.991, // 95% confidence for 2 DoF
        0.35f  // more transparent covariance-edge markers
    );

    marker_array_msg.markers.insert(marker_array_msg.markers.end(),
                                    debug_markers.begin(), debug_markers.end());
  }

  if (!measurement_array_msg.measurements.empty()) {
    this->measurement_array_pub_->publish(measurement_array_msg);
    this->debug_marker_pub_->publish(marker_array_msg);
  }
}

} // namespace ptz_exploration_core

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ptz_exploration_core::Projector>());
  rclcpp::shutdown();
  return 0;
}
