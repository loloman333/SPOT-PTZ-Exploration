#include "utils/ros.hpp"

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <octomap_msgs/conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>

namespace ptz_exploration_core::utils {

std::string extract_camera_from_frame(const std::string &frame_id,
                                      const std::vector<std::string> &cameras) {
  for (const auto &camera : cameras) {
    if (frame_id.find(camera) != std::string::npos) {
      return camera;
    }
  }
  return "";
}

std::string evaluate_pattern(const std::string &pattern,
                             const std::string &robot_name,
                             const std::string &camera_name) {
  std::string result = pattern;

  // Replace {robot} placeholder
  size_t robot_pos = result.find("{robot}");
  if (robot_pos != std::string::npos) {
    result.replace(robot_pos, 7, robot_name);
  }

  // Replace {camera} placeholder
  size_t camera_pos = result.find("{camera}");
  if (camera_pos != std::string::npos) {
    result.replace(camera_pos, 8, camera_name);
  }

  return result;
}

void initialize_camera_info_subscribers(
    rclcpp::Node &node, const std::vector<std::string> &cameras,
    const std::string &camera_info_pattern, const std::string &robot_name,
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> &cam_info_subs,
    std::function<void(const sensor_msgs::msg::CameraInfo::SharedPtr,
                       const std::string &)>
        callback) {
  for (const auto &camera : cameras) {
    if (camera.empty()) {
      continue;
    }

    std::string topic =
        evaluate_pattern(camera_info_pattern, robot_name, camera);

    auto subscription = node.create_subscription<sensor_msgs::msg::CameraInfo>(
        topic, 10,
        [callback, camera](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          callback(msg, camera);
        });

    cam_info_subs[camera] = subscription;

    RCLCPP_INFO(node.get_logger(),
                "Initialized camera_info subscriber for %s on topic %s",
                camera.c_str(), topic.c_str());
  }
}

void cache_camera_model_from_info(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg,
    const std::string &camera_name,
    std::map<std::string, CameraModel> &camera_models,
    const rclcpp::Logger &logger) {
  const bool is_first_time =
      camera_models.find(camera_name) == camera_models.end();

  camera_models[camera_name] = CameraModel{msg->k[0],
                                           msg->k[4],
                                           msg->k[2],
                                           msg->k[5],
                                           static_cast<int>(msg->width),
                                           static_cast<int>(msg->height)};

  if (is_first_time) {
    RCLCPP_INFO(
        logger,
        "CameraInfo received for %s (fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f)",
        camera_name.c_str(), msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
  } else {
    RCLCPP_DEBUG(
        logger,
        "CameraInfo updated for %s (fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f)",
        camera_name.c_str(), msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
  }
}

gtsam::Pose3 tf_to_pose(const geometry_msgs::msg::TransformStamped &tf) {
  const auto &t = tf.transform.translation;
  const auto &q = tf.transform.rotation;
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                      gtsam::Point3(t.x, t.y, t.z));
}

visualization_msgs::msg::Marker
make_delete_all_marker(const std::string &frame_id, const rclcpp::Time &stamp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

visualization_msgs::msg::Marker
make_landmark_points_marker(const std::string &frame_id,
                            const rclcpp::Time &stamp, double point_scale_m,
                            float r, float g, float b, float a) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "landmark_points";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = point_scale_m;
  marker.scale.y = point_scale_m;
  marker.scale.z = point_scale_m;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  return marker;
}

visualization_msgs::msg::Marker make_covariance_ellipsoid_marker(
    const std::string &frame_id, const rclcpp::Time &stamp, int marker_id,
    double px, double py, double pz, double qx, double qy, double qz, double qw,
    double scale_x, double scale_y, double scale_z, float r, float g, float b,
    double alpha) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "landmark_cov_3d";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = px;
  marker.pose.position.y = py;
  marker.pose.position.z = pz;
  marker.pose.orientation.x = qx;
  marker.pose.orientation.y = qy;
  marker.pose.orientation.z = qz;
  marker.pose.orientation.w = qw;

  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = alpha;
  return marker;
}

DetectionClassification
best_detection_classification(const vision_msgs::msg::Detection2D &detection) {
  DetectionClassification result;
  result.confidence = 0.0f;
  result.class_name = "";

  for (const auto &hypothesis : detection.results) {
    const float score = static_cast<float>(hypothesis.hypothesis.score);
    if (score > result.confidence) {
      result.confidence = score;
      result.class_name = hypothesis.hypothesis.class_id;
    }
  }

  return result;
}

std::shared_ptr<octomap::OcTree> deserialize_octomap_msg(
    const octomap_msgs::msg::Octomap &msg, tf2_ros::Buffer &tf_buffer,
    const std::string &world_frame, const std::string &base_frame,
    int tf_lookup_timeout_ms, double octomap_min_translation_m,
    double octomap_min_rotation_rad,
    const rclcpp::Duration &octomap_update_interval,
    geometry_msgs::msg::TransformStamped &last_octomap_tf,
    bool &has_last_octomap_pose, const rclcpp::Time &current_time,
    const rclcpp::Logger &logger, rclcpp::Clock &clock) {
  // --- GATE 1: Time Throttle ---
  if ((current_time - last_octomap_tf.header.stamp) < octomap_update_interval) {
    return nullptr;
  }

  // --- GATE 2: TF (Pose Change) Gate ---
  geometry_msgs::msg::TransformStamped current_tf_msg;
  try {
    current_tf_msg = tf_buffer.lookupTransform(
        world_frame, base_frame, tf2::TimePointZero,
        std::chrono::milliseconds(tf_lookup_timeout_ms));

    tf2::Transform current_pose;
    tf2::fromMsg(current_tf_msg.transform, current_pose);

    if (has_last_octomap_pose) {
      tf2::Transform last_octomap_pose;
      tf2::fromMsg(last_octomap_tf.transform, last_octomap_pose);

      const double translation_diff =
          (current_pose.getOrigin() - last_octomap_pose.getOrigin()).length();
      const double rotation_diff = current_pose.getRotation().angleShortestPath(
          last_octomap_pose.getRotation());

      if (translation_diff < octomap_min_translation_m &&
          rotation_diff < octomap_min_rotation_rad) {
        // Reset gate timer stamp while skipping expensive deserialization.
        last_octomap_tf.header.stamp = current_time;
        return nullptr;
      }
    }
  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN_THROTTLE(logger, clock, 2000,
                         "OctoMap gate TF lookup failed: %s", e.what());
    return nullptr;
  }

  try {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
        octomap_msgs::msgToMap(msg));

    if (!abstract_tree) {
      RCLCPP_ERROR(logger, "Failed to deserialize octomap message");
      return nullptr;
    }

    octomap::OcTree *octree =
        dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (!octree) {
      RCLCPP_ERROR(logger, "Octomap is not of type OcTree");
      return nullptr;
    }

    abstract_tree.release();
    last_octomap_tf = current_tf_msg;
    has_last_octomap_pose = true;
    return std::shared_ptr<octomap::OcTree>(octree);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Failed to convert octomap: %s", e.what());
    return nullptr;
  }
}

visualization_msgs::msg::Marker make_bearing_arrow_marker(
    const std::string &frame_id, const rclcpp::Time &stamp,
    const std::string &ns, int marker_id, const geometry_msgs::msg::Point &p0,
    const geometry_msgs::msg::Point &p1, double shaft_diameter,
    double head_diameter, double head_length, float r, float g, float b,
    float a, const rclcpp::Duration &lifetime) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.points.push_back(p0);
  marker.points.push_back(p1);
  marker.scale.x = shaft_diameter;
  marker.scale.y = head_diameter;
  marker.scale.z = head_length;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.lifetime = lifetime;
  return marker;
}

std::vector<visualization_msgs::msg::Marker> create_bearing_debug_markers(
    const std::string &frame_id, const rclcpp::Time &stamp,
    const std::string &ns, int marker_id_base,
    const Eigen::Vector3d &cam_origin, const Eigen::Vector3d &ray_map,
    const octomap::point3d &hit_point, bool hit, double sigma_azimuth_rad,
    double sigma_elevation_rad, double shaft_diameter, double head_diameter,
    double head_length, const rclcpp::Duration &lifetime,
    double confidence_chi_squared_2d, float covariance_alpha) {
  std::vector<visualization_msgs::msg::Marker> markers;
  markers.reserve(5);

  // Compute marker endpoints from camera origin and ray
  const double marker_length =
      hit ? (hit_point.x() - cam_origin(0)) * ray_map(0) +
                (hit_point.y() - cam_origin(1)) * ray_map(1) +
                (hit_point.z() - cam_origin(2)) * ray_map(2)
          : 5.0;

  geometry_msgs::msg::Point p0;
  p0.x = cam_origin(0);
  p0.y = cam_origin(1);
  p0.z = cam_origin(2);

  geometry_msgs::msg::Point p1;
  p1.x = cam_origin(0) + ray_map(0) * marker_length;
  p1.y = cam_origin(1) + ray_map(1) * marker_length;
  p1.z = cam_origin(2) + ray_map(2) * marker_length;

  // Color: Hit (bright red) or Miss (orange)
  const float r = hit ? 0.95f : 0.95f;
  const float g = hit ? 0.1f : 0.75f;
  const float b = hit ? 0.2f : 0.2f;
  const float a = 0.95f;

  // Main bearing arrow
  markers.push_back(make_bearing_arrow_marker(
      frame_id, stamp, ns, marker_id_base, p0, p1, shaft_diameter,
      head_diameter, head_length, r, g, b, a, lifetime));

  const Eigen::Vector3d origin(p0.x, p0.y, p0.z);
  const Eigen::Vector3d endpoint(p1.x, p1.y, p1.z);
  const Eigen::Vector3d ray_vec = endpoint - origin;
  const double ray_len = ray_vec.norm();

  if (ray_len <= 1e-9) {
    return markers;
  }

  const Eigen::Vector3d dir = ray_vec / ray_len;

  // Orthonormal tangent basis around nominal direction
  const Eigen::Vector3d basis_seed = (std::abs(dir.z()) < 0.9)
                                         ? Eigen::Vector3d::UnitZ()
                                         : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d tangent_u =
      (basis_seed - basis_seed.dot(dir) * dir).normalized();
  const Eigen::Vector3d tangent_v = dir.cross(tangent_u).normalized();

  const double k = std::sqrt(std::max(0.0, confidence_chi_squared_2d));
  const double d_az = k * sigma_azimuth_rad;
  const double d_el = k * sigma_elevation_rad;

  const std::array<Eigen::Vector2d, 4> edge_offsets = {
      Eigen::Vector2d(+d_az, 0.0), Eigen::Vector2d(-d_az, 0.0),
      Eigen::Vector2d(0.0, +d_el), Eigen::Vector2d(0.0, -d_el)};

  for (size_t k_edge = 0; k_edge < edge_offsets.size(); ++k_edge) {
    const Eigen::Vector2d &off = edge_offsets[k_edge];
    const Eigen::Vector3d perturbed_dir =
        (dir + off.x() * tangent_u + off.y() * tangent_v).normalized();

    geometry_msgs::msg::Point p_edge;
    p_edge.x = origin.x() + ray_len * perturbed_dir.x();
    p_edge.y = origin.y() + ray_len * perturbed_dir.y();
    p_edge.z = origin.z() + ray_len * perturbed_dir.z();

    markers.push_back(make_bearing_arrow_marker(
        frame_id, stamp, ns, marker_id_base + 1 + static_cast<int>(k_edge), p0,
        p_edge, shaft_diameter, head_diameter, head_length, r, g, b,
        covariance_alpha, lifetime));
  }

  return markers;
}

} // namespace ptz_exploration_core::utils
