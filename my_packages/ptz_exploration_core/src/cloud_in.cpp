#include "cloud_in.hpp"

#include "utils/ros.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstddef>

namespace ptz_exploration_core {

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

CloudIn::CloudIn() : rclcpp::Node("cloud_in") {
  this->declare_parameter("robot_name", "spot");
  this->declare_parameter("cameras", std::vector<std::string>{""});
  this->declare_parameter("points_pattern", "");
  this->declare_parameter("output_topic", "/cloud_in");
  this->declare_parameter("downsample_factor", 1);
  this->declare_parameter("max_range_m", -1.0);
  this->declare_parameter("max_z_m", -1.0);

  this->robot_name_ = this->get_parameter("robot_name").as_string();
  this->cameras_ = this->get_parameter("cameras").as_string_array();
  this->points_pattern_ = this->get_parameter("points_pattern").as_string();
  this->output_topic_ = this->get_parameter("output_topic").as_string();

  const int configured_downsample_factor =
      this->get_parameter("downsample_factor").as_int();
  this->downsample_factor_ = std::max(1, configured_downsample_factor);
  if (this->downsample_factor_ != configured_downsample_factor) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid downsample_factor parameter, using %d instead",
                this->downsample_factor_);
  }

  this->max_range_m_ = this->get_parameter("max_range_m").as_double();
  this->max_z_m_ = this->get_parameter("max_z_m").as_double();

  const auto qos = rclcpp::SensorDataQoS();

  this->cloud_pub_ =
      this->create_publisher<PointCloudMsg>(this->output_topic_, qos);

  for (const auto &camera : this->cameras_) {
    if (camera.empty()) {
      continue;
    }

    const std::string topic = utils::evaluate_pattern(
        this->points_pattern_, this->robot_name_, camera);

    this->cloud_subs_[camera] = this->create_subscription<PointCloudMsg>(
        topic, qos, [this, camera](const PointCloudMsg::ConstSharedPtr msg) {
          this->cloud_callback(msg, camera);
        });

    RCLCPP_INFO(this->get_logger(), "Initialized subscriber for %s",
                camera.c_str());
  }
}

void CloudIn::cloud_callback(const PointCloudMsg::ConstSharedPtr &msg,
                             const std::string &camera_name) {
  const std::size_t downsample_stride =
      static_cast<std::size_t>(this->downsample_factor_);
  const bool apply_range_filter = this->max_range_m_ > 0.0;
  const bool apply_z_filter = this->max_z_m_ > 0.0;

  if (this->downsample_factor_ == 1 && !apply_range_filter && !apply_z_filter) {
    auto republished_cloud = *msg;
    republished_cloud.header.stamp = this->now();
    this->cloud_pub_->publish(std::move(republished_cloud));
    RCLCPP_DEBUG(this->get_logger(), "Republished cloud from %s with factor %d",
                 camera_name.c_str(), this->downsample_factor_);
    return;
  }

  const std::size_t total_points = msg->width * msg->height;
  const std::size_t max_points =
      (total_points + downsample_stride - 1) / downsample_stride;
  const double max_range_sq = this->max_range_m_ * this->max_range_m_;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  PointCloudMsg republished_cloud;
  republished_cloud.header.frame_id = msg->header.frame_id;
  republished_cloud.header.stamp = this->now();
  republished_cloud.is_bigendian = msg->is_bigendian;
  republished_cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(republished_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(max_points);

  sensor_msgs::PointCloud2Iterator<float> out_x(republished_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(republished_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(republished_cloud, "z");

  std::size_t index = 0;
  std::size_t kept_count = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index) {
    if ((index % downsample_stride) != 0) {
      continue;
    }

    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;

    if (apply_range_filter) {
      const double range_sq = static_cast<double>(x) * static_cast<double>(x) +
                              static_cast<double>(y) * static_cast<double>(y) +
                              static_cast<double>(z) * static_cast<double>(z);
      if (range_sq > max_range_sq) {
        continue;
      }
    }

    if (apply_z_filter && z > this->max_z_m_) {
      continue;
    }

    *out_x = x;
    *out_y = y;
    *out_z = z;
    ++out_x;
    ++out_y;
    ++out_z;
    ++kept_count;
  }

  modifier.resize(kept_count);

  this->cloud_pub_->publish(republished_cloud);
  RCLCPP_DEBUG(this->get_logger(), "Republished cloud from %s with factor %d",
               camera_name.c_str(), this->downsample_factor_);
}

} // namespace ptz_exploration_core

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ptz_exploration_core::CloudIn>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}