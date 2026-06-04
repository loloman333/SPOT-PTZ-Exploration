#ifndef PTZ_EXPLORATION_CORE__CLOUD_IN_HPP_
#define PTZ_EXPLORATION_CORE__CLOUD_IN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <map>
#include <string>
#include <vector>

namespace ptz_exploration_core {

class CloudIn : public rclcpp::Node {
public:
  CloudIn();
  ~CloudIn() override = default;

private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using PointCloudSub = rclcpp::Subscription<PointCloudMsg>;

  void cloud_callback(const PointCloudMsg::ConstSharedPtr &msg,
                      const std::string &camera_name);

  std::string robot_name_;
  std::vector<std::string> cameras_;
  std::string points_pattern_;
  std::string output_topic_;
  int downsample_factor_{1};
  double max_range_m_{-1.0};
  double max_z_m_{-1.0};

  rclcpp::Publisher<PointCloudMsg>::SharedPtr cloud_pub_;
  std::map<std::string, PointCloudSub::SharedPtr> cloud_subs_;
};

} // namespace ptz_exploration_core

#endif // PTZ_EXPLORATION_CORE__CLOUD_IN_HPP_