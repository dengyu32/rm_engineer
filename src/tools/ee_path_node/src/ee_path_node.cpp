#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// 常用命令：
// ros2 run ee_path_node ee_path_node
// ros2 service call /clear_ee_path std_srvs/srv/Empty

class EePathNode : public rclcpp::Node {
public:
  EePathNode()
  : Node("ee_path_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "link6");
    path_topic_ = declare_parameter<std::string>("path_topic", "/ee_path");
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 50);
    max_points_ = declare_parameter<int>("max_points", 1000);
    position_threshold_m_ = declare_parameter<double>("position_threshold_m", 1e-4);

    path_.header.frame_id = base_frame_;
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10));
    clear_srv_ = create_service<std_srvs::srv::Empty>(
        "/clear_ee_path",
        std::bind(&EePathNode::clear_path, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(1, publish_period_ms_)),
        std::bind(&EePathNode::publish_path, this));

    RCLCPP_INFO(
        get_logger(),
        "Publishing %s from TF %s -> %s",
        path_topic_.c_str(),
        base_frame_.c_str(),
        ee_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Clear service available on /clear_ee_path");
  }

private:
  void clear_path(
      const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
    path_.poses.clear();
    path_.header.stamp = now();
    path_pub_->publish(path_);
    RCLCPP_INFO(get_logger(), "Cleared end-effector path");
  }

  void publish_path() {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "TF lookup failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = tf_msg.header.stamp;
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = tf_msg.transform.translation.x;
    pose.pose.position.y = tf_msg.transform.translation.y;
    pose.pose.position.z = tf_msg.transform.translation.z;
    pose.pose.orientation = tf_msg.transform.rotation;

    if (!should_append(pose)) {
      return;
    }

    path_.header.stamp = pose.header.stamp;
    path_.poses.push_back(pose);

    const std::size_t max_points = static_cast<std::size_t>(std::max(1, max_points_));
    if (path_.poses.size() > max_points) {
      path_.poses.erase(path_.poses.begin());
    }

    path_pub_->publish(path_);
  }

  bool should_append(const geometry_msgs::msg::PoseStamped &pose) const {
    if (path_.poses.empty()) {
      return true;
    }

    const auto &last = path_.poses.back().pose.position;
    const double dx = pose.pose.position.x - last.x;
    const double dy = pose.pose.position.y - last.y;
    const double dz = pose.pose.position.z - last.z;
    const double distance_squared = dx * dx + dy * dy + dz * dz;
    const double threshold_squared = position_threshold_m_ * position_threshold_m_;

    return distance_squared > threshold_squared;
  }

  std::string base_frame_;
  std::string ee_frame_;
  std::string path_topic_;
  int publish_period_ms_{50};
  int max_points_{1000};
  double position_threshold_m_{1e-4};

  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EePathNode>());
  rclcpp::shutdown();
  return 0;
}
