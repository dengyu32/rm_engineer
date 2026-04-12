#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class PoseMarkerNode : public rclcpp::Node {
public:
  PoseMarkerNode() : Node("pose_marker_node") {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/pose_marker");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose_marker_pose");
    source_pose_topic_ = declare_parameter<std::string>("source_pose_topic", "");
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);
    marker_scale_m_ = declare_parameter<double>("marker_scale_m", 0.03);
    marker_rgba_ = declare_parameter<std::vector<double>>(
        "marker_rgba", std::vector<double>{1.0, 0.2, 0.2, 1.0});
    pose_values_ = declare_parameter<std::vector<double>>(
        "pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    pose_msg_ = buildPoseMessage();
    marker_msg_ = buildMarkerMessage();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, rclcpp::QoS(10));
    marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(marker_topic_, rclcpp::QoS(10));
    if (!source_pose_topic_.empty()) {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          source_pose_topic_, rclcpp::SensorDataQoS(),
          std::bind(&PoseMarkerNode::poseCallback, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(1, publish_period_ms_)),
        std::bind(&PoseMarkerNode::publishMessages, this));

    RCLCPP_INFO(get_logger(),
                "[pose_marker_node] frame=%s marker_topic=%s pose_topic=%s source_pose_topic=%s",
                frame_id_.c_str(), marker_topic_.c_str(), pose_topic_.c_str(),
                source_pose_topic_.empty() ? "<fixed_pose>" : source_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(),
                "[pose_marker_node] pose=[x=%.5f, y=%.5f, z=%.5f, qx=%.5f, qy=%.5f, qz=%.5f, qw=%.5f]",
                pose_msg_.pose.position.x, pose_msg_.pose.position.y, pose_msg_.pose.position.z,
                pose_msg_.pose.orientation.x, pose_msg_.pose.orientation.y,
                pose_msg_.pose.orientation.z, pose_msg_.pose.orientation.w);
  }

private:
  geometry_msgs::msg::PoseStamped buildPoseMessage() const {
    if (pose_values_.size() != 7) {
      throw std::runtime_error("parameter 'pose' must contain 7 elements: [x, y, z, qx, qy, qz, qw]");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = pose_values_[0];
    pose.pose.position.y = pose_values_[1];
    pose.pose.position.z = pose_values_[2];
    pose.pose.orientation.x = pose_values_[3];
    pose.pose.orientation.y = pose_values_[4];
    pose.pose.orientation.z = pose_values_[5];
    pose.pose.orientation.w = pose_values_[6];
    return pose;
  }

  visualization_msgs::msg::Marker buildMarkerMessage() const {
    if (marker_rgba_.size() != 4) {
      throw std::runtime_error("parameter 'marker_rgba' must contain 4 elements: [r, g, b, a]");
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "pose_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose_msg_.pose;
    marker.scale.x = marker_scale_m_;
    marker.scale.y = marker_scale_m_;
    marker.scale.z = marker_scale_m_;
    marker.color.r = static_cast<float>(marker_rgba_[0]);
    marker.color.g = static_cast<float>(marker_rgba_[1]);
    marker.color.b = static_cast<float>(marker_rgba_[2]);
    marker.color.a = static_cast<float>(marker_rgba_[3]);
    return marker;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::scoped_lock lock(mutex_);
    pose_msg_ = *msg;
    if (pose_msg_.header.frame_id.empty()) {
      pose_msg_.header.frame_id = frame_id_;
    }
    marker_msg_.header.frame_id = pose_msg_.header.frame_id;
    marker_msg_.pose = pose_msg_.pose;
  }

  void publishMessages() {
    geometry_msgs::msg::PoseStamped pose_msg;
    visualization_msgs::msg::Marker marker_msg;
    {
      std::scoped_lock lock(mutex_);
      const auto stamp = now();
      pose_msg_ .header.stamp = stamp;
      marker_msg_.header.stamp = stamp;
      marker_msg_.header.frame_id = pose_msg_.header.frame_id;
      marker_msg_.pose = pose_msg_.pose;
      pose_msg = pose_msg_;
      marker_msg = marker_msg_;
    }
    pose_pub_->publish(pose_msg);
    marker_pub_->publish(marker_msg);
  }

  std::mutex mutex_;
  std::string frame_id_;
  std::string marker_topic_;
  std::string pose_topic_;
  std::string source_pose_topic_;
  int publish_period_ms_{200};
  double marker_scale_m_{0.03};
  std::vector<double> marker_rgba_;
  std::vector<double> pose_values_;

  geometry_msgs::msg::PoseStamped pose_msg_;
  visualization_msgs::msg::Marker marker_msg_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
