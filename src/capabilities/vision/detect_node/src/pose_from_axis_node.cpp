// PoseFromAxisNode：融合中心与轴向，通过 TF 解析 yaw 并发布完整位姿/服务。

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <engineer_interfaces/msg/pose.hpp>
#include <engineer_interfaces/srv/detect_energy_unit.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <cinttypes>

namespace arm_controller
{

class PoseFromAxisNode : public rclcpp::Node
{
public:
    explicit PoseFromAxisNode(const rclcpp::NodeOptions &options)
        : Node("pose_from_axis_node", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {

        center_topic_ = declare_parameter<std::string>("center_topic", "/detect/center");
        axis_topic_ = declare_parameter<std::string>("axis_topic", "/detect/v_max");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/detect/grasp_pose");
        service_name_ = declare_parameter<std::string>("service_name", "/vision/detect_energy_unit");
        target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
        yaw_reference_frame_ = declare_parameter<std::string>("yaw_reference_frame", "link6");
        optical_frame_ = declare_parameter<std::string>("optical_frame", "camera_color_optical_frame");
        camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link");
        apply_optical_correction_ = declare_parameter<bool>("apply_optical_correction", true);
        yaw_ref_axis_ = declare_parameter<std::vector<double>>("yaw_ref_axis", {0.0, 0.0, 1.0});
        world_up_ = declare_parameter<std::vector<double>>("world_up", {0.0, 0.0, 1.0});
        tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
        service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 1.0);

        mf_center_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PointStamped>>(
            this, center_topic_, rmw_qos_profile_sensor_data);
        mf_axis_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped>>(
            this, axis_topic_, rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *mf_center_sub_, *mf_axis_sub_);
        sync_->registerCallback(std::bind(
            &PoseFromAxisNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            pose_topic_, rclcpp::SensorDataQoS());
        debug_center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "/debug/detect_center_in_target", rclcpp::SensorDataQoS());
        debug_axis_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/debug/detect_axis_in_target", rclcpp::SensorDataQoS());

        service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        detect_srv_ = create_service<engineer_interfaces::srv::DetectEnergyUnit>(
            service_name_,
            std::bind(&PoseFromAxisNode::handleGetPose, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_cb_group_);

        RCLCPP_INFO(get_logger(),
                    "[detect_node][axis] [PoseFromAxisNode] center_topic=%s axis_topic=%s pose_topic=%s target_frame=%s yaw_ref_frame=%s",
                    center_topic_.c_str(), axis_topic_.c_str(), pose_topic_.c_str(),
                    target_frame_.c_str(), yaw_reference_frame_.c_str());
        RCLCPP_INFO(get_logger(),
                    "[detect_node][axis] [PoseFromAxisNode] service=%s timeout=%.3fs",
                    service_name_.c_str(), service_timeout_sec_);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::PointStamped, geometry_msgs::msg::Vector3Stamped>;

    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PointStamped>> mf_center_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped>> mf_axis_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_center_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr debug_axis_pub_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Service<engineer_interfaces::srv::DetectEnergyUnit>::SharedPtr detect_srv_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string center_topic_;
    std::string axis_topic_;
    std::string pose_topic_;
    std::string service_name_;
    std::string target_frame_;
    std::string yaw_reference_frame_;
    std::string optical_frame_;
    std::string camera_frame_;
    bool apply_optical_correction_{true};
    std::vector<double> yaw_ref_axis_;
    std::vector<double> world_up_;
    double tf_timeout_sec_;
    double service_timeout_sec_;

    std::mutex pose_mutex_;
    std::condition_variable pose_cv_;
    bool waiting_pose_{false};
    bool pose_ready_{false};
    engineer_interfaces::msg::Pose pending_pose_;
    geometry_msgs::msg::Vector3 pending_vector_;
    uint64_t request_seq_{0};
    uint64_t active_request_id_{0};
    uint64_t sync_seq_{0};
    uint64_t completed_request_count_{0};
    rclcpp::Time last_sync_wall_time_{0, 0, RCL_ROS_TIME};
    std::string last_sync_center_frame_;
    std::string last_sync_axis_frame_;
    builtin_interfaces::msg::Time last_sync_center_stamp_;
    builtin_interfaces::msg::Time last_sync_axis_stamp_;
    std::string last_drop_reason_{"waiting for synchronized /detect/center and /detect/v_max"};

    static Eigen::Vector3d vecFromParam(const std::vector<double> &v, const Eigen::Vector3d &fallback)
    {
        if (v.size() != 3) {
            return fallback;
        }
        Eigen::Vector3d out(v[0], v[1], v[2]);
        if (out.norm() < 1e-9) {
            return fallback;
        }
        return out;
    }

    static Eigen::Matrix3d opticalToCamera()
    {
        // ROS optical frame (x right, y down, z forward) -> camera/body frame
        // (x forward, y left, z up).
        Eigen::Matrix3d R;
        R << 0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0;
        return R;
    }

    bool transformPointWithOptionalCorrection(
        const geometry_msgs::msg::PointStamped::ConstSharedPtr &in,
        geometry_msgs::msg::PointStamped &out,
        std::string &error)
    {
        geometry_msgs::msg::PointStamped msg = *in;

        try {
            out = tf_buffer_.transform(
                msg, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
            error.clear();
            return true;
        } catch (const tf2::TransformException &ex) {
            if (apply_optical_correction_ && msg.header.frame_id == optical_frame_) {
                geometry_msgs::msg::PointStamped corrected = msg;
                const Eigen::Vector3d p_opt(msg.point.x, msg.point.y, msg.point.z);
                const Eigen::Vector3d p_cam = opticalToCamera() * p_opt;
                corrected.point.x = p_cam.x();
                corrected.point.y = p_cam.y();
                corrected.point.z = p_cam.z();
                corrected.header.frame_id = camera_frame_;

                try {
                    out = tf_buffer_.transform(
                        corrected, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
                    error.clear();
                    return true;
                } catch (const tf2::TransformException &ex2) {
                    if (camera_frame_ != "camera_link") {
                        corrected.header.frame_id = "camera_link";
                        try {
                            out = tf_buffer_.transform(
                                corrected, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
                            error.clear();
                            return true;
                        } catch (const tf2::TransformException &ex3) {
                            error = std::string(ex.what()) +
                                    " | optical fallback failed: " + ex2.what() +
                                    " | camera_link fallback failed: " + ex3.what();
                            RCLCPP_WARN_THROTTLE(
                                get_logger(), *get_clock(), 2000,
                                "[detect_node][axis] center tf failed: %s | optical fallback failed: %s | camera_link fallback failed: %s",
                                ex.what(), ex2.what(), ex3.what());
                            return false;
                        }
                    }

                    error = std::string(ex.what()) +
                            " | optical fallback failed: " + ex2.what();
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "[detect_node][axis] center tf failed: %s | optical fallback failed: %s",
                        ex.what(), ex2.what());
                    return false;
                }
            }

            if (apply_optical_correction_ && msg.header.frame_id == camera_frame_ &&
                camera_frame_ != "camera_link") {
                try {
                    msg.header.frame_id = "camera_link";
                    out = tf_buffer_.transform(
                        msg, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
                    error.clear();
                    return true;
                } catch (const tf2::TransformException &ex2) {
                    error = std::string(ex.what()) + " | fallback failed: " + ex2.what();
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "[detect_node][axis] center tf failed: %s | fallback failed: %s",
                                         ex.what(), ex2.what());
                    return false;
                }
            }
            error = ex.what();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[detect_node][axis] center tf failed: %s", ex.what());
            return false;
        }
    }

    bool transformVectorWithOptionalCorrection(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &in,
        geometry_msgs::msg::Vector3Stamped &out,
        std::string &error)
    {
        geometry_msgs::msg::Vector3Stamped msg = *in;

        try {
            out = tf_buffer_.transform(
                msg, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
            error.clear();
            return true;
        } catch (const tf2::TransformException &ex) {
            if (apply_optical_correction_ && msg.header.frame_id == optical_frame_) {
                geometry_msgs::msg::Vector3Stamped corrected = msg;
                const Eigen::Vector3d v_opt(msg.vector.x, msg.vector.y, msg.vector.z);
                const Eigen::Vector3d v_cam = opticalToCamera() * v_opt;
                corrected.vector.x = v_cam.x();
                corrected.vector.y = v_cam.y();
                corrected.vector.z = v_cam.z();
                corrected.header.frame_id = camera_frame_;

                try {
                    out = tf_buffer_.transform(
                        corrected, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
                    error.clear();
                    return true;
                } catch (const tf2::TransformException &ex2) {
                    if (camera_frame_ != "camera_link") {
                        corrected.header.frame_id = "camera_link";
                        try {
                            out = tf_buffer_.transform(
                                corrected, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
                            error.clear();
                            return true;
                        } catch (const tf2::TransformException &ex3) {
                            error = std::string(ex.what()) +
                                    " | optical fallback failed: " + ex2.what() +
                                    " | camera_link fallback failed: " + ex3.what();
                            RCLCPP_WARN_THROTTLE(
                                get_logger(), *get_clock(), 2000,
                                "[detect_node][axis] axis tf failed: %s | optical fallback failed: %s | camera_link fallback failed: %s",
                                ex.what(), ex2.what(), ex3.what());
                            return false;
                        }
                    }

                    error = std::string(ex.what()) +
                            " | optical fallback failed: " + ex2.what();
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "[detect_node][axis] axis tf failed: %s | optical fallback failed: %s",
                        ex.what(), ex2.what());
                    return false;
                }
            }

            if (apply_optical_correction_ && msg.header.frame_id == camera_frame_ &&
                camera_frame_ != "camera_link") {
                try {
                    msg.header.frame_id = "camera_link";
                    out = tf_buffer_.transform(
                        msg, target_frame_, tf2::durationFromSec(tf_timeout_sec_));
                    error.clear();
                    return true;
                } catch (const tf2::TransformException &ex2) {
                    error = std::string(ex.what()) + " | fallback failed: " + ex2.what();
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "[detect_node][axis] axis tf failed: %s | fallback failed: %s",
                                         ex.what(), ex2.what());
                    return false;
                }
            }
            error = ex.what();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[detect_node][axis] axis tf failed: %s", ex.what());
            return false;
        }
    }

    // 同步中心与轴向，变换到目标坐标系，计算 yaw 并发布位姿。
    void syncCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr center_msg,
                      const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr axis_msg)
    {
        geometry_msgs::msg::PointStamped center_t;
        geometry_msgs::msg::Vector3Stamped axis_t;
        std::string center_error;
        std::string axis_error;
        bool waiting = false;

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            ++sync_seq_;
            last_sync_wall_time_ = now();
            last_sync_center_frame_ = center_msg->header.frame_id;
            last_sync_axis_frame_ = axis_msg->header.frame_id;
            last_sync_center_stamp_ = center_msg->header.stamp;
            last_sync_axis_stamp_ = axis_msg->header.stamp;
            waiting = waiting_pose_;
        }

        if (!transformPointWithOptionalCorrection(center_msg, center_t, center_error)) {
            if (waiting) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                last_drop_reason_ = "center transform failed: " + center_error;
            }
            return;
        }
        if (!transformVectorWithOptionalCorrection(axis_msg, axis_t, axis_error)) {
            if (waiting) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                last_drop_reason_ = "axis transform failed: " + axis_error;
            }
            return;
        }

        debug_center_pub_->publish(center_t);
        debug_axis_pub_->publish(axis_t);

        Eigen::Vector3d z_obj(axis_t.vector.x, axis_t.vector.y, axis_t.vector.z);
        if (z_obj.norm() < 1e-6) {
            if (waiting) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                last_drop_reason_ = "axis too small after transform";
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[detect_node][axis] axis too small");
            return;
        }
        z_obj.normalize();

        Eigen::Vector3d world_up = vecFromParam(world_up_, Eigen::Vector3d(0.0, 0.0, 1.0));
        world_up.normalize();

        Eigen::Vector3d ref_axis = world_up;
        if (!yaw_reference_frame_.empty()) {
            try {
                const auto stamp = rclcpp::Time(center_t.header.stamp);
                const auto tf_time = tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds()));
                const auto tf = tf_buffer_.lookupTransform(
                    target_frame_, yaw_reference_frame_, tf_time,
                    tf2::durationFromSec(tf_timeout_sec_));
                tf2::Quaternion q;
                tf2::fromMsg(tf.transform.rotation, q);
                tf2::Matrix3x3 m(q);
                const Eigen::Vector3d yaw_ref_local =
                    vecFromParam(yaw_ref_axis_, Eigen::Vector3d(0.0, 0.0, 1.0));
                tf2::Vector3 axis_local(yaw_ref_local.x(), yaw_ref_local.y(), yaw_ref_local.z());
                tf2::Vector3 axis_world = m * axis_local;
                ref_axis = Eigen::Vector3d(axis_world.x(), axis_world.y(), axis_world.z());
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "[detect_node][axis] yaw ref tf failed: %s", ex.what());
                ref_axis = world_up;
            }
        }

        if (ref_axis.norm() < 1e-6) {
            ref_axis = world_up;
        }
        ref_axis.normalize();

        Eigen::Vector3d x_obj = z_obj.cross(ref_axis);
        if (x_obj.norm() < 1e-6) {
            x_obj = z_obj.cross(Eigen::Vector3d(1.0, 0.0, 0.0));
        }
        if (x_obj.norm() < 1e-6) {
            x_obj = z_obj.cross(Eigen::Vector3d(0.0, 1.0, 0.0));
        }
        if (x_obj.norm() < 1e-6) {
            if (waiting) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                last_drop_reason_ = "x axis degenerate after yaw reference projection";
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[detect_node][axis] x axis degenerate");
            return;
        }
        x_obj.normalize();

        Eigen::Vector3d y_obj = z_obj.cross(x_obj);
        if (y_obj.norm() < 1e-6) {
            if (waiting) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                last_drop_reason_ = "y axis degenerate after orthogonalization";
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[detect_node][axis] y axis degenerate");
            return;
        }
        y_obj.normalize();

        Eigen::Matrix3d R;
        R.col(0) = x_obj;
        R.col(1) = y_obj;
        R.col(2) = z_obj;

        const Eigen::Quaterniond q(R);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = center_t.header.stamp;
        pose.header.frame_id = target_frame_;
        pose.pose.position.x = center_t.point.x;
        pose.pose.position.y = center_t.point.y;
        pose.pose.position.z = center_t.point.z;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose_pub_->publish(pose);

        engineer_interfaces::msg::Pose target_pose;
        target_pose.x = pose.pose.position.x;
        target_pose.y = pose.pose.position.y;
        target_pose.z = pose.pose.position.z;
        target_pose.qx = pose.pose.orientation.x;
        target_pose.qy = pose.pose.orientation.y;
        target_pose.qz = pose.pose.orientation.z;
        target_pose.qw = pose.pose.orientation.w;

        geometry_msgs::msg::Vector3 target_vector;
        target_vector.x = axis_t.vector.x;
        target_vector.y = axis_t.vector.y;
        target_vector.z = axis_t.vector.z;

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (waiting_pose_) {
                pending_pose_ = target_pose;
                pending_vector_ = target_vector;
                pose_ready_ = true;
                waiting_pose_ = false;
                ++completed_request_count_;
                last_drop_reason_ = "request satisfied";
            }
        }
        pose_cv_.notify_all();
    }

    void handleGetPose(
        const std::shared_ptr<engineer_interfaces::srv::DetectEnergyUnit::Request> request,
        std::shared_ptr<engineer_interfaces::srv::DetectEnergyUnit::Response> response)
    {
        if (!request->enable) {
            response->success = false;
            response->message = "disabled";
            return;
        }

        std::unique_lock<std::mutex> lock(pose_mutex_);
        if (waiting_pose_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "[detect_node][axis] request rejected: busy active_request=%" PRIu64,
                active_request_id_);
            response->success = false;
            response->message = "busy";
            return;
        }

        const uint64_t request_id = ++request_seq_;
        const uint64_t sync_count_before_wait = sync_seq_;
        waiting_pose_ = true;
        pose_ready_ = false;
        active_request_id_ = request_id;
        last_drop_reason_ = "waiting for synchronized /detect/center and /detect/v_max";

        RCLCPP_INFO(
            get_logger(),
            "[detect_node][axis] request=%" PRIu64 " waiting for pose timeout=%.3fs completed=%" PRIu64 " sync_count=%" PRIu64,
            request_id,
            service_timeout_sec_,
            completed_request_count_,
            sync_count_before_wait);

        const auto timeout = std::chrono::duration<double>(service_timeout_sec_);
        const bool ok = pose_cv_.wait_for(lock, timeout, [&]() { return pose_ready_; });
        if (!ok) {
            waiting_pose_ = false;
            active_request_id_ = 0;
            response->success = false;
            response->message = "timeout";
            const uint64_t sync_count_after_wait = sync_seq_;
            const bool saw_sync = last_sync_wall_time_.nanoseconds() > 0;
            const double last_sync_age =
                saw_sync ? (now() - last_sync_wall_time_).seconds() : -1.0;
            RCLCPP_ERROR(
                get_logger(),
                "[detect_node][axis] request=%" PRIu64 " timeout after %.3fs: sync_count_before=%" PRIu64
                " sync_count_after=%" PRIu64 " last_drop_reason=%s last_sync_age=%.3fs"
                " last_center=(frame=%s stamp=%d.%09u) last_axis=(frame=%s stamp=%d.%09u)",
                request_id,
                service_timeout_sec_,
                sync_count_before_wait,
                sync_count_after_wait,
                last_drop_reason_.c_str(),
                last_sync_age,
                last_sync_center_frame_.c_str(),
                last_sync_center_stamp_.sec,
                last_sync_center_stamp_.nanosec,
                last_sync_axis_frame_.c_str(),
                last_sync_axis_stamp_.sec,
                last_sync_axis_stamp_.nanosec);
            return;
        }

        response->target.target_pose = pending_pose_;
        response->target.target_vector = pending_vector_;
        response->success = true;
        response->message = "ok";
        pose_ready_ = false;
        active_request_id_ = 0;
        RCLCPP_INFO(
            get_logger(),
            "[detect_node][axis] request=%" PRIu64 " success pose=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] vector=[%.4f, %.4f, %.4f]",
            request_id,
            response->target.target_pose.x,
            response->target.target_pose.y,
            response->target.target_pose.z,
            response->target.target_pose.qx,
            response->target.target_pose.qy,
            response->target.target_pose.qz,
            response->target.target_pose.qw,
            response->target.target_vector.x,
            response->target.target_vector.y,
            response->target.target_vector.z);
    }
};

} // 命名空间 arm_controller

RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::PoseFromAxisNode)
