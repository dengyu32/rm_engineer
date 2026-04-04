#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <algorithm>
#include <memory>

#include "yolos/tasks/segmentation.hpp"  // YOLOs-CPP 分割接口

namespace arm_controller
{

// 位姿质量等级
enum class PoseQuality
{
    FULL_6DOF,    // 完整 6DoF，yaw 可观测
    DEGRADED_5DOF // 降级 5DoF，yaw 不可观测（轴对称物体）
};

class DetectNode : public rclcpp::Node
{
public:
    explicit DetectNode(const rclcpp::NodeOptions &options);

private:
    // ================= Message Filters Sync (时间同步) =================
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> mf_color_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> mf_depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // ================= Subscribers =================
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // ================= Publisher =================
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // ================= YOLO Detector =================
    std::unique_ptr<yolos::seg::YOLOSegDetector> detector_;

    // ================= Camera parameters =================
    double fx_, fy_, cx_, cy_;
    double depth_scale_;

    // ================= Hold logic =================
    static constexpr double HOLD_SEC = 0.5;
    rclcpp::Time last_valid_time_;
    cv::Mat last_valid_vis_;

    // ================= Step3/Step4 输出（成员变量，供其他模块使用）=================
    Eigen::Vector3f center_;        // 物体中心（相机坐标系）
    Eigen::Vector3f v_max_;          // 物体主轴（相机坐标系）
    Eigen::Matrix3d R_init_;        // CAD → 相机 初始旋转
    Eigen::Matrix4d T_init_;        // CAD → 相机 初始位姿
    bool has_valid_pose_;           // 当前帧是否有有效位姿
    PoseQuality pose_quality_;      // 位姿质量（FULL_6DOF / DEGRADED_5DOF）

    // ================= CAD model for ICP =================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud_;  // CAD 点云（水杯侧壁）

    // ================= Callbacks =================
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);

    // ================= Main processing =================
    void process(const cv::Mat& color, const cv::Mat& depth, const rclcpp::Time& stamp);

    // ================= Point cloud filtering =================
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

} // namespace arm_controller
