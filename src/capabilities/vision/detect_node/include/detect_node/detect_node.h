#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
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
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>

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
    ~DetectNode() override;

private:
    struct SegObject
    {
        int class_id = -1;
        float conf = 0.0f;
        cv::Mat mask;
        cv::Rect bbox;
    };

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
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr axis_pub_;

    // ================= YOLO Detector =================
    yolos::seg::YOLOSegDetector detector_;

    // ================= Camera parameters =================
    double fx_, fy_, cx_, cy_;
    double depth_scale_;

    // ================= Target tracking (IoU) =================
    bool has_lock_;
    cv::Rect locked_bbox_;
    int bad_track_count_;
    double iou_min_;
    int bad_track_max_;

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
    int fallback_count_;            // 连续 FALLBACK 次数
    int alt_axis_count_;            // 连续一致备选轴计数
    Eigen::Vector3f alt_axis_;      // 备选轴方向
    double axis_conf_min_;          // 主轴方向置信度门限（密度不对称）
    double axis_smooth_alpha_;      // 主轴方向平滑系数
    bool axis_force_flip_;          // 强制翻转轴方向（调试用）

    // ================= Cylinder RANSAC params =================
    double cyl_voxel_leaf_;                 // 体素下采样大小 (m)
    double cyl_normal_radius_;              // 法向估计半径 (m)
    int cyl_max_iter_;                      // RANSAC 最大迭代次数
    double cyl_dist_thresh_;                // 点到模型距离阈值 (m)
    double cyl_radius_margin_std_mult_;     // 半径范围 std 倍数
    double cyl_radius_margin_min_;          // 半径范围最小扩展 (m)
    bool cyl_use_radius_limits_;            // 是否启用半径限制

    // ================= Parameters callback =================
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ================= CAD model for ICP =================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud_;  // CAD 点云（水杯侧壁）
    double cad_axis_len_;                            // CAD 轴向长度（Z 方向）
    bool cad_axis_valid_;                            // CAD 轴向长度有效
    double cad_radius_mean_;                         // CAD 侧壁半径均值
    double cad_radius_std_;                          // CAD 侧壁半径标准差
    bool cad_radius_valid_;                          // CAD 侧壁半径有效

    // ================= Callbacks =================
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);

    // ================= Main processing =================
    void process(const cv::Mat& color, const cv::Mat& depth, const rclcpp::Time& stamp);

    // ================= Point cloud filtering =================
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // ================= Utilities / ICP helpers =================
    std::vector<SegObject> runSegmentation(const cv::Mat& color, cv::Mat& vis);
    bool isBboxAtBorder(const cv::Rect& bbox, int img_w, int img_h, int border_margin) const;

    bool buildRawCloudFromMask(
        const cv::Mat& mask,
        const cv::Mat& depth,
        const rclcpp::Time& stamp,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud,
        std::vector<float>& z_vals,
        size_t& mask_nz,
        int& zero_depth,
        int& out_of_range,
        float& valid_ratio,
        size_t& sampled_total) const;

    bool depthBandPass(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud,
        const std::vector<float>& z_vals,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& depth_cloud,
        float& z_lo,
        float& z_hi) const;

    float quantileInplace(std::vector<float>& v, float q) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float leaf) const;
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float radius) const;
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormalsOnly(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float radius) const;
    bool fitCylinderAxis(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obs_xyz,
        float voxel_leaf,
        float normal_radius,
        int max_iter,
        float dist_thresh,
        float radius_min,
        float radius_max,
        Eigen::Vector3f& axis_out,
        Eigen::Vector3f& point_on_axis_out,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers_cloud_out,
        float& radius_out) const;
    bool icpPointToPlaneOneLevel(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cad_xyz,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obs_xyz,
        const Eigen::Matrix4f& init_guess,
        float voxel_leaf,
        float max_corr_dist,
        int max_iter,
        float normal_radius,
        float trim_ratio,
        Eigen::Matrix4f& T_out,
        float& fitness_out) const;

    bool estimatePoseAndPublish(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& depth_cloud,
        const cv::Mat& mask,
        int class_id,
        const rclcpp::Time& stamp,
        bool& icp_success);
};

} // namespace arm_controller
