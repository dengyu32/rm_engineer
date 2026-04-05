#include "detect_node/detect_node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace arm_controller
{

DetectNode::DetectNode(const rclcpp::NodeOptions &options)
    : Node("detect_node", options),
      detector_([]
                {
                    const std::string share_dir =
                        ament_index_cpp::get_package_share_directory("detect_node");
                    const std::filesystem::path model_dir =
                        std::filesystem::path(share_dir) / "models";
                    const std::string onnx_path =
                        (model_dir / "best.onnx").string();
                    const std::string names_path =
                        (model_dir / "target.names").string();
                    return yolos::seg::YOLOSegDetector(onnx_path, names_path, false);
                }()),  // CPU
      fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0),
      depth_scale_(0.001),
      has_lock_(false),
      bad_track_count_(0),
      iou_min_(0.3),
      bad_track_max_(5),
      has_valid_pose_(false),
      pose_quality_(PoseQuality::DEGRADED_5DOF),
      fallback_count_(0),
      alt_axis_count_(0),
      alt_axis_(Eigen::Vector3f::UnitZ()),
      axis_conf_min_(0.35),
      axis_smooth_alpha_(0.6),
      axis_force_flip_(false),
      cyl_voxel_leaf_(0.0035),
      cyl_normal_radius_(0.010),
      cyl_max_iter_(1000),
      cyl_dist_thresh_(0.0045),
      cyl_radius_margin_std_mult_(3.0),
      cyl_radius_margin_min_(0.005),
      cyl_use_radius_limits_(true),
      cad_axis_len_(0.0),
      cad_axis_valid_(false),
      cad_radius_mean_(0.0),
      cad_radius_std_(0.0),
      cad_radius_valid_(false)
{
    const std::string share_dir = ament_index_cpp::get_package_share_directory("detect_node");
    const std::filesystem::path model_dir = std::filesystem::path(share_dir) / "models";
    const std::string onnx_path = (model_dir / "yolo11s-seg.onnx").string();
    const std::string names_path = (model_dir / "target.names").string();

    // ================= Message Filters 时间同步 =================
    mf_color_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/camera/color/image_raw", rmw_qos_profile_sensor_data);
    mf_depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/camera/aligned_depth_to_color/image_raw", rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *mf_color_sub_, *mf_depth_sub_);
    sync_->registerCallback(std::bind(&DetectNode::syncCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/aligned_depth_to_color/camera_info",
        rclcpp::SensorDataQoS(),
        std::bind(&DetectNode::cameraInfoCallback, this, std::placeholders::_1));

    // 发布 T_init (CAD → 相机初始位姿)
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/detect/cad_initial_pose", rclcpp::SensorDataQoS());
    center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/detect/center", rclcpp::SensorDataQoS());
    axis_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/detect/v_max", rclcpp::SensorDataQoS());

    axis_conf_min_ = declare_parameter<double>("axis_conf_min", 0.35);
    axis_smooth_alpha_ = declare_parameter<double>("axis_smooth_alpha", 0.6);
    axis_force_flip_ = declare_parameter<bool>("axis_force_flip", false);
    iou_min_ = declare_parameter<double>("track_iou_min", 0.3);
    bad_track_max_ = declare_parameter<int>("track_bad_max", 5);
    cyl_voxel_leaf_ = declare_parameter<double>("cyl_voxel_leaf", 0.0035);
    cyl_normal_radius_ = declare_parameter<double>("cyl_normal_radius", 0.010);
    cyl_max_iter_ = declare_parameter<int>("cyl_max_iter", 1000);
    cyl_dist_thresh_ = declare_parameter<double>("cyl_dist_thresh", 0.0045);
    cyl_radius_margin_std_mult_ = declare_parameter<double>("cyl_radius_margin_std_mult", 3.0);
    cyl_radius_margin_min_ = declare_parameter<double>("cyl_radius_margin_min", 0.005);
    cyl_use_radius_limits_ = declare_parameter<bool>("cyl_use_radius_limits", true);

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "success";
            for (const auto& p : params) {
                const auto& name = p.get_name();
                if (name == "cyl_voxel_leaf") {
                    const double v = p.as_double();
                    if (v <= 0.0) { result.successful = false; result.reason = "cyl_voxel_leaf must be > 0"; break; }
                    cyl_voxel_leaf_ = v;
                } else if (name == "cyl_normal_radius") {
                    const double v = p.as_double();
                    if (v <= 0.0) { result.successful = false; result.reason = "cyl_normal_radius must be > 0"; break; }
                    cyl_normal_radius_ = v;
                } else if (name == "cyl_max_iter") {
                    const int v = p.as_int();
                    if (v <= 0) { result.successful = false; result.reason = "cyl_max_iter must be > 0"; break; }
                    cyl_max_iter_ = v;
                } else if (name == "cyl_dist_thresh") {
                    const double v = p.as_double();
                    if (v <= 0.0) { result.successful = false; result.reason = "cyl_dist_thresh must be > 0"; break; }
                    cyl_dist_thresh_ = v;
                } else if (name == "cyl_radius_margin_std_mult") {
                    const double v = p.as_double();
                    if (v < 0.0) { result.successful = false; result.reason = "cyl_radius_margin_std_mult must be >= 0"; break; }
                    cyl_radius_margin_std_mult_ = v;
                } else if (name == "cyl_radius_margin_min") {
                    const double v = p.as_double();
                    if (v < 0.0) { result.successful = false; result.reason = "cyl_radius_margin_min must be >= 0"; break; }
                    cyl_radius_margin_min_ = v;
                } else if (name == "cyl_use_radius_limits") {
                    cyl_use_radius_limits_ = p.as_bool();
                } else if (name == "axis_force_flip") {
                    axis_force_flip_ = p.as_bool();
                }
            }
            return result;
        });

    // ================= 加载 CAD 点云（一次性） =================
    cad_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    const std::string cad_path = (model_dir / "水杯_cad_sidewall.pcd").string();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(cad_path, *cad_cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load CAD point cloud from %s", cad_path.c_str());
        cad_cloud_->clear();
    } else {
        std::vector<double> rs;
        rs.reserve(cad_cloud_->size());
        float z_min = std::numeric_limits<float>::infinity();
        float z_max = -std::numeric_limits<float>::infinity();
        for (const auto& pt : cad_cloud_->points) {
            if (!std::isfinite(pt.z)) continue;
            z_min = std::min(z_min, pt.z);
            z_max = std::max(z_max, pt.z);
            const double r = std::hypot(static_cast<double>(pt.x), static_cast<double>(pt.y));
            if (std::isfinite(r)) rs.push_back(r);
        }
        if (std::isfinite(z_min) && std::isfinite(z_max) && z_max > z_min) {
            cad_axis_len_ = static_cast<double>(z_max - z_min);
            cad_axis_valid_ = true;
            RCLCPP_INFO(get_logger(), "[DetectNode] CAD axis length (Z): %.4f", cad_axis_len_);
        } else {
            cad_axis_valid_ = false;
            RCLCPP_WARN(get_logger(), "[DetectNode] CAD axis length invalid, z_min=%.4f z_max=%.4f", z_min, z_max);
        }
        RCLCPP_INFO(get_logger(), "[DetectNode] Loaded CAD point cloud: %zu points from %s",
                    cad_cloud_->size(), cad_path.c_str());

        if (rs.size() >= 100) {
            double sum = 0.0;
            for (double r : rs) sum += r;
            cad_radius_mean_ = sum / static_cast<double>(rs.size());
            double var = 0.0;
            for (double r : rs) {
                const double d = r - cad_radius_mean_;
                var += d * d;
            }
            cad_radius_std_ = std::sqrt(var / static_cast<double>(rs.size()));
            cad_radius_valid_ = std::isfinite(cad_radius_mean_) && std::isfinite(cad_radius_std_) && cad_radius_mean_ > 1e-6;
            if (cad_radius_valid_) {
                RCLCPP_INFO(get_logger(),
                    "[DetectNode] CAD radius mean=%.4f std=%.4f (m)",
                    cad_radius_mean_, cad_radius_std_);
            } else {
                RCLCPP_WARN(get_logger(), "[DetectNode] CAD radius invalid, mean=%.4f std=%.4f", cad_radius_mean_, cad_radius_std_);
            }
        } else {
            cad_radius_valid_ = false;
            RCLCPP_WARN(get_logger(), "[DetectNode] CAD radius stats invalid, rs.size=%zu", rs.size());
        }
    }

    RCLCPP_INFO(get_logger(), "[DetectNode] started with message_filters sync");
    RCLCPP_INFO(get_logger(), "[DetectNode] Model path: %s", onnx_path.c_str());
    RCLCPP_INFO(get_logger(), "[DetectNode] Names path: %s", names_path.c_str());
    RCLCPP_INFO(get_logger(), "[DetectNode] Model exists: %s", std::filesystem::exists(onnx_path) ? "yes" : "no");
    RCLCPP_INFO(get_logger(), "[DetectNode] Names exists: %s", std::filesystem::exists(names_path) ? "yes" : "no");
    if (std::filesystem::exists(onnx_path)) {
        const auto size = std::filesystem::file_size(onnx_path);
        RCLCPP_INFO(get_logger(), "[DetectNode] Model size: %zu bytes", static_cast<size_t>(size));
    }
    {
        const auto& names = detector_.getClassNames();
        RCLCPP_INFO(get_logger(), "[DetectNode] Class count: %zu", names.size());
        if (!names.empty()) {
            std::string first = names.front();
            std::string last = names.back();
            RCLCPP_INFO(get_logger(), "[DetectNode] First class: %s", first.c_str());
            RCLCPP_INFO(get_logger(), "[DetectNode] Last class: %s", last.c_str());
        }
    }

    // Create the visualization window explicitly to avoid lazy creation issues in component threads.
    cv::namedWindow("segmentation", cv::WINDOW_NORMAL);
    cv::startWindowThread();
}

DetectNode::~DetectNode()
{
    cv::destroyWindow("segmentation");
}

void DetectNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
}

void DetectNode::syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
{
    const int64_t max_pixels = 20000000;  // 20 MP sanity limit
    if (color_msg->width == 0 || color_msg->height == 0 ||
        depth_msg->width == 0 || depth_msg->height == 0)
    {
        RCLCPP_ERROR(get_logger(), "Empty image meta (color %ux%u, depth %ux%u), skip",
                     color_msg->width, color_msg->height,
                     depth_msg->width, depth_msg->height);
        return;
    }
    if (static_cast<int64_t>(color_msg->width) * static_cast<int64_t>(color_msg->height) > max_pixels ||
        static_cast<int64_t>(depth_msg->width) * static_cast<int64_t>(depth_msg->height) > max_pixels)
    {
        RCLCPP_ERROR(get_logger(), "Abnormal image meta (color %ux%u, depth %ux%u), skip",
                     color_msg->width, color_msg->height,
                     depth_msg->width, depth_msg->height);
        return;
    }

    // 转换彩色图像
    cv::Mat color = cv_bridge::toCvShare(color_msg, "bgr8")->image;

    // 转换深度图像
    cv::Mat depth;
    if (depth_msg->encoding == "16UC1")
    {
        depth = cv_bridge::toCvShare(depth_msg, "16UC1")->image;
        depth_scale_ = 0.001;
    }
    else if (depth_msg->encoding == "32FC1")
    {
        depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
        depth_scale_ = 1.0;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
        return;
    }

    // Clone after basic sanity checks to own the buffer.
    color = color.clone();
    depth = depth.clone();

    // 处理同步的帧
    process(color, depth, color_msg->header.stamp);
}


void DetectNode::process(const cv::Mat& color, const cv::Mat& depth, const rclcpp::Time& stamp)
{
    if (color.empty() || depth.empty()) return;
    if (color.rows <= 0 || color.cols <= 0 || depth.rows <= 0 || depth.cols <= 0) return;
    // Guard against corrupted frames that can cause huge allocations downstream.
    const int64_t max_pixels = 20000000;  // 20 MP sanity limit
    if (static_cast<int64_t>(color.total()) > max_pixels ||
        static_cast<int64_t>(depth.total()) > max_pixels)
    {
        RCLCPP_ERROR(get_logger(),
            "[DetectNode] Abnormal image size color=%dx%d depth=%dx%d, skip frame",
            color.cols, color.rows, depth.cols, depth.rows);
        return;
    }
    if (fx_ < 1e-6 || fy_ < 1e-6) return;
    // 已由 message_filters 同步，无需 dt 检查

    // ==================== 对齐自检：检查 color 和 depth 分辨率是否匹配 ====================
    static bool alignment_checked = false;
    if (!alignment_checked)
    {
        RCLCPP_INFO(get_logger(), "[Alignment Check] color: %dx%d, depth: %dx%d",
                    color.cols, color.rows,
                    depth.cols, depth.rows);

        if (color.cols != depth.cols || color.rows != depth.rows)
        {
            RCLCPP_ERROR(get_logger(),
                "[Alignment Check] FAILED: color(%dx%d) != depth(%dx%d). "
                "Check RealSense launch config (enable align_depth_to_color!)",
                color.cols, color.rows,
                depth.cols, depth.rows);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "[Alignment Check] PASSED: resolutions match");
        }
        alignment_checked = true;
    }

    // 如果分辨率不匹配，跳过处理（避免坐标错误）
    if (color.cols != depth.cols || color.rows != depth.rows)
    {
        return;
    }

    // clone 快照（避免数据竞争）
    cv::Mat color_local = color.clone();
    cv::Mat depth_local = depth.clone();

    const int img_w = color_local.cols;
    const int img_h = color_local.rows;
    const int border_margin = 20;  // 贴边阈值：像素

    // YOLO 分割检测
    cv::Mat vis = color_local.clone();
    std::vector<SegObject> results = runSegmentation(color_local, vis);

    bool frame_valid = false;  // 整帧是否有效
    // has_valid_pose_ 不再每帧重置，保留跨帧锚点

    if (!results.empty())
    {
        auto iou_bbox = [](const cv::Rect& a, const cv::Rect& b) -> float
        {
            const int x1 = std::max(a.x, b.x);
            const int y1 = std::max(a.y, b.y);
            const int x2 = std::min(a.x + a.width, b.x + b.width);
            const int y2 = std::min(a.y + a.height, b.y + b.height);
            const int w = x2 - x1;
            const int h = y2 - y1;
            if (w <= 0 || h <= 0) return 0.0f;
            const float inter = static_cast<float>(w * h);
            const float uni = static_cast<float>(a.area() + b.area()) - inter;
            return (uni > 1e-6f) ? (inter / uni) : 0.0f;
        };

        SegObject* selected = nullptr;
        if (has_lock_) {
            float best_iou = -1.0f;
            size_t best_i = 0;
            for (size_t i = 0; i < results.size(); ++i) {
                const float iou = iou_bbox(locked_bbox_, results[i].bbox);
                if (iou > best_iou) {
                    best_iou = iou;
                    best_i = i;
                }
            }
            if (best_iou >= static_cast<float>(iou_min_)) {
                selected = &results[best_i];
                bad_track_count_ = 0;
            } else {
                bad_track_count_++;
                if (bad_track_count_ >= bad_track_max_) {
                    size_t max_i = 0;
                    for (size_t i = 1; i < results.size(); ++i) {
                        if (results[i].conf > results[max_i].conf) max_i = i;
                    }
                    selected = &results[max_i];
                    bad_track_count_ = 0;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                        "[detect_node] Track reset by low IoU (%.2f < %.2f)",
                        best_iou, static_cast<float>(iou_min_));
                }
            }
        } else {
            size_t max_i = 0;
            for (size_t i = 1; i < results.size(); ++i) {
                if (results[i].conf > results[max_i].conf) max_i = i;
            }
            selected = &results[max_i];
            bad_track_count_ = 0;
        }

        if (selected) {
            locked_bbox_ = selected->bbox;
            has_lock_ = true;
        }

        const auto *objp = selected;
        bool skip = false;

        if (!objp) {
            skip = true;
        }

        do {
            if (skip) break;
            const auto &obj = *objp;
            if (obj.conf < 0.5f || obj.mask.empty()) { skip = true; break; }

            // -------------------- Step 0: Bbox 贴边检查（深度缺失高风险） --------------------
            bool is_at_border = isBboxAtBorder(obj.bbox, img_w, img_h, border_margin);

            if (is_at_border) {
                RCLCPP_WARN(get_logger(), "Class %d bbox at border, skip", obj.class_id);
                skip = true;
                break;
            }

            // -------------------- Step 1: 采样 mask 非零像素（控制点数量） --------------------
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
            std::vector<float> z_vals;
            size_t mask_nz = 0;
            int zero_depth = 0;
            int out_of_range = 0;
            float valid_ratio = 0.0f;
            size_t sampled_total = 0;
            if (!buildRawCloudFromMask(
                    obj.mask, depth_local, stamp,
                    raw_cloud, z_vals, mask_nz,
                    zero_depth, out_of_range,
                    valid_ratio, sampled_total)) {
                skip = true;
                break;
            }

            // -------------------- Step 0.5: valid_ratio 两级门禁 --------------------
            // 改动2：两级门禁（HOLD / DEGRADED / 正常）
            const float min_valid_ratio_hold = 0.15f;   // HOLD 阈值
            const float min_valid_ratio_normal = 0.30f;  // 正常阈值

            if (valid_ratio < min_valid_ratio_hold) {
                // HOLD：不更新，但别让整帧直接无效（留给其他对象）
                RCLCPP_WARN(get_logger(), "Class %d valid_ratio=%.2f < %.2f, HOLD (no update)",
                            obj.class_id, valid_ratio, min_valid_ratio_hold);
                skip = true;
                break;
            }

            // DEGRADED 模式：继续处理，但只更新 center，不更新轴
            const bool is_degraded = (valid_ratio < min_valid_ratio_normal);
            if (is_degraded) {
                RCLCPP_WARN(get_logger(), "Class %d valid_ratio=%.2f [DEGRADED], continue with lower confidence",
                            obj.class_id, valid_ratio);
            }

            const size_t n_raw = raw_cloud->size();

            // -------------------- Step 2: 深度分位数带通（距离自适应 margin） --------------------
            pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud;
            float z_lo = 0.0f;
            float z_hi = 0.0f;
            if (!depthBandPass(raw_cloud, z_vals, depth_cloud, z_lo, z_hi)) { skip = true; break; }

            const size_t n_depth = depth_cloud->size();

            // -------------------- Step 3: 过滤（voxel → 最大簇 → ROR） --------------------
            filterPointCloud(depth_cloud);

            const size_t n_final = depth_cloud->size();

            RCLCPP_INFO(get_logger(),
                "Class %d conf=%.2f mask_nz=%zu raw=%zu depth=%zu final=%zu valid_ratio=%.2f z=[%.3f,%.3f] zero=%d oor=%d",
                obj.class_id, obj.conf, mask_nz, n_raw, n_depth, n_final, valid_ratio, z_lo, z_hi, zero_depth, out_of_range);

            if (depth_cloud->empty()) { skip = true; break; }

            // 改动3：降低点数门禁（200 → 100）
            if (n_final < 100) {
                RCLCPP_WARN(get_logger(), "Class %d final points=%zu < 100, skip", obj.class_id, n_final);
                skip = true;
                break;
            }

            // DEGRADED 模式：只更新 center，跳过轴估计和发布
            if (is_degraded) {
                // 计算简单质心作为 center
                Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
                for (const auto& pt : depth_cloud->points) {
                    center += Eigen::Vector3f(pt.x, pt.y, pt.z);
                }
                center /= static_cast<float>(depth_cloud->size());

                // 改动: DEGRADED 模式也应用平滑，防止跳变
                const float alpha = 0.65f;
                if (has_valid_pose_) {
                    center_ = alpha * center + (1.0f - alpha) * center_;
                } else {
                    center_ = center;
                }
                frame_valid = true;
                RCLCPP_INFO(get_logger(), "Class %d DEGRADED: updated center only [%.3f,%.3f,%.3f]",
                            obj.class_id, center.x(), center.y(), center.z());
                break;
            }

            // 正常模式：完整处理
            frame_valid = true;
            bool icp_success = false;
            if (!estimatePoseAndPublish(depth_cloud, obj.mask, obj.class_id, stamp, icp_success)) {
                skip = true;
                break;
            }

            // -------------------- 可视化：中心点和中心轴 --------------------
            if (center_.z() > 1e-6f)
            {
                const float depth_center = center_.z();
                const float arrow_length_3d = std::clamp(0.15f * (depth_center / 1.0f), 0.05f, 0.20f);
                const Eigen::Vector3f arrow_end_3d = center_ + v_max_ * arrow_length_3d;

                const int u_center = static_cast<int>(fx_ * center_.x() / center_.z() + cx_);
                const int v_center = static_cast<int>(fy_ * center_.y() / center_.z() + cy_);
                const int u_arrow = static_cast<int>(fx_ * arrow_end_3d.x() / arrow_end_3d.z() + cx_);
                const int v_arrow = static_cast<int>(fy_ * arrow_end_3d.y() / arrow_end_3d.z() + cy_);

                if (u_center >= 0 && v_center >= 0 && u_center < vis.cols && v_center < vis.rows)
                {
                    const int cross_size = 10;
                    cv::line(vis, cv::Point(u_center - cross_size, v_center), cv::Point(u_center + cross_size, v_center),
                             cv::Scalar(0, 0, 255), 3);
                    cv::line(vis, cv::Point(u_center, v_center - cross_size), cv::Point(u_center, v_center + cross_size),
                             cv::Scalar(0, 0, 255), 3);

                    char axis_text[64];
                    snprintf(axis_text, sizeof(axis_text), "Z: [%.2f,%.2f,%.2f]", v_max_.x(), v_max_.y(), v_max_.z());
                    cv::putText(vis, axis_text, cv::Point(u_center + 20, v_center - 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

                    char pos_text[64];
                    snprintf(pos_text, sizeof(pos_text), "Pos: [%.3f,%.3f,%.3f]", center_.x(), center_.y(), center_.z());
                    cv::putText(vis, pos_text, cv::Point(u_center + 20, v_center + 10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                }

                if (arrow_end_3d.z() > 1e-6f)
                {
                    const int arrow_head_len = 15;
                    cv::arrowedLine(vis, cv::Point(u_center, v_center), cv::Point(u_arrow, v_arrow),
                                   cv::Scalar(0, 255, 255), 3, cv::LINE_AA, 0, arrow_head_len);
                }
            }

            // -------------------- 可视化：画过滤后的点（绿色） --------------------
            for (const auto &pt : depth_cloud->points)
            {
                const int u = static_cast<int>(fx_ * pt.x / pt.z + cx_);
                const int v = static_cast<int>(fy_ * pt.y / pt.z + cy_);
                if (u >= 0 && v >= 0 && u < vis.cols && v < vis.rows) {
                    vis.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 255, 0);
                }
            }
        } while (false);

        (void)skip;
    }
    else
    {
        if (has_lock_) {
            bad_track_count_++;
            if (bad_track_count_ >= bad_track_max_) {
                has_lock_ = false;
                bad_track_count_ = 0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                    "[detect_node] Track lost: no candidates");
            }
        }
    }

    // ==================== Hold 逻辑：整帧无效时复用上一帧 ====================
    if (!frame_valid)
    {
        if (!last_valid_vis_.empty())
        {
            const double elapsed = (now() - last_valid_time_).seconds();
            if (elapsed < HOLD_SEC)
            {
                RCLCPP_WARN(get_logger(), "Frame invalid, hold last valid (%.2fs remaining)", HOLD_SEC - elapsed);
                vis = last_valid_vis_;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Frame invalid and hold timeout, fallback to raw color");
                vis = color_local.clone();
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Frame invalid and no previous valid, fallback to raw color");
            vis = color_local.clone();
        }
    }
    else
    {
        last_valid_vis_ = vis.clone();
        last_valid_time_ = now();
    }

    cv::imshow("segmentation", vis);
    cv::waitKey(1);
}

} // namespace arm_controller

RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::DetectNode)
