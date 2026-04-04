#include "detect_node/detect_node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace arm_controller
{

DetectNode::DetectNode(const rclcpp::NodeOptions &options)
    : Node("detect_node", options),
      fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0),
      depth_scale_(0.001),
      has_valid_pose_(false),
      pose_quality_(PoseQuality::DEGRADED_5DOF)
{
    const auto share_dir = ament_index_cpp::get_package_share_directory("detect_node");
    const auto default_model_path = (std::filesystem::path(share_dir) / "models" / "yolo11s-seg.onnx").string();
    const auto default_labels_path = (std::filesystem::path(share_dir) / "models" / "coco.names").string();

    const auto model_path = declare_parameter<std::string>("model_path", default_model_path);
    const auto labels_path = declare_parameter<std::string>("labels_path", default_labels_path);
    const auto cad_path = declare_parameter<std::string>("cad_path", "");
    const auto use_gpu = declare_parameter<bool>("use_gpu", true);

    detector_ = std::make_unique<yolos::seg::YOLOSegDetector>(model_path, labels_path, use_gpu);

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

    // ================= 加载 CAD 点云（一次性） =================
    cad_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if (cad_path.empty()) {
        RCLCPP_WARN(get_logger(), "[DetectNode] cad_path parameter is empty; ICP alignment will be disabled.");
        cad_cloud_->clear();
    } else if (pcl::io::loadPCDFile<pcl::PointXYZ>(cad_path, *cad_cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load CAD point cloud from %s", cad_path.c_str());
        cad_cloud_->clear();
    } else {
        RCLCPP_INFO(get_logger(), "[DetectNode] Loaded CAD point cloud: %zu points from %s",
                    cad_cloud_->size(), cad_path.c_str());
    }

    RCLCPP_INFO(get_logger(), "[DetectNode] started with message_filters sync");
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
    // 转换彩色图像
    cv::Mat color = cv_bridge::toCvShare(color_msg, "bgr8")->image.clone();

    // 转换深度图像
    cv::Mat depth;
    if (depth_msg->encoding == "16UC1")
    {
        depth = cv_bridge::toCvShare(depth_msg, "16UC1")->image.clone();
        depth_scale_ = 0.001;
    }
    else if (depth_msg->encoding == "32FC1")
    {
        depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image.clone();
        depth_scale_ = 1.0;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
        return;
    }

    // 处理同步的帧
    process(color, depth, color_msg->header.stamp);
}

// ==================== 小工具：用 nth_element 求分位数 ====================
static float quantile_inplace(std::vector<float>& v, float q)
{
    if (v.empty()) return 0.0f;
    if (q < 0.0f) q = 0.0f;
    if (q > 1.0f) q = 1.0f;

    const size_t k = static_cast<size_t>(q * (v.size() - 1));
    auto it = v.begin() + k;
    std::nth_element(v.begin(), it, v.end());
    return *it;
}

// ==================== Step6 ICP 辅助函数 ====================

// 1) Voxel 下采样（带空输入保护）
static pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float leaf)
{
    if (!in || in->empty() || leaf < 1e-6f) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(in);
    voxel.setLeafSize(leaf, leaf, leaf);
    auto out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*out);
    return out;
}

// 2) 法线估计（点到平面必备，带 NaN 法线过滤）
static pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float radius)
{
    if (!in || in->empty() || radius < 1e-6f) {
        return pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    }

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(in);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*normals);

    // 门禁：normals->size() 必须等于 in->size()，否则越界
    if (!normals || normals->size() != in->size()) {
        return pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    }

    auto out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    out->reserve(in->size());

    for (size_t i = 0; i < in->size(); ++i) {
        // 法线有效性检查：必须 finite（非 NaN、非无限）
        const float nx = (*normals)[i].normal_x;
        const float ny = (*normals)[i].normal_y;
        const float nz = (*normals)[i].normal_z;
        if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz)) {
            continue;  // 跳过无效法线
        }

        pcl::PointNormal pn;
        pn.x = (*in)[i].x; pn.y = (*in)[i].y; pn.z = (*in)[i].z;
        pn.normal_x = nx; pn.normal_y = ny; pn.normal_z = nz;
        out->push_back(pn);
    }
    return out;
}

// 3) 单层点到平面 ICP
static bool icpPointToPlaneOneLevel(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cad_xyz,     // source: CAD (CAD系)
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obs_xyz,     // target: 观测 (cam系)
    const Eigen::Matrix4f& init_guess,                         // CAD->cam 初值
    float voxel_leaf,
    float max_corr_dist,
    int max_iter,
    float normal_radius,
    float trim_ratio,                                         // 鲁棒 trim 比例（0.9f 或 0.95f）
    Eigen::Matrix4f& T_out,                                    // 输出 CAD->cam
    float& fitness_out)
{
    (void)trim_ratio;
    fitness_out = 1e9f;
    T_out = init_guess;

    if (!cad_xyz || cad_xyz->empty() || !obs_xyz || obs_xyz->empty()) return false;

    // 1) 同尺度 voxel
    auto cad_ds = voxelDownsample(cad_xyz, voxel_leaf);
    auto obs_ds = voxelDownsample(obs_xyz, voxel_leaf);
    if (!cad_ds || cad_ds->empty() || !obs_ds || obs_ds->empty()) return false;

    // 2) NaN 清理（非常关键）
    std::vector<int> idx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_clean(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_clean(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::removeNaNFromPointCloud(*cad_ds, *cad_clean, idx);
    pcl::removeNaNFromPointCloud(*obs_ds, *obs_clean, idx);

    if (cad_clean->size() < 80 || obs_clean->size() < 80) return false;

    // 3) 法线（点到平面必备）
    auto cad_pn = estimateNormals(cad_clean, normal_radius);
    auto obs_pn = estimateNormals(obs_clean, normal_radius);
    if (!cad_pn || cad_pn->empty() || !obs_pn || obs_pn->empty()) return false;

    // 4) ICP point-to-plane
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(cad_pn);
    icp.setInputTarget(obs_pn);

    icp.setMaximumIterations(max_iter);
    icp.setMaxCorrespondenceDistance(max_corr_dist);
    icp.setTransformationEpsilon(1e-4f);
    icp.setEuclideanFitnessEpsilon(1e-4f);

    // point-to-plane 估计器
    icp.setTransformationEstimation(
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr(
            new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>()
        )
    );

    // 5) 鲁棒 trim（PCL 版本不支持 CorrespondenceRejectorTrimmed，改用简单方式）
    // 暂时不做 trim，直接对齐
    pcl::PointCloud<pcl::PointNormal> aligned;
    icp.align(aligned, init_guess);

    if (!icp.hasConverged()) return false;

    T_out = icp.getFinalTransformation();               // 直接就是 CAD->cam
    fitness_out = static_cast<float>(icp.getFitnessScore());
    return true;
}

void DetectNode::process(const cv::Mat& color, const cv::Mat& depth, const rclcpp::Time& stamp)
{
    if (color.empty() || depth.empty()) return;
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
    std::vector<yolos::seg::Segmentation> results =
        detector_->segment(color_local, 0.5f, 0.5f);

    // 显示可视化（原图 + mask + 点云）
    cv::Mat vis = color_local.clone();

    bool frame_valid = false;  // 整帧是否有效
    has_valid_pose_ = false;  // 每帧开始时重置

    if (!results.empty())
    {
        // 绘制 mask
        detector_->drawMasksOnly(vis, results, 0.6f);

        for (const auto &obj : results)
        {
            if (obj.conf < 0.5f || obj.mask.empty()) continue;

            // -------------------- Step 0: Bbox 贴边检查（深度缺失高风险） --------------------
            cv::Rect bbox(obj.box.x, obj.box.y, obj.box.width, obj.box.height);
            bool is_at_border = (bbox.x < border_margin) ||
                                (bbox.y < border_margin) ||
                                (bbox.x + bbox.width > img_w - border_margin) ||
                                (bbox.y + bbox.height > img_h - border_margin);

            if (is_at_border) {
                RCLCPP_WARN(get_logger(), "Class %d bbox at border, skip", obj.classId);
                continue;
            }

            // -------------------- Step 1: 采样 mask 非零像素（控制点数量） --------------------
            std::vector<cv::Point> nz;
            cv::findNonZero(obj.mask, nz);
            if (nz.empty()) continue;

            // 改动4：改为随机抽样（减少统计抖动）
            const size_t max_sample = 6000;
            const size_t sample_count = std::min(nz.size(), max_sample);

            // 随机抽样（每帧局部 seed，避免静态共享副作用）
            std::vector<size_t> indices(nz.size());
            for (size_t i = 0; i < nz.size(); ++i) indices[i] = i;
            std::mt19937 rng(static_cast<unsigned int>(stamp.nanoseconds()));  // 每帧不同 seed
            std::shuffle(indices.begin(), indices.end(), rng);

            // 计算 valid 深度采样数（用于 valid_ratio 门禁）
            int valid_sampled = 0;
            int sampled_total = 0;

            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            raw_cloud->reserve(sample_count);

            std::vector<float> z_vals;
            z_vals.reserve(sample_count);

            int zero_depth = 0;
            int out_of_range = 0;

            for (size_t idx = 0; idx < sample_count; ++idx)
            {
                const size_t i = indices[idx];
                const int x = nz[i].x;
                const int y = nz[i].y;

                if (x < 0 || y < 0 || x >= depth_local.cols || y >= depth_local.rows) continue;

                float Z = 0.0f;
                if (depth_local.type() == CV_16UC1)
                {
                    const uint16_t d = depth_local.at<uint16_t>(y, x);
                    if (d == 0) { zero_depth++; continue; }
                    Z = static_cast<float>(d) * static_cast<float>(depth_scale_);
                }
                else // CV_32FC1
                {
                    const float d = depth_local.at<float>(y, x);
                    if (!std::isfinite(d) || d < 1e-6f) { zero_depth++; continue; }
                    Z = d;
                }

                if (Z < 0.1f || Z > 2.0f) { out_of_range++; continue; }

                const float X = (static_cast<float>(x) - static_cast<float>(cx_)) * Z / static_cast<float>(fx_);
                const float Y = (static_cast<float>(y) - static_cast<float>(cy_)) * Z / static_cast<float>(fy_);

                sampled_total++;  // 只统计坐标合法的抽样点
                raw_cloud->push_back(pcl::PointXYZ(X, Y, Z));
                z_vals.push_back(Z);
                valid_sampled++;
            }

            if (raw_cloud->empty()) continue;

            // -------------------- Step 0.5: valid_ratio 两级门禁 --------------------
            // 改动2：两级门禁（HOLD / DEGRADED / 正常）
            const float valid_ratio = (sampled_total > 0) ? static_cast<float>(valid_sampled) / sampled_total : 0.0f;
            const float min_valid_ratio_hold = 0.15f;   // HOLD 阈值
            const float min_valid_ratio_normal = 0.30f;  // 正常阈值

            if (valid_ratio < min_valid_ratio_hold) {
                // HOLD：不更新，但别让整帧直接无效（留给其他对象）
                RCLCPP_WARN(get_logger(), "Class %d valid_ratio=%.2f < %.2f, HOLD (no update)",
                            obj.classId, valid_ratio, min_valid_ratio_hold);
                continue;
            }

            // DEGRADED 模式：继续处理，但只更新 center，不更新轴
            const bool is_degraded = (valid_ratio < min_valid_ratio_normal);
            if (is_degraded) {
                RCLCPP_WARN(get_logger(), "Class %d valid_ratio=%.2f [DEGRADED], continue with lower confidence",
                            obj.classId, valid_ratio);
            }

            const size_t n_raw = raw_cloud->size();

            // -------------------- Step 2: 深度分位数带通（距离自适应 margin） --------------------
            std::vector<float> z_tmp = z_vals;
            const float z25 = quantile_inplace(z_tmp, 0.25f);

            z_tmp = z_vals;
            const float z75 = quantile_inplace(z_tmp, 0.75f);

            z_tmp = z_vals;
            const float z50 = quantile_inplace(z_tmp, 0.50f);

            // 距离自适应 margin：至少 3cm，远距离再放大
            const float margin = std::max(0.03f, 0.03f * z50);
            float z_lo = z25 - margin;
            float z_hi = z75 + margin;

            const float min_band = 0.02f;
            if (z_hi - z_lo < min_band) {
                z_lo = z50 - 0.03f;
                z_hi = z50 + 0.03f;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            depth_cloud->reserve(raw_cloud->size());

            for (const auto& pt : raw_cloud->points) {
                if (pt.z >= z_lo && pt.z <= z_hi) depth_cloud->push_back(pt);
            }

            if (depth_cloud->empty()) continue;

            const size_t n_depth = depth_cloud->size();

            // -------------------- Step 3: 过滤（voxel → 最大簇 → ROR） --------------------
            filterPointCloud(depth_cloud);

            const size_t n_final = depth_cloud->size();

            RCLCPP_INFO(get_logger(),
                "Class %d conf=%.2f mask_nz=%zu raw=%zu depth=%zu final=%zu valid_ratio=%.2f z=[%.3f,%.3f] zero=%d oor=%d",
                obj.classId, obj.conf, nz.size(), n_raw, n_depth, n_final, valid_ratio, z_lo, z_hi, zero_depth, out_of_range);

            if (depth_cloud->empty()) continue;

            // 改动3：降低点数门禁（200 → 100）
            if (n_final < 100) {
                RCLCPP_WARN(get_logger(), "Class %d final points=%zu < 100, skip", obj.classId, n_final);
                continue;
            }

            // DEGRADED 模式：只更新 center，跳过轴估计和发布
            if (is_degraded) {
                // 计算简单质心作为 center
                Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
                for (const auto& pt : depth_cloud->points) {
                    center += Eigen::Vector3f(pt.x, pt.y, pt.z);
                }
                center /= static_cast<float>(depth_cloud->size());
                center_ = center;  // 仅更新 center
                frame_valid = true;
                RCLCPP_INFO(get_logger(), "Class %d DEGRADED: updated center only [%.3f,%.3f,%.3f]",
                            obj.classId, center.x(), center.y(), center.z());
                continue;
            }

            // 正常模式：完整处理
            frame_valid = true;

            // -------------------- Step 3A-1: PCA 主成分分析（得到 v_max/v_min） --------------------
            Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
            for (const auto& pt : depth_cloud->points) {
                centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
            }
            centroid /= static_cast<float>(depth_cloud->size());

            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (const auto& pt : depth_cloud->points) {
                Eigen::Vector3f d(pt.x - centroid.x(), pt.y - centroid.y(), pt.z - centroid.z());
                cov += d * d.transpose();
            }
            const float denom = std::max<size_t>(depth_cloud->size() - 1, 1);
            cov /= denom;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov);
            if (eigensolver.info() != Eigen::Success) {
                RCLCPP_WARN(get_logger(), "Class %d PCA eigen decomposition failed", obj.classId);
                continue;
            }

            const Eigen::Vector3f eigenvalues = eigensolver.eigenvalues();
            const Eigen::Matrix3f eigenvectors = eigensolver.eigenvectors();

            Eigen::Vector3f v_min = eigenvectors.col(0);
            Eigen::Vector3f v_max = eigenvectors.col(2);

            v_max.normalize();
            v_min.normalize();

            if (v_max.z() < 0.0f) v_max = -v_max;
            if (v_min.z() < 0.0f) v_min = -v_min;

            // 跨帧方向一致（防止遮挡时主轴偶尔翻转）
            if (has_valid_pose_ && v_max.dot(v_max_) < 0.0f) v_max = -v_max;

            RCLCPP_INFO(get_logger(),
                "Class %d PCA: centroid=[%.3f,%.3f,%.3f] eigenvalues=[%.3f,%.3f,%.3f] v_max=[%.3f,%.3f,%.3f] v_min=[%.3f,%.3f,%.3f]",
                obj.classId,
                centroid.x(), centroid.y(), centroid.z(),
                eigenvalues[0], eigenvalues[1], eigenvalues[2],
                v_max.x(), v_max.y(), v_max.z(),
                v_min.x(), v_min.y(), v_min.z());

            // -------------------- Step 3B: 侧壁筛选（用 r_mode 选主体直段） --------------------
            std::vector<float> radii;
            radii.reserve(depth_cloud->size());
            for (const auto& pt : depth_cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f d = p - centroid;
                float projection = d.dot(v_max);
                Eigen::Vector3f axial = projection * v_max;
                float r = (d - axial).norm();
                radii.push_back(r);
            }

            const float bin_size = 0.002f;
            const float r_min = radii.empty() ? 0.0f : *std::min_element(radii.begin(), radii.end());
            const float r_max = radii.empty() ? 0.1f : *std::max_element(radii.begin(), radii.end());
            const float r_max_clamped = std::min(r_max, r_min + 0.08f);

            // 改动5：修复 side_points 变量遮蔽（只声明一次）
            pcl::PointCloud<pcl::PointXYZ>::Ptr side_points;

            const int num_bins = static_cast<int>(std::ceil((r_max_clamped - r_min) / bin_size));
            if (num_bins > 0) {
                std::vector<int> histogram(num_bins, 0);
                for (float r : radii) {
                    if (r > r_max_clamped) continue;
                    int bin_idx = static_cast<int>((r - r_min) / bin_size);
                    if (bin_idx >= 0 && bin_idx < num_bins) {
                        histogram[bin_idx]++;
                    }
                }

                int max_bin_idx = 0;
                int max_count = histogram[0];
                for (int i = 1; i < num_bins; ++i) {
                    if (histogram[i] > max_count) {
                        max_count = histogram[i];
                        max_bin_idx = i;
                    }
                }

                const float r_mode = r_min + (max_bin_idx + 0.5f) * bin_size;
                const float delta_r = std::max(0.002f, 0.08f * r_mode);
                const float r_lo = r_mode - delta_r;
                const float r_hi = r_mode + delta_r;

                // 改动5：修复 - 使用 reset 而不是重新声明
                side_points.reset(new pcl::PointCloud<pcl::PointXYZ>());
                side_points->reserve(depth_cloud->size());
                for (size_t i = 0; i < depth_cloud->size(); ++i) {
                    if (radii[i] >= r_lo && radii[i] <= r_hi) {
                        side_points->push_back(depth_cloud->points[i]);
                    }
                }

                const float side_ratio = static_cast<float>(side_points->size()) / depth_cloud->size();
                const bool use_refined = (side_ratio >= 0.2f && side_points->size() >= 100);

                RCLCPP_INFO(get_logger(),
                    "Class %d Step3B: r_mode=%.3f ±%.3f [%d in peak] side_n=%zu ratio=%.2f %s",
                    obj.classId, r_mode, delta_r, max_count,
                    side_points->size(), side_ratio,
                    use_refined ? "REFINE" : "FALLBACK");

                if (use_refined) {
                    Eigen::Vector3f centroid_refined(0.0f, 0.0f, 0.0f);
                    for (const auto& pt : side_points->points) {
                        centroid_refined += Eigen::Vector3f(pt.x, pt.y, pt.z);
                    }
                    centroid_refined /= static_cast<float>(side_points->size());

                    Eigen::Matrix3f cov_refined = Eigen::Matrix3f::Zero();
                    for (const auto& pt : side_points->points) {
                        Eigen::Vector3f d(pt.x - centroid_refined.x(), pt.y - centroid_refined.y(), pt.z - centroid_refined.z());
                        cov_refined += d * d.transpose();
                    }
                    const float denom_refined = std::max<size_t>(side_points->size() - 1, 1);
                    cov_refined /= denom_refined;

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_refined(cov_refined);
                    if (eig_refined.info() == Eigen::Success) {
                        Eigen::Vector3f refined_axis = eig_refined.eigenvectors().col(2);
                        refined_axis.normalize();
                        if (refined_axis.z() < 0.0f) refined_axis = -refined_axis;

                        if (refined_axis.dot(v_max) < 0.7f) {
                            RCLCPP_WARN(get_logger(),
                                "Class %d refined_axis deviates (dot=%.2f), reject refine", obj.classId, refined_axis.dot(v_max));
                        } else {
                            RCLCPP_INFO(get_logger(),
                                "Class %d refined_axis: refined=[%.3f,%.3f,%.3f] centroid=[%.3f,%.3f,%.3f]",
                                obj.classId,
                                refined_axis.x(), refined_axis.y(), refined_axis.z(),
                                centroid_refined.x(), centroid_refined.y(), centroid_refined.z());
                            v_max = refined_axis;
                            // centroid = centroid_refined;  // 删掉：会造成轴向原点漂移 → center 上下跳
                        }
                    }
                }
            }

            // -------------------- Step 3D: 中心估计（trimmed median） --------------------
            // 轴向参考原点：优先用上一帧 center_ 作为稳定锚点，防止遮挡导致 centroid 沿轴漂移
            Eigen::Vector3f p0 = centroid;
            if (has_valid_pose_) p0 = center_;

            const bool has_valid_side = (side_points && !side_points->empty() && side_points->size() >= 100);
            const auto& pts_for_center = has_valid_side ? side_points->points : depth_cloud->points;

            std::vector<float> t_values;
            t_values.reserve(pts_for_center.size());
            for (const auto& pt : pts_for_center) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                float t = (p - p0).dot(v_max);
                t_values.push_back(t);
            }

            const float trim_ratio = 0.15f;

            Eigen::Vector3f center = p0;

            if (t_values.size() < 50) {
                RCLCPP_WARN(get_logger(), "Class %d too few points (%zu) for trimmed median, use centroid", obj.classId, t_values.size());
            } else {
                std::sort(t_values.begin(), t_values.end());

                const size_t n = t_values.size();
                const size_t lo = static_cast<size_t>(trim_ratio * n);
                const size_t hi = n - lo - 1;

                if (lo >= hi) {
                    RCLCPP_WARN(get_logger(), "Class %d trimmed range invalid, use median", obj.classId);
                    const float t_center = t_values[n / 2];
                    center = p0 + t_center * v_max;
                } else {
                    const size_t mid = (lo + hi) / 2;
                    const float t_center = t_values[mid];
                    center = p0 + t_center * v_max;
                }

                RCLCPP_INFO(get_logger(),
                    "Class %d Step3D: t_range=[%.3f,%.3f] t_center=%.3f center=[%.3f,%.3f,%.3f] use_side=%d",
                    obj.classId, t_values[lo], t_values[hi], t_values[(lo + hi) / 2],
                    center.x(), center.y(), center.z(), has_valid_side);
            }

            // 3D-5: 5DoF 结果已就绪

            // -------------------- 保护：v_max 非零检查 --------------------
            if (v_max.norm() < 1e-6f)
            {
                RCLCPP_WARN(get_logger(), "Class %d v_max is zero, skip Step4C/4D", obj.classId);
                continue;
            }

            // -------------------- Step 4C: 构造 R_init（向量到向量旋转） --------------------
            Eigen::Vector3d cad_axis(0, 0, 1);
            Eigen::Vector3d obj_axis(v_max.cast<double>().normalized());

            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(cad_axis, obj_axis);
            Eigen::Matrix3d R_init = q.toRotationMatrix();

            Eigen::Vector3d test = R_init * cad_axis;
            std::cout << "R_init * [0,0,1] = [" << test.transpose() << "]" << std::endl;
            std::cout << "v_max (normalized) = [" << obj_axis.transpose() << "]" << std::endl;
            std::cout << "Difference norm: " << (test - obj_axis).norm() << std::endl;

            RCLCPP_INFO(get_logger(),
                "Class %d Step4C: R_init constructed (yaw留给ICP)",
                obj.classId);

            // -------------------- Step 4D: 拼接 T_init（CAD → 相机） --------------------
            Eigen::Vector3d t_init = center.cast<double>();
            Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
            T_init.block<3, 3>(0, 0) = R_init;
            T_init.block<3, 1>(0, 3) = t_init;

            RCLCPP_INFO(get_logger(),
                "Class %d Step4D: T_init constructed, t=[%.3f,%.3f,%.3f]",
                obj.classId, t_init.x(), t_init.y(), t_init.z());

            // -------------------- Step 6: point-to-plane ICP (multi-res + trim) --------------------
            bool icp_success = false;
            float fit1 = 0.f, fit2 = 0.f;

            // 初值（CAD->cam）
            Eigen::Matrix4f T_level1 = T_init.cast<float>();
            Eigen::Matrix4f T_level2 = T_init.cast<float>();

            if (cad_cloud_ && !cad_cloud_->empty())
            {
                // Level1：4mm, 10mm, iter15
                {
                    Eigen::Matrix4f T_out;
                    float fitness = 0.f;
                    const bool ok = icpPointToPlaneOneLevel(
                        cad_cloud_, depth_cloud,     // source CAD, target obs(cam)
                        T_level1,                    // 初值 CAD->cam
                        0.004f,                      // voxel 4mm
                        0.010f,                      // max corr 10mm
                        15,                          // iter
                        0.012f,                      // normal radius 12mm (>= ~3*leaf)
                        0.95f,                       // trim_ratio（鲁棒 trim 比例）
                        T_out, fitness);

                    if (ok) { T_level1 = T_out; fit1 = fitness; }
                    else {
                        RCLCPP_WARN(get_logger(), "Class %d Step6 L1 ICP FAIL, fallback init", obj.classId);
                        T_level1 = T_init.cast<float>();
                        fit1 = 1e9f;
                    }
                }

                // Level2：2mm, 5mm, iter15（用 Level1 输出作为初值）
                {
                    Eigen::Matrix4f T_out;
                    float fitness = 0.f;
                    const bool ok = icpPointToPlaneOneLevel(
                        cad_cloud_, depth_cloud,
                        T_level1,                    // L1 输出作为初值
                        0.002f,                      // voxel 2mm
                        0.005f,                      // max corr 5mm
                        15,
                        0.008f,                      // normal radius 8mm
                        0.95f,                       // trim_ratio（鲁棒 trim 比例）
                        T_out, fitness);

                    if (ok) { T_level2 = T_out; fit2 = fitness; icp_success = true; }
                    else {
                        RCLCPP_WARN(get_logger(), "Class %d Step6 L2 ICP FAIL, use L1", obj.classId);
                        T_level2 = T_level1;
                        fit2 = 1e9f;
                        // 这里可以选择：L2失败但L1成功也算 success（我建议算）
                        icp_success = (fit1 < 1e8f);
                    }
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Class %d CAD cloud not loaded, skip Step6 ICP", obj.classId);
            }

            // ICP 门禁（建议保守一点，避免抖动）
            if (icp_success)
            {
                // 允许 L2 或 L1 的结果
                const Eigen::Matrix4d T_icp = T_level2.cast<double>();

                // 与初值的平移差（防止 ICP 跳到别的物体）
                const Eigen::Vector3d t_init_val = T_init.block<3,1>(0,3);
                const Eigen::Vector3d t_icp_val  = T_icp.block<3,1>(0,3);
                const double dt = (t_icp_val - t_init_val).norm();

                // fitness 门禁（按你物体/点数可能要调，但先给一个能用的）
                const bool pass = (fit2 < 0.003f) && (dt < 0.05);  // 5cm 以内

                if (pass)
                {
                    // ==================== Yaw 可观测性判定：±5° 扰动测试 ====================
                    Eigen::Vector3d obj_axis(v_max.cast<double>().normalized());

                    auto evalYaw = [&](double deg)
                    {
                        // 构造绕 obj_axis 旋转 deg 度的旋转矩阵
                        Eigen::AngleAxisd aa(deg * M_PI / 180.0, obj_axis);
                        Eigen::Matrix4d T_test = T_icp;
                        // 只修改旋转部分，保持平移不变
                        T_test.block<3,3>(0,0) = aa.toRotationMatrix() * T_icp.block<3,3>(0,0);

                        // 运行 ICP 评估 fitness（扰动评估：5次迭代足够）
                        Eigen::Matrix4f T_f = T_test.cast<float>();
                        Eigen::Matrix4f dummy;
                        float fit;
                        icpPointToPlaneOneLevel(
                            cad_cloud_, depth_cloud,
                            T_f,
                            0.002f, 0.005f, 5,  // 迭代次数降至 5
                            0.008f,
                            0.95f,              // trim_ratio
                            dummy, fit);
                        return fit;
                    };

                    const float f0 = fit2;
                    const float f_plus  = evalYaw(+5.0);
                    const float f_minus = evalYaw(-5.0);

                    // 阈值：fitness 变化 > 0.001 认为可观测
                    const float fitness_threshold = 0.001f;
                    const bool yaw_observable =
                        (fabs(f_plus - f0) > fitness_threshold) ||
                        (fabs(f_minus - f0) > fitness_threshold);

                    if (yaw_observable)
                    {
                        // 6DoF：使用 ICP 的完整位姿
                        T_init = T_icp;
                        R_init = T_icp.block<3,3>(0,0);
                        center = t_icp_val.cast<float>();
                        pose_quality_ = PoseQuality::FULL_6DOF;

                        RCLCPP_INFO(get_logger(),
                            "Class %d Step6 ICP OK [FULL_6DoF]: fit1=%.4f fit2=%.4f yaw_test=[%.4f,%.4f,%.4f] dt=%.2fcm t=[%.3f,%.3f,%.3f]",
                            obj.classId, fit1, fit2, f_minus, f0, f_plus, dt * 100.0,
                            center.x(), center.y(), center.z());
                    }
                    else
                    {
                        // 5DoF：使用 ICP 的位置，保留初始 yaw（只更新平移，不更新旋转）
                        center = t_icp_val.cast<float>();
                        // R_init 保持不变（保留 FromTwoVectors 的轴对齐结果，不使用 ICP 的 yaw）
                        pose_quality_ = PoseQuality::DEGRADED_5DOF;

                        RCLCPP_WARN(get_logger(),
                            "Class %d Step6 ICP OK [DEGRADED_5DoF]: fit1=%.4f fit2=%.4f yaw_test=[%.4f,%.4f,%.4f] -> yaw NOT observable, use initial yaw dt=%.2fcm t=[%.3f,%.3f,%.3f]",
                            obj.classId, fit1, fit2, f_minus, f0, f_plus, dt * 100.0,
                            center.x(), center.y(), center.z());
                    }
                }
                else
                {
                    RCLCPP_WARN(get_logger(),
                        "Class %d Step6 ICP REJECT: fit2=%.4f dt=%.2fcm -> fallback T_init",
                        obj.classId, fit2, dt * 100.0);
                    icp_success = false; // 让后面走 fallback 发布
                }
            }

            // -------------------- 保存到成员变量 --------------------
            center_ = center;
            v_max_ = v_max;
            R_init_ = R_init;
            T_init_ = T_init;
            has_valid_pose_ = true;

            // -------------------- 发布 pose（ICP 成功发布 ICP，失败发布 T_init）--------------------
            {
                auto pose_msg = geometry_msgs::msg::PoseStamped();
                pose_msg.header.stamp = now();
                pose_msg.header.frame_id = "camera_color_optical_frame";

                Eigen::Quaterniond q_final(R_init_);
                pose_msg.pose.orientation.x = q_final.x();
                pose_msg.pose.orientation.y = q_final.y();
                pose_msg.pose.orientation.z = q_final.z();
                pose_msg.pose.orientation.w = q_final.w();

                pose_msg.pose.position.x = center_.x();
                pose_msg.pose.position.y = center_.y();
                pose_msg.pose.position.z = center_.z();

                pose_pub_->publish(pose_msg);

                if (icp_success) {
                    RCLCPP_INFO(get_logger(), "Class %d Published ICP pose to /detect/cad_initial_pose", obj.classId);
                } else {
                    RCLCPP_INFO(get_logger(), "Class %d Published T_init pose to /detect/cad_initial_pose", obj.classId);
                }
            }

            // -------------------- 可视化：中心点和中心轴 --------------------
            if (center.z() > 1e-6f)
            {
                const float depth_center = center.z();
                const float arrow_length_3d = std::clamp(0.15f * (depth_center / 1.0f), 0.05f, 0.20f);
                const Eigen::Vector3f arrow_end_3d = center + v_max * arrow_length_3d;

                const int u_center = static_cast<int>(fx_ * center.x() / center.z() + cx_);
                const int v_center = static_cast<int>(fy_ * center.y() / center.z() + cy_);
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
                    snprintf(axis_text, sizeof(axis_text), "Z: [%.2f,%.2f,%.2f]", v_max.x(), v_max.y(), v_max.z());
                    cv::putText(vis, axis_text, cv::Point(u_center + 20, v_center - 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

                    char pos_text[64];
                    snprintf(pos_text, sizeof(pos_text), "Pos: [%.3f,%.3f,%.3f]", center.x(), center.y(), center.z());
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

void DetectNode::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (!cloud || cloud->empty()) return;

    // -------------------- 1) VoxelGrid 下采样 --------------------
    // 改动3：放宽参数（4mm → 3mm）
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.003f, 0.003f, 0.003f);  // 3mm
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.filter(*tmp);
        cloud = tmp;
        if (cloud->empty()) return;
    }

    // -------------------- 2) 欧式聚类：只保留最大簇 --------------------
    // 改动3：放宽参数（min_cluster_size: 200 → 100），保持 tolerance=10mm（防止夹具点粘连）
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setSearchMethod(tree);

        ec.setClusterTolerance(0.010f);   // 10mm（保持不变，防止夹具点粘连）
        ec.setMinClusterSize(100);         // 100（放宽）
        ec.setMaxClusterSize(200000);
        ec.setInputCloud(cloud);
        ec.extract(clusters);

        if (!clusters.empty()) {
            size_t best_i = 0;
            size_t best_n = clusters[0].indices.size();
            for (size_t i = 1; i < clusters.size(); ++i) {
                if (clusters[i].indices.size() > best_n) {
                    best_n = clusters[i].indices.size();
                    best_i = i;
                }
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr keep(new pcl::PointCloud<pcl::PointXYZ>());
            keep->reserve(best_n);
            for (int idx : clusters[best_i].indices) keep->push_back((*cloud)[idx]);
            cloud = keep;
        }
        if (cloud->empty()) return;
    }

    // -------------------- 3) RadiusOutlierRemoval：最后再清理碎点 --------------------
    // 改动3：略微放宽（radius: 12mm → 10mm, minNeighbors: 5 → 3）
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(cloud);
        ror.setRadiusSearch(0.010f);      // 10mm
        ror.setMinNeighborsInRadius(3);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        ror.filter(*tmp);
        cloud = tmp;
    }
}

} // namespace arm_controller

RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::DetectNode)
