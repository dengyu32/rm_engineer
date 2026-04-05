#include "detect_node/detect_node.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace arm_controller
{

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectNode::voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float leaf) const
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

pcl::PointCloud<pcl::PointNormal>::Ptr DetectNode::estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float radius) const
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

    if (!normals || normals->size() != in->size()) {
        return pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    }

    auto out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    out->reserve(in->size());

    for (size_t i = 0; i < in->size(); ++i) {
        const float px = (*in)[i].x;
        const float py = (*in)[i].y;
        const float pz = (*in)[i].z;
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
            continue;
        }
        const float nx = (*normals)[i].normal_x;
        const float ny = (*normals)[i].normal_y;
        const float nz = (*normals)[i].normal_z;
        if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz)) {
            continue;
        }

        pcl::PointNormal pn;
        pn.x = px; pn.y = py; pn.z = pz;
        pn.normal_x = nx; pn.normal_y = ny; pn.normal_z = nz;
        out->push_back(pn);
    }
    return out;
}

pcl::PointCloud<pcl::Normal>::Ptr DetectNode::estimateNormalsOnly(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float radius) const
{
    if (!in || in->empty() || radius < 1e-6f) {
        return pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    }

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(in);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*normals);
    return normals;
}

bool DetectNode::fitCylinderAxis(
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
    float& radius_out) const
{
    axis_out = Eigen::Vector3f::UnitZ();
    point_on_axis_out = Eigen::Vector3f::Zero();
    inliers_cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
    radius_out = 0.0f;

    if (!obs_xyz || obs_xyz->empty()) return false;

    auto obs_ds = voxelDownsample(obs_xyz, voxel_leaf);
    if (!obs_ds || obs_ds->empty()) return false;

    std::vector<int> idx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_clean(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::removeNaNFromPointCloud(*obs_ds, *obs_clean, idx);
    if (!obs_clean || obs_clean->size() < 200) return false;

    auto normals = estimateNormalsOnly(obs_clean, normal_radius);
    if (!normals || normals->size() != obs_clean->size()) return false;

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(max_iter);
    seg.setDistanceThreshold(dist_thresh);
    if (radius_min > 1e-6f && radius_max > radius_min) {
        seg.setRadiusLimits(radius_min, radius_max);
    }
    seg.setInputCloud(obs_clean);
    seg.setInputNormals(normals);

    pcl::PointIndices inliers;
    pcl::ModelCoefficients coeff;
    seg.segment(inliers, coeff);

    if (inliers.indices.size() < 150) return false;
    if (coeff.values.size() < 7) return false;

    const float x0 = coeff.values[0];
    const float y0 = coeff.values[1];
    const float z0 = coeff.values[2];
    const float ax = coeff.values[3];
    const float ay = coeff.values[4];
    const float az = coeff.values[5];
    const float r = coeff.values[6];

    Eigen::Vector3f axis(ax, ay, az);
    const float axis_norm = axis.norm();
    if (axis_norm < 1e-6f) return false;
    axis /= axis_norm;

    axis_out = axis;
    point_on_axis_out = Eigen::Vector3f(x0, y0, z0);
    inliers_cloud_out->reserve(inliers.indices.size());
    for (int idx_in : inliers.indices) {
        inliers_cloud_out->push_back((*obs_clean)[idx_in]);
    }
    radius_out = r;
    return true;
}

bool DetectNode::icpPointToPlaneOneLevel(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cad_xyz,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obs_xyz,
    const Eigen::Matrix4f& init_guess,
    float voxel_leaf,
    float max_corr_dist,
    int max_iter,
    float normal_radius,
    float trim_ratio,
    Eigen::Matrix4f& T_out,
    float& fitness_out) const
{
    (void)trim_ratio;
    fitness_out = 1e9f;
    T_out = init_guess;

    if (!cad_xyz || cad_xyz->empty() || !obs_xyz || obs_xyz->empty()) return false;

    auto cad_ds = voxelDownsample(cad_xyz, voxel_leaf);
    auto obs_ds = voxelDownsample(obs_xyz, voxel_leaf);
    if (!cad_ds || cad_ds->empty() || !obs_ds || obs_ds->empty()) return false;

    std::vector<int> idx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_clean(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_clean(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::removeNaNFromPointCloud(*cad_ds, *cad_clean, idx);
    pcl::removeNaNFromPointCloud(*obs_ds, *obs_clean, idx);

    if (cad_clean->size() < 80 || obs_clean->size() < 80) return false;

    auto cad_pn = estimateNormals(cad_clean, normal_radius);
    auto obs_pn = estimateNormals(obs_clean, normal_radius);
    if (!cad_pn || cad_pn->empty() || !obs_pn || obs_pn->empty()) return false;

    auto sanitize = [](const pcl::PointCloud<pcl::PointNormal>::Ptr& in_cloud)
        -> pcl::PointCloud<pcl::PointNormal>::Ptr
    {
        auto out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        out->reserve(in_cloud->size());
        for (const auto& p : in_cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            if (!std::isfinite(p.normal_x) || !std::isfinite(p.normal_y) || !std::isfinite(p.normal_z)) continue;
            out->push_back(p);
        }
        return out;
    };

    cad_pn = sanitize(cad_pn);
    obs_pn = sanitize(obs_pn);
    if (cad_pn->size() < 50 || obs_pn->size() < 50) return false;

    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(cad_pn);
    icp.setInputTarget(obs_pn);

    icp.setMaximumIterations(max_iter);
    icp.setMaxCorrespondenceDistance(max_corr_dist);
    icp.setTransformationEpsilon(1e-4f);
    icp.setEuclideanFitnessEpsilon(1e-4f);

    icp.setTransformationEstimation(
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr(
            new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>()
        )
    );

    pcl::PointCloud<pcl::PointNormal> aligned;
    icp.align(aligned, init_guess);

    if (!icp.hasConverged()) return false;

    T_out = icp.getFinalTransformation();
    fitness_out = static_cast<float>(icp.getFitnessScore());
    return true;
}

bool DetectNode::estimatePoseAndPublish(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& depth_cloud,
    const cv::Mat& mask,
    int class_id,
    const rclcpp::Time& stamp,
    bool& icp_success)
{
    (void)stamp;
    icp_success = false;

    double axis_conf = 0.0;

    if (!depth_cloud || depth_cloud->empty()) return false;

    // ==================== Cylinder RANSAC (先拟合圆柱) ====================
    bool cyl_ok = false;
    Eigen::Vector3f cyl_axis = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f cyl_point = Eigen::Vector3f::Zero();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cyl_inliers;
    float cyl_radius = 0.0f;
    Eigen::Vector3f cyl_center = Eigen::Vector3f::Zero();
    bool cyl_center_valid = false;
    {
        float radius_min = 0.0f;
        float radius_max = 0.0f;
        if (cad_radius_valid_ && cyl_use_radius_limits_) {
            const float margin = std::max(static_cast<float>(cyl_radius_margin_min_),
                                          static_cast<float>(cyl_radius_margin_std_mult_ * cad_radius_std_));
            radius_min = std::max(0.001f, static_cast<float>(cad_radius_mean_ - margin));
            radius_max = static_cast<float>(cad_radius_mean_ + margin);
        }

        cyl_ok = fitCylinderAxis(
            depth_cloud,
            static_cast<float>(cyl_voxel_leaf_),
            static_cast<float>(cyl_normal_radius_),
            cyl_max_iter_,
            static_cast<float>(cyl_dist_thresh_),
            radius_min,
            radius_max,
            cyl_axis,
            cyl_point,
            cyl_inliers,
            cyl_radius);

        if (cyl_ok && cyl_inliers && cyl_inliers->size() >= 150) {
            cyl_axis.normalize();
            std::vector<float> t_vals;
            t_vals.reserve(cyl_inliers->size());
            for (const auto& pt : cyl_inliers->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                const float t = (p - cyl_point).dot(cyl_axis);
                t_vals.push_back(t);
            }
            if (t_vals.size() >= 50) {
                std::sort(t_vals.begin(), t_vals.end());
                const size_t n = t_vals.size();
                const size_t lo = static_cast<size_t>(0.15f * n);
                const size_t hi = n - lo - 1;
                if (lo < hi) {
                    const size_t mid = (lo + hi) / 2;
                    cyl_center = cyl_point + t_vals[mid] * cyl_axis;
                } else {
                    cyl_center = cyl_point;
                }
            } else {
                cyl_center = cyl_point;
            }
            const float t_proj = (cyl_center - cyl_point).dot(cyl_axis);
            cyl_center = cyl_point + t_proj * cyl_axis;
            cyl_center_valid = true;
        }
    }

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
        RCLCPP_WARN(get_logger(), "Class %d PCA eigen decomposition failed", class_id);
        return false;
    }

    const Eigen::Vector3f eigenvalues = eigensolver.eigenvalues();
    const Eigen::Matrix3f eigenvectors = eigensolver.eigenvectors();

    Eigen::Vector3f v_min = eigenvectors.col(0);
    Eigen::Vector3f v_max = eigenvectors.col(2);

    v_max.normalize();
    v_min.normalize();

    // 仅保留“帽端=+axis”规则，移除密度投票与翻转抑制
    axis_conf = 0.0;

    RCLCPP_INFO(get_logger(),
        "Class %d PCA: centroid=[%.3f,%.3f,%.3f] eigenvalues=[%.3f,%.3f,%.3f] v_max=[%.3f,%.3f,%.3f] v_min=[%.3f,%.3f,%.3f] conf=%.3f",
        class_id,
        centroid.x(), centroid.y(), centroid.z(),
        eigenvalues[0], eigenvalues[1], eigenvalues[2],
        v_max.x(), v_max.y(), v_max.z(),
        v_min.x(), v_min.y(), v_min.z(),
        axis_conf);

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr side_points;
    std::string side_source = "hist";

    auto buildSidePoints = [&](float r_center, float delta) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
        out->reserve(depth_cloud->size());
        const float r_lo_local = r_center - delta;
        const float r_hi_local = r_center + delta;
        for (size_t i = 0; i < depth_cloud->size(); ++i) {
            if (radii[i] >= r_lo_local && radii[i] <= r_hi_local) {
                out->push_back(depth_cloud->points[i]);
            }
        }
        return out;
    };

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

        side_points = buildSidePoints(r_mode, delta_r);

        if (cad_radius_valid_) {
            const float cad_r = static_cast<float>(cad_radius_mean_);
            const float cad_delta = std::max(0.003f,
                std::min(0.015f, static_cast<float>(3.0 * cad_radius_std_)));
            auto cand = buildSidePoints(cad_r, cad_delta);
            if (cand && cand->size() > side_points->size()) {
                side_points = cand;
                side_source = "cad";
            }
        }

        if (side_points->size() < 80) {
            std::vector<float> r_tmp = radii;
            const float r_q75 = quantileInplace(r_tmp, 0.75f);
            r_tmp = radii;
            const float r_q90 = quantileInplace(r_tmp, 0.90f);
            const float r_target = 0.5f * (r_q75 + r_q90);
            const float delta_q = std::max(0.003f, 0.12f * r_target);
            auto cand = buildSidePoints(r_target, delta_q);
            if (cand && cand->size() > side_points->size()) {
                side_points = cand;
                side_source = "quant";
            }
        }

        const float side_ratio = static_cast<float>(side_points->size()) / depth_cloud->size();
        const size_t min_side_n = std::min<size_t>(100, std::max<size_t>(50, depth_cloud->size() / 12));
        const bool use_refined = (side_ratio >= 0.12f && side_points->size() >= min_side_n);

        RCLCPP_INFO(get_logger(),
            "Class %d Step3B: r_mode=%.3f ±%.3f [%d in peak] side_n=%zu ratio=%.2f src=%s %s",
            class_id, r_mode, delta_r, max_count,
            side_points->size(), side_ratio,
            side_source.c_str(),
            use_refined ? "REFINE" : "FALLBACK");

        if (use_refined) {
            fallback_count_ = 0;
        } else {
            fallback_count_++;
            if (fallback_count_ >= 5) {
                has_valid_pose_ = false;
                alt_axis_count_ = 0;
                RCLCPP_WARN(get_logger(),
                    "Class %d fallback x%d, reset has_valid_pose_ to allow re-lock",
                    class_id, fallback_count_);
            }
        }

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

                float dot = refined_axis.dot(v_max);
                if (dot < 0.0f) {
                    refined_axis = -refined_axis;
                    dot = -dot;
                }
                if (dot < 0.7f) {
                    RCLCPP_WARN(get_logger(),
                        "Class %d refined_axis deviates (dot=%.2f), reject refine", class_id, dot);
                } else {
                    RCLCPP_INFO(get_logger(),
                        "Class %d refined_axis: refined=[%.3f,%.3f,%.3f] centroid=[%.3f,%.3f,%.3f]",
                        class_id,
                        refined_axis.x(), refined_axis.y(), refined_axis.z(),
                        centroid_refined.x(), centroid_refined.y(), centroid_refined.z());
                    v_max = refined_axis;
                }
            }
        }
    }

    Eigen::Vector3f p0 = centroid;
    if (has_valid_pose_) p0 = center_;

    const size_t min_side_n = std::min<size_t>(100, std::max<size_t>(50, depth_cloud->size() / 12));
    const bool has_valid_side = (side_points && !side_points->empty() && side_points->size() >= min_side_n);
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
        RCLCPP_WARN(get_logger(), "Class %d too few points (%zu) for trimmed median, use centroid", class_id, t_values.size());
    } else {
        std::sort(t_values.begin(), t_values.end());

        const size_t n = t_values.size();
        const size_t qlo = static_cast<size_t>(0.10f * (n - 1));
        const size_t qhi = static_cast<size_t>(0.90f * (n - 1));
        const float t_min_obs = t_values[qlo];
        const float t_max_obs = t_values[qhi];
        const float obs_span = t_max_obs - t_min_obs;

        const size_t lo = static_cast<size_t>(trim_ratio * n);
        const size_t hi = n - lo - 1;

        if (lo >= hi) {
            RCLCPP_WARN(get_logger(), "Class %d trimmed range invalid, use median", class_id);
            const float t_center = t_values[n / 2];
            center = p0 + t_center * v_max;
        } else {
            const size_t mid = (lo + hi) / 2;
            const float t_center = t_values[mid];
            center = p0 + t_center * v_max;
        }

        const float partial_ratio = 0.80f;
        if (cad_axis_valid_ && obs_span > 1e-6f && obs_span < partial_ratio * static_cast<float>(cad_axis_len_)) {
            const float half_len = 0.5f * static_cast<float>(cad_axis_len_);
            const Eigen::Vector3f center_a = p0 + (t_min_obs + half_len) * v_max;
            const Eigen::Vector3f center_b = p0 + (t_max_obs - half_len) * v_max;

            const Eigen::Vector3f ref = has_valid_pose_ ? center_ : centroid;
            const float da = (center_a - ref).norm();
            const float db = (center_b - ref).norm();
            center = (da <= db) ? center_a : center_b;

            RCLCPP_INFO(get_logger(),
                "Class %d Step3D partial: obs_span=%.3f < %.3f*cad_len(%.3f), "
                "center=[%.3f,%.3f,%.3f] choose=%s",
                class_id, obs_span, partial_ratio, static_cast<float>(cad_axis_len_),
                center.x(), center.y(), center.z(), (da <= db) ? "min-side" : "max-side");
        }

        RCLCPP_INFO(get_logger(),
            "Class %d Step3D: t_range=[%.3f,%.3f] t_center=%.3f center=[%.3f,%.3f,%.3f] use_side=%d",
            class_id, t_values[lo], t_values[hi], t_values[(lo + hi) / 2],
            center.x(), center.y(), center.z(), has_valid_side);
    }

    // If we have enough side points, fit circle on cross-section plane to lock axis position
    bool axis_point_valid = false;
    Eigen::Vector3f axis_point_cf = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis_dir_cf = Eigen::Vector3f::Zero();
    if (has_valid_side) {
        Eigen::Vector3f v = v_max.normalized();
        Eigen::Vector3f tmp = (std::abs(v.z()) < 0.9f) ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
        Eigen::Vector3f u = v.cross(tmp).normalized();
        Eigen::Vector3f w = v.cross(u).normalized();

        const size_t n_side = side_points->size();
        Eigen::MatrixXf A(static_cast<int>(n_side), 3);
        Eigen::VectorXf b(static_cast<int>(n_side));
        int idx = 0;
        for (const auto& pt : side_points->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            const float x = p.dot(u);
            const float y = p.dot(w);
            A(idx, 0) = x;
            A(idx, 1) = y;
            A(idx, 2) = 1.0f;
            b(idx) = -(x * x + y * y);
            idx++;
        }

        Eigen::Vector3f sol = A.colPivHouseholderQr().solve(b);
        const float a = sol(0);
        const float bb = sol(1);
        const float c = sol(2);
        const float cx = -0.5f * a;
        const float cy = -0.5f * bb;
        const float r2 = cx * cx + cy * cy - c;

        if (std::isfinite(cx) && std::isfinite(cy) && r2 > 1e-6f) {
            const float r_fit = std::sqrt(r2);
            if (!cad_radius_valid_ || std::abs(r_fit - static_cast<float>(cad_radius_mean_)) < 0.02f) {
                const Eigen::Vector3f axis_point = cx * u + cy * w;
                axis_point_valid = true;
                axis_point_cf = axis_point;
                axis_dir_cf = v;

                std::vector<float> t_vals;
                t_vals.reserve(n_side);
                for (const auto& pt : side_points->points) {
                    Eigen::Vector3f p(pt.x, pt.y, pt.z);
                    t_vals.push_back((p - axis_point).dot(v));
                }
                if (t_vals.size() >= 50) {
                    std::sort(t_vals.begin(), t_vals.end());
                    const size_t n = t_vals.size();
                    const size_t lo = static_cast<size_t>(0.15f * n);
                    const size_t hi = n - lo - 1;
                    if (lo < hi) {
                        const size_t mid = (lo + hi) / 2;
                        center = axis_point + t_vals[mid] * v;
                    } else {
                        center = axis_point;
                    }
                } else {
                    center = axis_point;
                }

                RCLCPP_INFO(get_logger(),
                    "Class %d Step3C circle-fit: r=%.3f axis_point=[%.3f,%.3f,%.3f] center=[%.3f,%.3f,%.3f]",
                    class_id, r_fit,
                    axis_point.x(), axis_point.y(), axis_point.z(),
                    center.x(), center.y(), center.z());
            } else {
                RCLCPP_WARN(get_logger(),
                    "Class %d Step3C circle-fit rejected: r=%.3f cad=%.3f",
                    class_id, r_fit, static_cast<float>(cad_radius_mean_));
            }
        }
    }

    // ==================== Cylinder RANSAC: override axis/center ====================
    if (cyl_ok && cyl_inliers && cyl_inliers->size() >= 150 && cyl_center_valid) {
        cyl_axis.normalize();
        center = cyl_center;
        v_max = cyl_axis;
        RCLCPP_INFO(get_logger(),
            "Class %d Cylinder OK: r=%.3f inliers=%zu axis=[%.3f,%.3f,%.3f] center=[%.3f,%.3f,%.3f]",
            class_id, cyl_radius, cyl_inliers->size(),
            v_max.x(), v_max.y(), v_max.z(),
            center.x(), center.y(), center.z());
    } else {
        RCLCPP_WARN(get_logger(), "Class %d Cylinder FAIL, fallback PCA axis/center", class_id);
    }

    // 帽端约束（最终轴向）：重新按最终 v_max 评估端点半径，确保大端指向 +axis
    {
        size_t n_plus2 = 0;
        size_t n_minus2 = 0;
        std::vector<float> r_plus_vals2;
        std::vector<float> r_minus_vals2;
        float t_min2 = std::numeric_limits<float>::infinity();
        float t_max2 = -std::numeric_limits<float>::infinity();
        for (const auto& pt : depth_cloud->points) {
            const Eigen::Vector3f d(pt.x - centroid.x(), pt.y - centroid.y(), pt.z - centroid.z());
            const float t = d.dot(v_max);
            if (t < t_min2) t_min2 = t;
            if (t > t_max2) t_max2 = t;
        }
        const float span2 = t_max2 - t_min2;
        // 取轴向上端 20% 和下端 20% 的点做平均半径对比
        const float t_high = t_min2 + 0.80f * span2;
        const float t_low = t_min2 + 0.20f * span2;
        if (std::isfinite(span2) && span2 > 1e-6f) {
            for (const auto& pt : depth_cloud->points) {
                const Eigen::Vector3f d(pt.x - centroid.x(), pt.y - centroid.y(), pt.z - centroid.z());
                const float t = d.dot(v_max);
                if (t >= t_high) {
                    const float r = (d - t * v_max).norm();
                    r_plus_vals2.push_back(r);
                    n_plus2++;
                } else if (t <= t_low) {
                    const float r = (d - t * v_max).norm();
                    r_minus_vals2.push_back(r);
                    n_minus2++;
                }
            }
        }
        const size_t min_end_n = std::max<size_t>(50, depth_cloud->size() / 20); // 至少 5% 或 50 点
        if (n_plus2 >= min_end_n && n_minus2 >= min_end_n &&
            r_plus_vals2.size() >= min_end_n && r_minus_vals2.size() >= min_end_n) {
            double r_plus_sum2 = 0.0;
            double r_minus_sum2 = 0.0;
            for (float r : r_plus_vals2) r_plus_sum2 += r;
            for (float r : r_minus_vals2) r_minus_sum2 += r;
            const double r_plus_mean2 = r_plus_sum2 / static_cast<double>(r_plus_vals2.size());
            const double r_minus_mean2 = r_minus_sum2 / static_cast<double>(r_minus_vals2.size());
            const Eigen::Vector3f v_candidate = (r_plus_mean2 < r_minus_mean2) ? -v_max : v_max;
            const int confirm_frames = 4;
            if (has_valid_pose_ && v_max_.norm() > 1e-6f) {
                const float dot_prev = v_candidate.dot(v_max_);
                if (dot_prev < 0.0f) {
                    if (alt_axis_count_ == 0 || alt_axis_.dot(v_candidate) < 0.95f) {
                        alt_axis_ = v_candidate;
                        alt_axis_count_ = 1;
                    } else {
                        alt_axis_count_++;
                    }
                    if (alt_axis_count_ < confirm_frames) {
                        v_max = v_max_;
                    } else {
                        v_max = v_candidate;
                        alt_axis_count_ = 0;
                    }
                } else {
                    alt_axis_count_ = 0;
                    v_max = v_candidate;
                }
            } else {
                v_max = v_candidate;
                alt_axis_count_ = 0;
            }
        }
    }

    auto projectedOnMask = [&](const Eigen::Vector3f& c) -> bool
    {
        if (mask.empty()) return true;
        if (c.z() < 1e-6f || fx_ < 1e-6 || fy_ < 1e-6) return false;
        const int u = static_cast<int>(std::lround(fx_ * c.x() / c.z() + cx_));
        const int v = static_cast<int>(std::lround(fy_ * c.y() / c.z() + cy_));
        if (u < 0 || v < 0 || u >= mask.cols || v >= mask.rows) return false;
        const int r = 2;
        const int u0 = std::max(0, u - r);
        const int v0 = std::max(0, v - r);
        const int u1 = std::min(mask.cols - 1, u + r);
        const int v1 = std::min(mask.rows - 1, v + r);
        for (int yy = v0; yy <= v1; ++yy) {
            const uchar* row = mask.ptr<uchar>(yy);
            for (int xx = u0; xx <= u1; ++xx) {
                if (row[xx] != 0) return true;
            }
        }
        return false;
    };

    if (!projectedOnMask(center))
    {
        const Eigen::Vector3f fallback = centroid;
        RCLCPP_WARN(get_logger(),
            "Class %d center projects outside mask, fallback to centroid [%.3f,%.3f,%.3f]",
            class_id, fallback.x(), fallback.y(), fallback.z());
        center = fallback;
    }

    if (v_max.norm() < 1e-6f)
    {
        RCLCPP_WARN(get_logger(), "Class %d v_max is zero, skip Step4C/4D", class_id);
        return false;
    }

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
        class_id);

    Eigen::Vector3d t_init = center.cast<double>();
    Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
    T_init.block<3, 3>(0, 0) = R_init;
    T_init.block<3, 1>(0, 3) = t_init;

    RCLCPP_INFO(get_logger(),
        "Class %d Step4D: T_init constructed, t=[%.3f,%.3f,%.3f]",
        class_id, t_init.x(), t_init.y(), t_init.z());

    bool icp_ok = false;
    float fit1 = 0.f, fit2 = 0.f;

    Eigen::Matrix4f T_level1 = T_init.cast<float>();
    Eigen::Matrix4f T_level2 = T_init.cast<float>();

    if (cad_cloud_ && !cad_cloud_->empty())
    {
        {
            Eigen::Matrix4f T_out;
            float fitness = 0.f;
            const bool ok = icpPointToPlaneOneLevel(
                cad_cloud_, depth_cloud,
                T_level1,
                0.004f,
                0.010f,
                15,
                0.012f,
                0.95f,
                T_out, fitness);

            if (ok) { T_level1 = T_out; fit1 = fitness; }
            else {
                RCLCPP_WARN(get_logger(), "Class %d Step6 L1 ICP FAIL, fallback init", class_id);
                T_level1 = T_init.cast<float>();
                fit1 = 1e9f;
            }
        }

        {
            Eigen::Matrix4f T_out;
            float fitness = 0.f;
            const bool ok = icpPointToPlaneOneLevel(
                cad_cloud_, depth_cloud,
                T_level1,
                0.002f,
                0.005f,
                15,
                0.008f,
                0.95f,
                T_out, fitness);

            if (ok) { T_level2 = T_out; fit2 = fitness; icp_ok = true; }
            else {
                RCLCPP_WARN(get_logger(), "Class %d Step6 L2 ICP FAIL, use L1", class_id);
                T_level2 = T_level1;
                fit2 = 1e9f;
                icp_ok = (fit1 < 1e8f);
            }
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Class %d CAD cloud not loaded, skip Step6 ICP", class_id);
    }

    if (icp_ok)
    {
        const Eigen::Matrix4d T_icp = T_level2.cast<double>();

        const Eigen::Vector3d t_init_val = T_init.block<3,1>(0,3);
        const Eigen::Vector3d t_icp_val  = T_icp.block<3,1>(0,3);
        const double dt = (t_icp_val - t_init_val).norm();

        Eigen::Vector3d axis_init = v_max.cast<double>().normalized();
        Eigen::Vector3d axis_icp = T_icp.block<3,3>(0,0).col(2).normalized();
        if (axis_init.dot(axis_icp) < 0.0) axis_icp = -axis_icp;
        const double dt_axis = std::acos(std::clamp(axis_init.dot(axis_icp), -1.0, 1.0));

        // 改动: 放宽 ICP 验证条件，降低对 fitness 和 dt_axis 的要求
        // yaw 由 pose_from_axis_node 计算，这里只校验位置和主轴
        const bool pass = (fit2 < 0.005f) && (dt < 0.08) && (dt_axis < 0.8);  // 放宽阈值

        if (pass)
        {
            // 只使用 ICP 的平移结果，旋转保持 PCA 的结果
            center = t_icp_val.cast<float>();
            // R_init 保持不变（来自 PCA），yaw 由 pose_from_axis_node 计算
            pose_quality_ = PoseQuality::DEGRADED_5DOF;  // 始终使用 5DOF

            RCLCPP_INFO(get_logger(),
                "Class %d Step6 ICP OK [5DOF]: fit1=%.4f fit2=%.4f dt=%.2fcm dt_axis=%.1f° t=[%.3f,%.3f,%.3f]",
                class_id, fit1, fit2, dt * 100.0, dt_axis * 180.0 / M_PI,
                center.x(), center.y(), center.z());
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                "Class %d Step6 ICP REJECT: fit2=%.4f dt=%.2fcm dt_axis=%.1f° -> fallback T_init",
                class_id, fit2, dt * 100.0, dt_axis * 180.0 / M_PI);
            icp_ok = false;
        }
    }

    // Smooth center/axis to reduce jitter
    {
        // 改动: 增大 alpha 以减少滞后 (0.25 -> 0.65)
        const float alpha_center = 0.65f;          // new measurement weight
        const float alpha_axis = static_cast<float>(axis_smooth_alpha_);
        const float max_jump = 0.06f;        // meters, clamp sudden jumps

        if (has_valid_pose_) {
            if (cyl_ok) {
                const float t_new = (center - cyl_point).dot(cyl_axis);
                const float t_prev = (center_ - cyl_point).dot(cyl_axis);
                float t_smooth = alpha_center * t_new + (1.0f - alpha_center) * t_prev;
                float dt = t_smooth - t_prev;
                if (std::abs(dt) > max_jump) {
                    dt = (dt > 0.0f) ? max_jump : -max_jump;
                    t_smooth = t_prev + dt;
                }
                center = cyl_point + t_smooth * cyl_axis;
                center_ = center;
            } else if (axis_point_valid && axis_dir_cf.norm() > 1e-6f) {
                const Eigen::Vector3f v = axis_dir_cf.normalized();
                const float t_new = (center - axis_point_cf).dot(v);
                const float t_prev = (center_ - axis_point_cf).dot(v);
                float t_smooth = alpha_center * t_new + (1.0f - alpha_center) * t_prev;
                float dt = t_smooth - t_prev;
                if (std::abs(dt) > max_jump) {
                    dt = (dt > 0.0f) ? max_jump : -max_jump;
                    t_smooth = t_prev + dt;
                }
                center = axis_point_cf + t_smooth * v;
                center_ = center;
            } else {
                Eigen::Vector3f delta = center - center_;
                const float dist = delta.norm();
                if (dist > max_jump && dist > 1e-6f) {
                    delta = delta / dist * max_jump;
                    center = center_ + delta;
                }
                center_ = alpha_center * center + (1.0f - alpha_center) * center_;
            }

            // Axis smoothing (no lock, no confidence gating)
            if (v_max_.norm() > 1e-6f && v_max.norm() > 1e-6f) {
                if (v_max.dot(v_max_) < 0.0f) {
                    // fast flip to avoid long correction time
                    v_max_ = v_max;
                } else {
                    Eigen::Vector3f v_smooth = alpha_axis * v_max + (1.0f - alpha_axis) * v_max_;
                    if (v_smooth.norm() > 1e-6f) {
                        v_max_ = v_smooth.normalized();
                    } else {
                        v_max_ = v_max;
                    }
                }
            } else {
                v_max_ = v_max;
            }
        } else {
            if (cyl_ok) {
                const float t_new = (center - cyl_point).dot(cyl_axis);
                center_ = cyl_point + t_new * cyl_axis;
            } else {
                center_ = center;
            }
            v_max_ = v_max;
            alt_axis_count_ = 0;
        }
    }
    R_init_ = R_init;
    T_init_ = T_init;
    has_valid_pose_ = true;

    {
        // 改动: 发布 5DOF 位姿 (位置 + Z轴方向)，yaw 由 pose_from_axis_node 计算
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "camera_color_optical_frame";

        // 使用 PCA 结果构建旋转矩阵（只有 Z 轴方向确定，yaw 未定）
        Eigen::Quaterniond q_final(R_init_);
        pose_msg.pose.orientation.x = q_final.x();
        pose_msg.pose.orientation.y = q_final.y();
        pose_msg.pose.orientation.z = q_final.z();
        pose_msg.pose.orientation.w = q_final.w();

        pose_msg.pose.position.x = center_.x();
        pose_msg.pose.position.y = center_.y();
        pose_msg.pose.position.z = center_.z();

        pose_pub_->publish(pose_msg);

        // 发布中心点
        auto center_msg = geometry_msgs::msg::PointStamped();
        center_msg.header = pose_msg.header;
        center_msg.point.x = center_.x();
        center_msg.point.y = center_.y();
        center_msg.point.z = center_.z();
        center_pub_->publish(center_msg);

        // 发布主轴方向 (Z轴)
        if (v_max_.norm() > 1e-6f) {
            Eigen::Vector3f v_pub = v_max_;
            if (axis_force_flip_) v_pub = -v_pub;
            auto axis_msg = geometry_msgs::msg::Vector3Stamped();
            axis_msg.header = pose_msg.header;
            axis_msg.vector.x = v_pub.x();
            axis_msg.vector.y = v_pub.y();
            axis_msg.vector.z = v_pub.z();
            axis_pub_->publish(axis_msg);
        }

        if (icp_ok) {
            RCLCPP_INFO(get_logger(), "Class %d Published 5DOF pose (ICP refined position) to /detect/cad_initial_pose", class_id);
        } else {
            RCLCPP_INFO(get_logger(), "Class %d Published 5DOF pose (PCA only) to /detect/cad_initial_pose", class_id);
        }
    }

    icp_success = icp_ok;
    return true;
}

} // namespace arm_controller
