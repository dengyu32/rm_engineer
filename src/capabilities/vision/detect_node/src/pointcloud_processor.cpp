// 点云处理工具：过滤与聚类选择，供 DetectNode 使用。

#include "detect_node/detect_node.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace arm_controller
{

bool DetectNode::buildRawCloudFromMask(
    const cv::Mat& mask,
    const cv::Mat& depth,
    const rclcpp::Time& stamp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud,
    std::vector<float>& z_vals,
    size_t& mask_nz,
    int& zero_depth,
    int& out_of_range,
    float& valid_ratio,
    size_t& sampled_total) const
{
    std::vector<cv::Point> nz;
    cv::findNonZero(mask, nz);
    mask_nz = nz.size();
    if (nz.empty()) return false;

    const size_t max_sample = 6000;
    const size_t sample_count = std::min(nz.size(), max_sample);

    // 使用确定性采样，减少帧间抖动。
    const size_t step = (nz.size() > max_sample)
        ? static_cast<size_t>(std::ceil(static_cast<double>(nz.size()) / max_sample))
        : 1;

    int valid_sampled = 0;
    sampled_total = 0;
    zero_depth = 0;
    out_of_range = 0;

    raw_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    raw_cloud->reserve(sample_count);
    z_vals.clear();
    z_vals.reserve(sample_count);

    for (size_t idx = 0; idx < sample_count; ++idx)
    {
        const size_t i = idx * step;
        if (i >= nz.size()) break;
        const int x = nz[i].x;
        const int y = nz[i].y;

        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) continue;

        float Z = 0.0f;
        if (depth.type() == CV_16UC1)
        {
            const uint16_t d = depth.at<uint16_t>(y, x);
            if (d == 0) { zero_depth++; continue; }
            Z = static_cast<float>(d) * static_cast<float>(depth_scale_);
        }
        else
        {
            const float d = depth.at<float>(y, x);
            if (!std::isfinite(d) || d < 1e-6f) { zero_depth++; continue; }
            Z = d;
        }

        if (Z < 0.1f || Z > 2.0f) { out_of_range++; continue; }

        const float X = (static_cast<float>(x) - static_cast<float>(cx_)) * Z / static_cast<float>(fx_);
        const float Y = (static_cast<float>(y) - static_cast<float>(cy_)) * Z / static_cast<float>(fy_);

        sampled_total++;
        raw_cloud->push_back(pcl::PointXYZ(X, Y, Z));
        z_vals.push_back(Z);
        valid_sampled++;
    }

    if (raw_cloud->empty()) return false;

    valid_ratio = (sampled_total > 0) ? static_cast<float>(valid_sampled) / sampled_total : 0.0f;
    return true;
}

float DetectNode::quantileInplace(std::vector<float>& v, float q) const
{
    if (v.empty()) return 0.0f;
    if (q < 0.0f) q = 0.0f;
    if (q > 1.0f) q = 1.0f;

    const size_t k = static_cast<size_t>(q * (v.size() - 1));
    auto it = v.begin() + k;
    std::nth_element(v.begin(), it, v.end());
    return *it;
}

bool DetectNode::depthBandPass(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud,
    const std::vector<float>& z_vals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& depth_cloud,
    float& z_lo,
    float& z_hi) const
{
    if (!raw_cloud || raw_cloud->empty() || z_vals.empty()) return false;

    std::vector<float> z_tmp = z_vals;
    const float z25 = quantileInplace(z_tmp, 0.25f);

    z_tmp = z_vals;
    const float z75 = quantileInplace(z_tmp, 0.75f);

    z_tmp = z_vals;
    const float z50 = quantileInplace(z_tmp, 0.50f);

    const float margin = std::max(0.03f, 0.03f * z50);
    z_lo = z25 - margin;
    z_hi = z75 + margin;

    const float min_band = 0.02f;
    if (z_hi - z_lo < min_band) {
        z_lo = z50 - 0.03f;
        z_hi = z50 + 0.03f;
    }

    depth_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    depth_cloud->reserve(raw_cloud->size());

    for (const auto& pt : raw_cloud->points) {
        if (pt.z >= z_lo && pt.z <= z_hi) depth_cloud->push_back(pt);
    }

    return !depth_cloud->empty();
}

void DetectNode::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (!cloud || cloud->empty()) return;

    // -------------------- 1) VoxelGrid 下采样 --------------------
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.003f, 0.003f, 0.003f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.filter(*tmp);
        cloud = tmp;
        if (cloud->empty()) return;
    }

    // -------------------- 2) 欧式聚类：中心先验 + 防锁死断路器 --------------------
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setSearchMethod(tree);
        ec.setClusterTolerance(0.010f);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(200000);
        ec.setInputCloud(cloud);
        ec.extract(clusters);

        if (!clusters.empty())
        {
            size_t best_i = 0;
            bool use_prior = has_valid_pose_;
            float best_dist = std::numeric_limits<float>::infinity();

            std::vector<Eigen::Vector3f> centers(clusters.size(), Eigen::Vector3f::Zero());

            for (size_t i = 0; i < clusters.size(); ++i)
            {
                for (int idx : clusters[i].indices)
                {
                    const auto& pt = (*cloud)[idx];
                    centers[i] += Eigen::Vector3f(pt.x, pt.y, pt.z);
                }
                centers[i] /= static_cast<float>(clusters[i].indices.size());
            }

            if (use_prior)
            {
                for (size_t i = 0; i < clusters.size(); ++i)
                {
                    float dist = (centers[i] - center_).norm();
                    if (dist < best_dist)
                    {
                        best_dist = dist;
                        best_i = i;
                    }
                }

                const float max_center_jump = 0.08f;

                if (best_dist > max_center_jump)
                {
                    size_t max_i = 0;
                    for (size_t i = 1; i < clusters.size(); ++i)
                    {
                        if (clusters[i].indices.size() > clusters[max_i].indices.size())
                            max_i = i;
                    }

                    RCLCPP_WARN(get_logger(),
                        "[detect_node][pc] Cluster PRIOR too far (%.1fmm > %.1fmm), fallback to MAX cluster (%zu pts)",
                        best_dist * 1000.f,
                        max_center_jump * 1000.f,
                        clusters[max_i].indices.size());

                    best_i = max_i;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                        "[detect_node][pc] Cluster selection: PRIOR dist=%.1fmm pts=%zu",
                        best_dist * 1000.f,
                        clusters[best_i].indices.size());
                }
            }
            else
            {
                for (size_t i = 1; i < clusters.size(); ++i)
                {
                    if (clusters[i].indices.size() > clusters[best_i].indices.size())
                        best_i = i;
                }

                RCLCPP_INFO(get_logger(),
                    "[detect_node][pc] Cluster selection: MAX cluster pts=%zu",
                    clusters[best_i].indices.size());
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr keep(new pcl::PointCloud<pcl::PointXYZ>());
            keep->reserve(clusters[best_i].indices.size());

            for (int idx : clusters[best_i].indices)
                keep->push_back((*cloud)[idx]);

            cloud = keep;
        }

        if (cloud->empty()) return;
    }

    // -------------------- 3) RadiusOutlierRemoval：最后再清理碎点 --------------------
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(cloud);
        ror.setRadiusSearch(0.010f);
        ror.setMinNeighborsInRadius(3);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        ror.filter(*tmp);
        cloud = tmp;
    }
}

} // 命名空间 arm_controller
