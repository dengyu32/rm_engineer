// 分割封装：运行 YOLO 分割并选择目标用于后续处理。

#include "detect_node/detect_node.h"

namespace arm_controller
{

// 运行 YOLO 分割并返回筛选后的目标。
std::vector<DetectNode::SegObject> DetectNode::runSegmentation(const cv::Mat& color, cv::Mat& vis)
{
    std::vector<DetectNode::SegObject> out;
    std::vector<yolos::seg::Segmentation> results =
        detector_.segment(color, 0.7f, 0.5f);

    if (results.empty()) return out;

    // 单类别模型：只保留 class_id=0，保留所有高置信度候选，后续做追踪选择
    std::vector<yolos::seg::Segmentation> filtered;
    filtered.reserve(results.size());
    for (const auto& obj : results) {
        if (obj.classId != 0) continue;
        if (obj.conf < 0.7f) continue;
        filtered.push_back(obj);
    }

    if (filtered.empty()) return out;

    detector_.drawMasksOnly(vis, filtered, 0.6f);
    out.reserve(filtered.size());

    for (const auto& obj : filtered)
    {
        DetectNode::SegObject item;
        item.class_id = obj.classId;
        item.conf = obj.conf;
        item.mask = obj.mask;
        item.bbox = cv::Rect(obj.box.x, obj.box.y, obj.box.width, obj.box.height);
        out.push_back(std::move(item));

        // 绘制调试标签，确认 class name/id/conf。
        const auto& names = detector_.getClassNames();
        std::string cls = (obj.classId >= 0 && static_cast<size_t>(obj.classId) < names.size())
                              ? names[obj.classId]
                              : "unknown";
        char text[96];
        snprintf(text, sizeof(text), "%s id=%d conf=%.2f", cls.c_str(), obj.classId, obj.conf);
        cv::Point org(item.bbox.x, std::max(15, item.bbox.y - 5));
        cv::putText(vis, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
    return out;
}

bool DetectNode::isBboxAtBorder(
    const cv::Rect& bbox, int img_w, int img_h, int border_margin) const
{
    return (bbox.x < border_margin) ||
           (bbox.y < border_margin) ||
           (bbox.x + bbox.width > img_w - border_margin) ||
           (bbox.y + bbox.height > img_h - border_margin);
}

} // 命名空间 arm_controller
