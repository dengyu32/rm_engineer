#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace task_orchestrator {

// ============================================================================
//  TaskId / TaskName
// ----------------------------------------------------------------------------
//  - 任务枚举定义（X-MACRO）
//  - task_name / is_supported_task 统一入口
// ============================================================================

// 任务LIST宏 X-MACRO 
#define TASK_LIST(X) \
    X(IDLE,           0) \
    X(AUTO_INIT,      1) \
    X(AUTO_GRAB,      2) \
    X(AUTO_STORE,     3) \
    X(AUTO_GET,       4) \
    X(FIXED_GRAB,     5) \
    X(TEST_SOLVE,     6) \
    X(TEST_CARTESIAN, 7)

// 任务ID枚举
enum class TaskId : uint8_t {

#define AS_ENUM(name, val) name = val,
    TASK_LIST(AS_ENUM)
#undef AS_ENUM

    COUNT // 自动排在最后
};

// 任务名称数组
inline constexpr std::array<const char*, static_cast<std::size_t>(TaskId::COUNT)> kTaskNames = {

#define AS_STRING(name, val) #name,
    TASK_LIST(AS_STRING)
#undef AS_STRING

};

// 检查任务ID是否有效
inline bool is_supported_task(TaskId id) {
    return static_cast<uint8_t>(id) < static_cast<uint8_t>(TaskId::COUNT);
}

// 获取任务名称
inline const char* task_name(TaskId id) {
    if (is_supported_task(id)) {
        return kTaskNames[static_cast<std::size_t>(id)];
    }
    return "UNKNOWN";
}

} // namespace task_orchestrator

namespace task_orchestrator::preset {

// ============================================================================
//  Preset
// ----------------------------------------------------------------------------
//  - 机器人预置位姿 / 槽位关节
// ============================================================================

// Robot state presets
inline constexpr std::array<float, 6> ZERO{{
    0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
}};

inline constexpr std::array<float, 6> HOME{{
    0.f, 1.042f, -2.618f, 0.f, 0.f, 0.f,
}};

inline constexpr std::array<float, 6> SLOTS[] = {
    {-0.9250f, -0.1396f, 1.9722f, -3.0718f, -1.2741f,  0.7679f},
    { 0.4363f, -0.1047f, 1.9024f,  0.0175f,  1.3265f, -0.9774f},
};

inline constexpr std::array<float, 6> DESCEND_SLOTS[] = {
    {-0.8901f, -0.0175f, 2.0944f, 0.0175f, 1.0821f, -2.3213f},
    { 0.4014f, -0.0524f, 2.1642f, 0.0175f, 1.0123f, -1.0297f},
};

} // namespace task_orchestrator::preset

namespace task_orchestrator::protocol {

// ============================================================================
//  Protocol
// ----------------------------------------------------------------------------
//  - Context keys / Command kinds
// ============================================================================

// Context keys
inline constexpr const char *kVisionPose = "VisionPose";
inline constexpr const char *kVisionVector = "VisionVector";
inline constexpr const char *kSlotId = "SlotID";

// Command kinds
inline constexpr const char *kArmMoveKind = "arm.move";
inline constexpr const char *kGripperKind = "gripper.cmd";
inline constexpr const char *kVisionKind = "vision.detect";
inline constexpr const char *kSlotSelectKind = "slot.select";
inline constexpr const char *kSlotLockKind = "slot.lock";
inline constexpr const char *kSlotUnlockKind = "slot.unlock";

} // namespace task_orchestrator::protocol
