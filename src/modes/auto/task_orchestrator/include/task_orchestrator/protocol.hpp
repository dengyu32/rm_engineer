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
// 可供测试的有 1 3 4 6 7 8 
#define TASK_LIST(X) \
    X(IDLE,           0) \
    X(AUTO_INIT,      1) \
    X(AUTO_GRAB,      2) \
    X(AUTO_STORE,     3) \
    X(AUTO_GET,       4) \
    X(FIXED_GRAB,     5) \
    X(TEST_NOMAL,     6) \
    X(TEST_CARTESIAN, 7) \
    X(TEST_LINE,      8) \
    X(TEST_VISION,    9) 
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
