#pragma once

#include <array>

namespace task_orchestrator {

// 关节位姿 
inline constexpr std::array<float,6> ZERO{
    0.f, 0.f, 0.f, 0.f, 0.f, 0.f
};

inline constexpr std::array<float,6> HOME{
    0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f
};

// TO MEASURE
inline constexpr std::array<float,6> SLOT_1{};

inline constexpr std::array<float,6> SLOT_2{};

}