#pragma once

#include <array>

namespace task_orchestrator {


// 关节位姿 
inline constexpr std::array<float,6> ZERO{
    0.f, 0.f, 0.f, 0.f, 0.f, 0.f
};

inline constexpr std::array<float,6> HOME{
    0.f, 1.042f, -2.618f, 0.f, 0.f, 0.f
};

/*
测试两种：
关节 + 关节 
关节 + 笛卡尔(7~9cm)
*/

inline constexpr std::array<float,6> SLOT_1{
    -0.9250f, -0.1396f, 1.9722f, -3.0718f, -1.2741f, 0.7679f
};

inline constexpr std::array<float,6> SLOT_1_DESCEND{
    -0.8901f, -0.0175f, 2.0944f, 0.0175f, 1.0821f, -2.3213f
};

inline constexpr std::array<float,6> SLOT_2{
    0.4363f, -0.1047f, 1.9024f, 0.0175f, 1.3265f, -0.9774f
};

inline constexpr std::array<float,6> SLOT_2_DESCEND{
    0.4014, -0.0524, 2.1642, 0.0175, 1.0123, -1.0297
};

// 方向 + 标量距离 [x, y, z, distance]

}