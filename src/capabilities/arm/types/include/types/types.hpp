#pragma once

#include <string>
#include <vector>

namespace types {

struct Pose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct JointState {
  std::vector<std::string> names;
  std::vector<double> positions;
};

struct TrajectoryPoint {
  std::vector<double> positions;
  std::vector<double> velocities;
  double time_from_start{0.0};
};

struct Trajectory {
  std::vector<std::string> joint_names;
  std::vector<TrajectoryPoint> points;
};

struct SolveResponse {
  Trajectory trajectory;
};

}  // namespace types
