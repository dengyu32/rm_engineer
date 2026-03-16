#pragma once

#include <memory>
#include <optional>
#include <string>

#include "solve_core/types.hpp"
#include "solve_core/moveit_adapter.hpp"

namespace solve_core {

class SolveCore {
public:

  explicit SolveCore(std::shared_ptr<MoveItAdapter> adapter,
                     const SolveCoreConfig &config = SolveCoreConfig{});

  std::optional<SolveResponse> plan(const SolveRequest &req, std::string &err);

private:
  std::shared_ptr<MoveItAdapter> adapter_;
  SolveCoreConfig config_;

  std::optional<SolveResponse> plan_normal(const SolveRequest &req, std::string &err);
  std::optional<SolveResponse> plan_cartesian(const SolveRequest &req, std::string &err);
  std::optional<SolveResponse> plan_joints(const SolveRequest &req, std::string &err);

};

} // namespace solve_core
