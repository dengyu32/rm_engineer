#include "auto_node/capability_registry.hpp"

#include "step_executor/registry_bridge.hpp"
#include "auto_library/execute_result.hpp"
#include "task_orchestrator/protocol.hpp"

#include "arm_solve_client/arm_solve_client.hpp"
#include "gripper_control_node/gripper_control_node.hpp"
#include "slot_select_node/slot_select_node.hpp"
#include "vision_detect_client/vision_detect_client.hpp"

namespace engineer_auto::auto_node {

namespace armc = engineer_auto::arm_solve_client;
namespace gripc = engineer_auto::gripper_control_node;
namespace slotc = engineer_auto::slot_select_node;
namespace visc = engineer_auto::vision_detect_client;

namespace protocol = task_orchestrator::protocol;

namespace {
inline step_executor::ExecuteResult Fail(const std::string &msg) {
  step_executor::ExecuteResult result{};
  result.status = step_executor::ExecuteStatus::Failed;
  result.error.message = msg;
  result.error.retriable = false;
  return result;
}
}

std::shared_ptr<step_executor::ICapabilityBridge> createAutoCapabilityBridge(rclcpp::Node &node) {
    auto registry = std::make_shared<step_executor::RegistryBridge>();

    // 定义一个通用的绑定辅助 Lambda，进一步压缩代码
    auto bind = [&](const std::string& kind,
                    step_executor::RegistryBridge::HandlerFn run_fn,
                    step_executor::RegistryBridge::CancelFn cancel_fn = {}) {
        registry->registerHandler(kind, std::move(run_fn), std::move(cancel_fn));
    };

    // 1. Arm Solve Client
    {
        auto a = std::make_shared<armc::ArmSolveClient>(node, armc::ArmSolveClientConfig::load(node));
        bind(protocol::kArmMoveKind, 
             [a](const auto& c) { armc::ArmMoveSpec s; std::string e; return a->buildSpec(c, s, e) ? a->execute(s) : Fail(e); },
             [a]() { a->cancel(); });
    }

    // 2. Gripper Node
    {
        auto g = std::make_shared<gripc::GripperControlNode>(node, gripc::GripperPresetConfig::load(node));
        bind(protocol::kGripperKind, [g](const auto& c) { return g->execute(c); }, [g]() { g->cancel(); });
    }

    // 3. Slot Select (一个对象，多个绑定)
    {
        auto s = std::make_shared<slotc::SlotSelectNode>(node, slotc::SlotSelectConfig::load(node));
        bind(protocol::kSlotSelectKind, [s](const auto& c) { return s->executeSelect(c); });
        bind(protocol::kSlotLockKind,   [s](const auto& c) { return s->executeLockUnlock(c, slotc::SlotStrategy::LockSlot); });
        bind(protocol::kSlotUnlockKind, [s](const auto& c) { return s->executeLockUnlock(c, slotc::SlotStrategy::UnlockSlot); });
    }

    // 4. Vision
    {
        auto v = std::make_shared<visc::VisionDetectClient>(node);
        bind(protocol::kVisionKind, [v](const auto& c) { return v->execute(c); });
    }

    return registry;
}

} // namespace engineer_auto::auto_node
