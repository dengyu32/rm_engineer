#pragma once

#include <cstdint>

namespace engineer_auto::slot_select_node {

enum class SlotStrategy : uint8_t {
  SelectSlotToPut = 0,
  LockSlot = 1,
  SelectSlotToTake = 2,
  UnlockSlot = 3,
};

} // namespace engineer_auto::slot_select_node
