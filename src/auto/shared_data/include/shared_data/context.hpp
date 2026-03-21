#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <variant>

#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace task_step_library {

enum class SharedKey : uint8_t {
  VisionPose = 0,
  VisionVector = 1,
  SelectedSlot = 2,
  _Count = 3,
};

template <SharedKey K>
struct SharedType;

template <>
struct SharedType<SharedKey::VisionPose> {
  using type = engineer_interfaces::msg::Pose;
};

template <>
struct SharedType<SharedKey::VisionVector> {
  using type = geometry_msgs::msg::Vector3;
};

template <>
struct SharedType<SharedKey::SelectedSlot> {
  using type = int;
};

using SharedValue = std::variant<engineer_interfaces::msg::Pose,
                                 geometry_msgs::msg::Vector3,
                                 int>;

struct StepResult {
  struct Update {
    SharedKey key;
    SharedValue value;
  };
  std::vector<Update> updates;

  template <SharedKey K>
  void set(const typename SharedType<K>::type &value) {
    updates.push_back(Update{K, SharedValue{value}});
  }

  void set(SharedKey key, const SharedValue &value) {
    updates.push_back(Update{key, value});
  }
};

class SharedData {
public:
  void clear() { present_.fill(false); }

  template <SharedKey K>
  bool has() const {
    return has(K);
  }

  bool has(SharedKey key) const {
    return present_[toIndex(key)];
  }

  template <SharedKey K>
  bool get(typename SharedType<K>::type &out) const {
    SharedValue value;
    if (!get(K, value)) {
      return false;
    }
    const auto *ptr = std::get_if<typename SharedType<K>::type>(&value);
    if (!ptr) {
      return false;
    }
    out = *ptr;
    return true;
  }

  bool get(SharedKey key, SharedValue &out) const {
    const std::size_t idx = toIndex(key);
    if (!present_[idx]) {
      return false;
    }
    out = values_[idx];
    return true;
  }

  template <SharedKey K>
  bool set(const typename SharedType<K>::type &value) {
    return set(K, SharedValue{value});
  }

  bool set(SharedKey key, const SharedValue &value) {
    if (!valueMatchesKey(key, value)) {
      return false;
    }
    const std::size_t idx = toIndex(key);
    values_[idx] = value;
    present_[idx] = true;
    return true;
  }

private:
  static constexpr std::size_t toIndex(SharedKey key) {
    return static_cast<std::size_t>(key);
  }

  static bool valueMatchesKey(SharedKey key, const SharedValue &value) {
    switch (key) {
    case SharedKey::VisionPose:
      return std::holds_alternative<engineer_interfaces::msg::Pose>(value);
    case SharedKey::VisionVector:
      return std::holds_alternative<geometry_msgs::msg::Vector3>(value);
    case SharedKey::SelectedSlot:
      return std::holds_alternative<int>(value);
    default:
      return false;
    }
  }

  std::array<bool, static_cast<std::size_t>(SharedKey::_Count)> present_{};
  std::array<SharedValue, static_cast<std::size_t>(SharedKey::_Count)> values_{};
};

} // namespace task_step_library
