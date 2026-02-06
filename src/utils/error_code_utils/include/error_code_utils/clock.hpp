#pragma once

#include <cstdint>
#include <chrono>

namespace error_code_utils {

// -----------------------------------------------------------------------------
// Clock Abstraction
// -----------------------------------------------------------------------------

struct IClock {
  virtual int64_t now_ms() const = 0;
  virtual ~IClock() = default;
};

struct SteadyClock : public IClock {
  int64_t now_ms() const override {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  }
};

struct FakeClock : public IClock {
  int64_t t{0};

  int64_t now_ms() const override {
    return t;
  }

  void advance_ms(int64_t delta) {
    t += delta;
  }
};

}  // namespace error_code_utils
