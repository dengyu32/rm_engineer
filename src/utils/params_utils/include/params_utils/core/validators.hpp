#pragma once

namespace params_utils {
namespace detail {

//--------------------------------------
// Validators
//--------------------------------------

template <class T>
inline auto in_range(T low, T high) {
  return [=](T v) { return v >= low && v <= high; };
}

}  // namespace detail
}  // namespace params_utils
