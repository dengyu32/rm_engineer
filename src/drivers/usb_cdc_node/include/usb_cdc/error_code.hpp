#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"
#include "error_code_utils/status_code_adapter.hpp"

namespace usb_cdc {

enum class UsbCdcErrc {
  OpenFailed = 0,
  OpenException,
  SendFailed,
  PacketTooSmall,
};

inline SYSTEM_ERROR2_NAMESPACE::system_code to_system_code(UsbCdcErrc code) {
  using SYSTEM_ERROR2_NAMESPACE::errc;
  switch (code) {
  case UsbCdcErrc::OpenFailed:
    return errc::not_connected;
  case UsbCdcErrc::OpenException:
    return errc::io_error;
  case UsbCdcErrc::SendFailed:
    return errc::io_error;
  case UsbCdcErrc::PacketTooSmall:
  default:
    return errc::invalid_argument;
  }
}

inline error_code_utils::Error make_error(
    UsbCdcErrc code,
    const std::string &message,
    const std::map<std::string, std::string> &context = {}) {
  auto sys = to_system_code(code);
  return error_code_utils::from_system_code(error_code_utils::ErrorDomain::COMM, sys,
                                            message, context);
}

} // namespace usb_cdc
