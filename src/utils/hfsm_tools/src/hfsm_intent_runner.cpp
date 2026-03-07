#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "engineer_interfaces/msg/hfsm_intent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/int32.hpp"

namespace {

struct Options {
  int count = 50;
  int trigger_intent = 3;
  int reset_intent = 1;
  double sleep_after_trigger = 23;
  double sleep_after_reset = 5;
  double wait_between_runs = 2;
  double startup_delay = 1.0;
  bool wait_for_subscriber = true;
  double max_wait_subscriber_sec = 5.0;
  bool has_startup_delay = false;
  bool has_wait_for_subscriber = false;
  bool has_max_wait_subscriber_sec = false;
};

void PrintUsage(const char *prog) {
  std::cout << "HFSM intent runner for repeated tests.\n\n"
            << "Usage: " << prog << " [--count N] [--trigger_intent N] [--reset_intent N]\n"
            << "       [--sleep_after_trigger S] [--sleep_after_reset S] [--wait_between_runs S]\n"
            << "       [--startup_delay S] [--wait_for_subscriber B] [--max_wait_subscriber_sec S]\n\n"
            << "Options:\n"
            << "  --count N               number of test runs (default: 50)\n"
            << "  --trigger_intent N      intent_id to trigger (default: 2)\n"
            << "  --reset_intent N        intent_id to reset (default: 1)\n"
            << "  --sleep_after_trigger S sleep seconds after trigger publish (default: 0.2)\n"
            << "  --sleep_after_reset S   sleep seconds after reset publish (default: 0.2)\n"
            << "  --wait_between_runs S   sleep seconds between runs (default: 0.5)\n"
            << "  --startup_delay S       sleep seconds before any publish (default: 1.0)\n"
            << "  --wait_for_subscriber B wait for intent subscriber true/false (default: true)\n"
            << "  --max_wait_subscriber_sec S\n"
            << "                          max seconds to wait for subscriber (0 = infinite, default: 5.0)\n"
            << "  -h, --help              show this help message\n";
}

bool ParseArgs(const std::vector<std::string> &args, Options &out, bool &show_help,
               std::string &error) {
  show_help = false;
  for (size_t i = 1; i < args.size(); ++i) {
    const std::string &arg = args[i];
    if (arg == "-h" || arg == "--help") {
      show_help = true;
      return true;
    }

    auto read_value = [&](std::string &value) -> bool {
      if (i + 1 >= args.size()) {
        error = "missing value for " + arg;
        return false;
      }
      value = args[++i];
      return true;
    };

    auto parse_int = [&](const std::string &value, int &dst) -> bool {
      try {
        size_t pos = 0;
        int parsed = std::stoi(value, &pos);
        if (pos != value.size()) {
          throw std::invalid_argument("trailing");
        }
        dst = parsed;
      } catch (const std::exception &) {
        error = "invalid integer for " + arg + ": " + value;
        return false;
      }
      return true;
    };

    auto parse_double = [&](const std::string &value, double &dst) -> bool {
      try {
        size_t pos = 0;
        double parsed = std::stod(value, &pos);
        if (pos != value.size()) {
          throw std::invalid_argument("trailing");
        }
        dst = parsed;
      } catch (const std::exception &) {
        error = "invalid float for " + arg + ": " + value;
        return false;
      }
      return true;
    };

    auto parse_bool = [&](const std::string &value, bool &dst) -> bool {
      if (value == "true" || value == "1") {
        dst = true;
        return true;
      }
      if (value == "false" || value == "0") {
        dst = false;
        return true;
      }
      error = "invalid bool for " + arg + ": " + value;
      return false;
    };

    std::string value;
    if (arg.rfind("--count=", 0) == 0) {
      value = arg.substr(std::string("--count=").size());
      if (!parse_int(value, out.count)) {
        return false;
      }
      continue;
    }
    if (arg == "--count") {
      if (!read_value(value) || !parse_int(value, out.count)) {
        return false;
      }
      continue;
    }
    if (arg.rfind("--trigger_intent=", 0) == 0) {
      value = arg.substr(std::string("--trigger_intent=").size());
      if (!parse_int(value, out.trigger_intent)) {
        return false;
      }
      continue;
    }
    if (arg == "--trigger_intent") {
      if (!read_value(value) || !parse_int(value, out.trigger_intent)) {
        return false;
      }
      continue;
    }
    if (arg.rfind("--reset_intent=", 0) == 0) {
      value = arg.substr(std::string("--reset_intent=").size());
      if (!parse_int(value, out.reset_intent)) {
        return false;
      }
      continue;
    }
    if (arg == "--reset_intent") {
      if (!read_value(value) || !parse_int(value, out.reset_intent)) {
        return false;
      }
      continue;
    }
    if (arg.rfind("--sleep_after_trigger=", 0) == 0) {
      value = arg.substr(std::string("--sleep_after_trigger=").size());
      if (!parse_double(value, out.sleep_after_trigger)) {
        return false;
      }
      continue;
    }
    if (arg == "--sleep_after_trigger") {
      if (!read_value(value) || !parse_double(value, out.sleep_after_trigger)) {
        return false;
      }
      continue;
    }
    if (arg.rfind("--sleep_after_reset=", 0) == 0) {
      value = arg.substr(std::string("--sleep_after_reset=").size());
      if (!parse_double(value, out.sleep_after_reset)) {
        return false;
      }
      continue;
    }
    if (arg == "--sleep_after_reset") {
      if (!read_value(value) || !parse_double(value, out.sleep_after_reset)) {
        return false;
      }
      continue;
    }
    if (arg.rfind("--wait_between_runs=", 0) == 0) {
      value = arg.substr(std::string("--wait_between_runs=").size());
      if (!parse_double(value, out.wait_between_runs)) {
        return false;
      }
      continue;
    }
    if (arg == "--wait_between_runs") {
      if (!read_value(value) || !parse_double(value, out.wait_between_runs)) {
        return false;
      }
      continue;
    }
    if (arg.rfind("--startup_delay=", 0) == 0) {
      value = arg.substr(std::string("--startup_delay=").size());
      if (!parse_double(value, out.startup_delay)) {
        return false;
      }
      out.has_startup_delay = true;
      continue;
    }
    if (arg == "--startup_delay") {
      if (!read_value(value) || !parse_double(value, out.startup_delay)) {
        return false;
      }
      out.has_startup_delay = true;
      continue;
    }
    if (arg.rfind("--wait_for_subscriber=", 0) == 0) {
      value = arg.substr(std::string("--wait_for_subscriber=").size());
      if (!parse_bool(value, out.wait_for_subscriber)) {
        return false;
      }
      out.has_wait_for_subscriber = true;
      continue;
    }
    if (arg == "--wait_for_subscriber") {
      if (!read_value(value) || !parse_bool(value, out.wait_for_subscriber)) {
        return false;
      }
      out.has_wait_for_subscriber = true;
      continue;
    }
    if (arg.rfind("--max_wait_subscriber_sec=", 0) == 0) {
      value = arg.substr(std::string("--max_wait_subscriber_sec=").size());
      if (!parse_double(value, out.max_wait_subscriber_sec)) {
        return false;
      }
      out.has_max_wait_subscriber_sec = true;
      continue;
    }
    if (arg == "--max_wait_subscriber_sec") {
      if (!read_value(value) || !parse_double(value, out.max_wait_subscriber_sec)) {
        return false;
      }
      out.has_max_wait_subscriber_sec = true;
      continue;
    }

    error = "unrecognized argument: " + arg;
    return false;
  }

  return true;
}

void LogRun(rclcpp::Node &node, const std::chrono::steady_clock::time_point &start_wall,
            const std::string &message) {
  const auto now = node.get_clock()->now();
  const builtin_interfaces::msg::Time stamp = now;
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_wall).count();
  RCLCPP_INFO(node.get_logger(), "[%d.%09u] %s (elapsed %.2fs)",
              static_cast<int>(stamp.sec), static_cast<unsigned int>(stamp.nanosec),
              message.c_str(), elapsed);
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  Options options;
  bool show_help = false;
  std::string error;
  const auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  if (!ParseArgs(non_ros_args, options, show_help, error)) {
    std::cerr << error << "\n";
    PrintUsage(non_ros_args.empty() ? argv[0] : non_ros_args[0].c_str());
    rclcpp::shutdown();
    return 2;
  }
  if (show_help) {
    PrintUsage(non_ros_args.empty() ? argv[0] : non_ros_args[0].c_str());
    rclcpp::shutdown();
    return 0;
  }

  auto node = std::make_shared<rclcpp::Node>("hfsm_intent_runner");
  node->declare_parameter<double>("startup_delay", options.startup_delay);
  node->declare_parameter<bool>("wait_for_subscriber", options.wait_for_subscriber);
  node->declare_parameter<double>("max_wait_subscriber_sec", options.max_wait_subscriber_sec);
  if (!options.has_startup_delay) {
    options.startup_delay = node->get_parameter("startup_delay").as_double();
  } else {
    node->set_parameter(rclcpp::Parameter("startup_delay", options.startup_delay));
  }
  if (!options.has_wait_for_subscriber) {
    options.wait_for_subscriber = node->get_parameter("wait_for_subscriber").as_bool();
  } else {
    node->set_parameter(rclcpp::Parameter("wait_for_subscriber", options.wait_for_subscriber));
  }
  if (!options.has_max_wait_subscriber_sec) {
    options.max_wait_subscriber_sec = node->get_parameter("max_wait_subscriber_sec").as_double();
  } else {
    node->set_parameter(
        rclcpp::Parameter("max_wait_subscriber_sec", options.max_wait_subscriber_sec));
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  auto intent_pub =
      node->create_publisher<engineer_interfaces::msg::HFSMIntent>("/hfsm_intents", qos);
  auto round_pub = node->create_publisher<std_msgs::msg::Int32>("/test_round", qos);

  if (options.startup_delay > 0.0) {
    RCLCPP_INFO(node->get_logger(), "Startup delay %.2fs begin", options.startup_delay);
    const auto delay_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(options.startup_delay));
    rclcpp::sleep_for(delay_ns);
    RCLCPP_INFO(node->get_logger(), "Startup delay complete");
  }

  // This runner intentionally waits for subscriber and adds a startup delay
  // to avoid startup race conditions under VOLATILE QoS.
  // This is a design choice, not a workaround.
  if (options.wait_for_subscriber) {
    const auto max_wait = options.max_wait_subscriber_sec;
    const auto start_wait = node->get_clock()->now();
    while (rclcpp::ok() && intent_pub->get_subscription_count() == 0) {
      const auto count = intent_pub->get_subscription_count();
      const auto elapsed =
          (node->get_clock()->now() - start_wait).seconds();
      if (max_wait > 0.0 && elapsed > max_wait) {
        RCLCPP_WARN(node->get_logger(),
                    "Timeout waiting for subscriber (count=%zu, waited %.2fs), continue anyway",
                    count, elapsed);
        break;
      }
      RCLCPP_INFO_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "Waiting for subscriber to /hfsm_intents ... (count=%zu)", count);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    const auto final_count = intent_pub->get_subscription_count();
    if (final_count > 0) {
      RCLCPP_INFO(node->get_logger(),
                  "Subscriber connected (count=%zu). First intent will proceed.", final_count);
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "No subscriber detected. First intent will proceed anyway.");
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Not waiting for subscriber before first intent");
  }

  const auto start_wall = std::chrono::steady_clock::now();
  const int total_runs = std::max(options.count, 0);

  for (int i = 1; i <= total_runs; ++i) {
    if (!rclcpp::ok()) {
      break;
    }

    {
      std::ostringstream message;
      message << "Run " << i << "/" << total_runs << ": publish test_round=" << i;
      LogRun(*node, start_wall, message.str());
    }
    std_msgs::msg::Int32 round_msg;
    round_msg.data = i;
    round_pub->publish(round_msg);
    rclcpp::spin_some(node);

    {
      std::ostringstream message;
      message << "Run " << i << "/" << total_runs << ": publish intent_id="
              << options.trigger_intent << " (trigger)";
      LogRun(*node, start_wall, message.str());
    }
    engineer_interfaces::msg::HFSMIntent trigger_msg;
    trigger_msg.intent_id = static_cast<uint8_t>(options.trigger_intent);
    trigger_msg.stamp = node->get_clock()->now();
    intent_pub->publish(trigger_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(options.sleep_after_trigger, 0.0)));

    {
      std::ostringstream message;
      message << "Run " << i << "/" << total_runs << ": publish intent_id="
              << options.reset_intent << " (reset)";
      LogRun(*node, start_wall, message.str());
    }
    engineer_interfaces::msg::HFSMIntent reset_msg;
    reset_msg.intent_id = static_cast<uint8_t>(options.reset_intent);
    reset_msg.stamp = node->get_clock()->now();
    intent_pub->publish(reset_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(options.sleep_after_reset, 0.0)));

    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(options.wait_between_runs, 0.0)));
  }

  rclcpp::shutdown();
  return 0;
}
