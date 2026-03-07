import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from engineer_interfaces.msg import HFSMIntent
from std_msgs.msg import Int32


def _log_run(node: Node, start_wall: float, message: str) -> None:
    now = node.get_clock().now()
    stamp = now.to_msg()
    elapsed = time.monotonic() - start_wall
    node.get_logger().info(
        f"[{stamp.sec}.{stamp.nanosec:09d}] {message} (elapsed {elapsed:.2f}s)"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="HFSM intent runner for repeated tests.")
    parser.add_argument("--count", type=int, default=50)
    parser.add_argument("--trigger_intent", type=int, default=2)
    parser.add_argument("--reset_intent", type=int, default=1)
    parser.add_argument("--sleep_after_trigger", type=float, default=0.2)
    parser.add_argument("--sleep_after_reset", type=float, default=0.2)
    parser.add_argument("--wait_between_runs", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("hfsm_intent_runner")

    qos = QoSProfile(depth=10)
    intent_pub = node.create_publisher(HFSMIntent, "/hfsm_intents", qos)
    round_pub = node.create_publisher(Int32, "/test_round", qos)

    start_wall = time.monotonic()
    total_runs = max(args.count, 0)

    try:
        for i in range(1, total_runs + 1):
            if not rclpy.ok():
                break

            _log_run(node, start_wall, f"Run {i}/{total_runs}: publish test_round={i}")
            round_msg = Int32()
            round_msg.data = i
            round_pub.publish(round_msg)
            rclpy.spin_once(node, timeout_sec=0.0)

            _log_run(node, start_wall, f"Run {i}/{total_runs}: publish intent_id={args.trigger_intent} (trigger)")
            trigger_msg = HFSMIntent()
            trigger_msg.intent_id = int(args.trigger_intent)
            trigger_msg.stamp = node.get_clock().now().to_msg()
            intent_pub.publish(trigger_msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(max(args.sleep_after_trigger, 0.0))

            _log_run(node, start_wall, f"Run {i}/{total_runs}: publish intent_id={args.reset_intent} (reset)")
            reset_msg = HFSMIntent()
            reset_msg.intent_id = int(args.reset_intent)
            reset_msg.stamp = node.get_clock().now().to_msg()
            intent_pub.publish(reset_msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(max(args.sleep_after_reset, 0.0))

            time.sleep(max(args.wait_between_runs, 0.0))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
