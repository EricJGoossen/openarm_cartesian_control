#!/usr/bin/env python3
"""
check_ordering.py -- verify reduced-model joint order matches the param order.

This is the Stage 0a prerequisite. If the order differs, every torque the
controller commands is computed from a scrambled configuration vector, and all
downstream tests produce plausible-but-wrong results. Run this FIRST.

Prints, side by side:
    index | controller param order | reduced-model tree order | MATCH?

Exit code 0 if every position matches, 1 otherwise.

Usage:
    ./check_ordering.py --arm left
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import String
import pinocchio as pin


ARM_JOINTS = {
    "left":  [f"openarm_left_joint{i}"  for i in range(1, 8)],
    "right": [f"openarm_right_joint{i}" for i in range(1, 8)],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True, choices=["left", "right"])
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    param_order = ARM_JOINTS[args.arm]

    rclpy.init()
    node = Node("check_ordering")
    holder = {"urdf": None}
    latched = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )
    node.create_subscription(String, "/robot_description",
                             lambda m: holder.update(urdf=m.data), latched)

    t0 = time.time()
    while rclpy.ok() and holder["urdf"] is None and (time.time() - t0) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if holder["urdf"] is None:
        print("ERROR: no /robot_description received", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    full = pin.buildModelFromXML(holder["urdf"])
    lock = [jid for jid in range(1, full.njoints)
            if full.names[jid] not in param_order]
    model = pin.buildReducedModel(full, lock, pin.neutral(full))

    reduced_order = [model.names[jid] for jid in range(1, model.njoints)]

    node.destroy_node(); rclpy.shutdown()

    print(f"{'idx':<4} {'param order':<26} {'reduced-model order':<26} match")
    print("-" * 70)
    all_match = True
    n = max(len(param_order), len(reduced_order))
    for i in range(n):
        p = param_order[i] if i < len(param_order) else "<none>"
        r = reduced_order[i] if i < len(reduced_order) else "<none>"
        ok = (p == r)
        all_match = all_match and ok
        print(f"{i:<4} {p:<26} {r:<26} {'OK' if ok else 'MISMATCH'}")

    print("-" * 70)
    if all_match:
        print("PASS: orders match. q/dq packing is correct.")
        sys.exit(0)
    else:
        print("FAIL: orders differ. The controller needs an index map "
              "(joint_names[i] -> reduced idx_q/idx_v) before torque tests.")
        sys.exit(1)


if __name__ == "__main__":
    main()