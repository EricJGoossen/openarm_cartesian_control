#!/usr/bin/env python3
"""
fk_pose.py -- print the current TCP pose of one OpenArm arm.

Reads the latest /joint_states once, runs Pinocchio forward kinematics on a
reduced model (same joints the cartesian controller commands), and prints the
end-effector pose. Used by the bash test wrappers to build relative-offset
goals from wherever the arm currently is.

Output (one line, space-separated, for easy bash parsing):
    px py pz qw qx qy qz

Usage:
    ./fk_pose.py --arm left
    ./fk_pose.py --arm right --ee openarm_right_hand_tcp

Requires: the robot_description on the /robot_description topic (latched) and
/joint_states being published. Both are up in the normal launch.
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String

import pinocchio as pin


# Arm joint name lists must match the controller's `joints` param order.
ARM_JOINTS = {
    "left":  [f"openarm_left_joint{i}"  for i in range(1, 8)],
    "right": [f"openarm_right_joint{i}" for i in range(1, 8)],
}
DEFAULT_EE = {
    "left":  "openarm_left_hand_tcp",
    "right": "openarm_right_hand_tcp",
}


class FkNode(Node):
    def __init__(self, arm, ee_frame):
        super().__init__("fk_pose_helper")
        self.arm = arm
        self.ee_frame = ee_frame
        self.joint_order = ARM_JOINTS[arm]

        self.urdf = None
        self.joint_state = None

        # robot_description is latched (transient local).
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(String, "/robot_description",
                                 self._urdf_cb, latched)
        self.create_subscription(JointState, "/joint_states",
                                 self._js_cb, 10)

    def _urdf_cb(self, msg):
        self.urdf = msg.data

    def _js_cb(self, msg):
        # Keep the most recent that contains all our joints.
        if all(j in msg.name for j in self.joint_order):
            self.joint_state = msg

    def ready(self):
        return self.urdf is not None and self.joint_state is not None

    def compute(self):
        # Build full model from URDF, reduce to the arm's joints (mirrors the
        # controller's Option-A reduction so FK matches exactly).
        full = pin.buildModelFromXML(self.urdf)

        lock = []
        for jid in range(1, full.njoints):
            if full.names[jid] not in self.joint_order:
                lock.append(jid)
        q_ref = pin.neutral(full)
        model = pin.buildReducedModel(full, lock, q_ref)
        data = model.createData()

        # Order our joint values to match the *reduced model's* joint order,
        # not the controller's param order. (This is exactly the ordering
        # subtlety we want to surface -- see check_ordering below.)
        name_to_pos = dict(zip(self.joint_state.name, self.joint_state.position))
        q = np.zeros(model.nq)
        for jid in range(1, model.njoints):
            jname = model.names[jid]
            idx_q = model.joints[jid].idx_q
            q[idx_q] = name_to_pos[jname]

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        fid = model.getFrameId(self.ee_frame)
        if fid == model.nframes:
            raise RuntimeError(f"EE frame '{self.ee_frame}' not in reduced model")
        oMf = data.oMf[fid]
        pos = oMf.translation
        quat = pin.Quaternion(oMf.rotation)  # normalized
        return pos, quat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True, choices=["left", "right"])
    ap.add_argument("--ee", default=None, help="EE frame name (default per arm)")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    ee = args.ee or DEFAULT_EE[args.arm]

    rclpy.init()
    node = FkNode(args.arm, ee)

    import time
    t0 = time.time()
    while rclpy.ok() and not node.ready() and (time.time() - t0) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.ready():
        node.get_logger().error(
            "Timed out waiting for /robot_description and /joint_states")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        pos, quat = node.compute()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # px py pz qw qx qy qz
    print(f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
          f"{quat.w:.6f} {quat.x:.6f} {quat.y:.6f} {quat.z:.6f}")


if __name__ == "__main__":
    main()