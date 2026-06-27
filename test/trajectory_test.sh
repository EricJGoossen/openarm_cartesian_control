#!/usr/bin/env bash
# stage45_trajectory.sh -- Stage 4 (trajectory following) + Stage 5 (safety).
#
# Only run after Stages 0-3 pass. Exercises the action server plumbing,
# interpolation, preemption/cancel, and the clamp/reject safety paths.
#
# Run:  ARM=left ./stage45_trajectory.sh
source "$(dirname "$0")/common.sh"

banner "Stage 4a -- single-waypoint hold (plumbing)"
echo "One point at the CURRENT pose, 3 s out. Should hold and report SUCCEEDED."
echo "This exercises goal accept -> feedback -> completion with no real motion."
confirm "Send hold-in-place goal?"
send_offset x 0.0 3.0
c_grn "If status was SUCCEEDED, action plumbing works."
echo

banner "Stage 4b -- two-waypoint slow move"
echo "Current pose -> 5 cm in +x over 5 s. Confirms lerp/slerp interpolation is"
echo "smooth and feedback error stays small."
read_pose
SEC=5
read -r npx <<< "$(python3 -c "print(f'{$PX+0.05:.6f}')")"
GOAL=$(cat <<YAML
trajectory:
  points:
    - pose:
        position: {x: $PX, y: $PY, z: $PZ}
        orientation: {w: $QW, x: $QX, y: $QY, z: $QZ}
      twist: {linear: {x: 0.0,y: 0.0,z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}
      time_from_start: {sec: 0, nanosec: 0}
    - pose:
        position: {x: $npx, y: $PY, z: $PZ}
        orientation: {w: $QW, x: $QX, y: $QY, z: $QZ}
      twist: {linear: {x: 0.0,y: 0.0,z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}
      time_from_start: {sec: $SEC, nanosec: 0}
YAML
)
confirm "Send two-waypoint trajectory?"
ros2 action send_goal "$ACTION" "$ACTION_TYPE" "$GOAL" --feedback
c_grn "Two-waypoint done."
echo

banner "Stage 4c -- multi-waypoint with timing"
echo "Several points over ~12 s. Watch feedback: desired tracks waypoints,"
echo "actual lags slightly (impedance compliance), error stays bounded."
read_pose
GOAL=$(cat <<YAML
trajectory:
  points:
    - pose: {position: {x: $PX, y: $PY, z: $PZ}, orientation: {w: $QW,x: $QX,y: $QY,z: $QZ}}
      twist: {linear: {x: 0.0,y: 0.0,z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}
      time_from_start: {sec: 0, nanosec: 0}
    - pose: {position: {x: $(python3 -c "print(f'{$PX+0.04:.6f}')"), y: $PY, z: $PZ}, orientation: {w: $QW,x: $QX,y: $QY,z: $QZ}}
      twist: {linear: {x: 0.0,y: 0.0,z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}
      time_from_start: {sec: 4, nanosec: 0}
    - pose: {position: {x: $(python3 -c "print(f'{$PX+0.04:.6f}')"), y: $(python3 -c "print(f'{$PY+0.04:.6f}')"), z: $PZ}, orientation: {w: $QW,x: $QX,y: $QY,z: $QZ}}
      twist: {linear: {x: 0.0,y: 0.0,z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}
      time_from_start: {sec: 8, nanosec: 0}
    - pose: {position: {x: $PX, y: $PY, z: $PZ}, orientation: {w: $QW,x: $QX,y: $QY,z: $QZ}}
      twist: {linear: {x: 0.0,y: 0.0,z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}
      time_from_start: {sec: 12, nanosec: 0}
YAML
)
confirm "Send multi-waypoint trajectory (returns to start)?"
ros2 action send_goal "$ACTION" "$ACTION_TYPE" "$GOAL" --feedback
c_grn "Multi-waypoint done."
echo

banner "Stage 4d -- preemption + cancel"
echo "PREEMPTION: send a long goal, then a second goal mid-execution. The first"
echo "should abort with 'Preempted by new goal'; the second should run."
echo "Run these two in quick succession (use two terminals or background the 1st):"
echo
echo "  # terminal 1 (long, 15 s):"
echo "  ros2 action send_goal $ACTION $ACTION_TYPE \\"
echo "    '{trajectory: {points: [{pose: {position: {x: $PX,y: $PY,z: $PZ}, orientation: {w: $QW,x: $QX,y: $QY,z: $QZ}}, time_from_start: {sec: 15}}]}}'"
echo
echo "  # terminal 2 (a few seconds later): send any new goal -> first aborts"
echo
echo "CANCEL: send a goal, then Ctrl-C the send_goal client (or use the cancel"
echo "API). Controller should report canceled and HOLD at current reference,"
echo "not jump."
confirm "Acknowledge preemption + cancel were tested?"
c_grn "Stage 4 complete."
echo

banner "Stage 5a -- workspace position clamp"
echo "Command a setpoint OUTSIDE the position limits (config: x,y in [-0.8,0.8],"
echo "z in [0,1.2]). TCP should stop at the boundary; clamp stderr should fire."
echo "Watch in another terminal:  ros2 topic echo /rosout | grep -i clamp"
read_pose
confirm "Send an out-of-bounds +x goal (current + 1.5 m)?"
send_offset x 1.5 4.0
c_grn "Clamp test sent -- confirm TCP stopped at boundary + stderr fired."
echo

banner "Stage 5b -- joint torque saturation"
echo "Command an aggressive setpoint far from current. Joint torques should clamp"
echo "at joint_torque_limits (joints 5-7 at 10 Nm saturate first), not exceed"
echo "rated motor torque. Watch for per-joint clamp messages."
confirm "Send an aggressive 30 cm +z step (expect saturation)?"
send_offset z 0.30 3.0
c_grn "Saturation test sent -- confirm clamp messages, no over-torque."
echo

banner "Stage 5c -- empty trajectory rejection"
echo "Sending a goal with zero points -- onGoalRequest should REJECT it."
ros2 action send_goal "$ACTION" "$ACTION_TYPE" \
  '{trajectory: {points: []}}' || c_grn "Rejected as expected (non-zero exit)."
echo

banner "Stage 5d -- deactivation mid-trajectory"
echo "Start a long trajectory, then deactivate the controller while it runs."
echo "It should abort the goal cleanly and zero effort (arm goes limp/gravity)."
echo
echo "  # start a long goal, then in another terminal:"
echo "  ros2 control set_controller_state $CTRL inactive"
echo
confirm "Acknowledge deactivation-mid-trajectory was tested?"
c_grn "Stage 5 complete. Safety paths validated."
echo
banner "ALL STAGES COMPLETE"