#!/usr/bin/env bash
# common.sh -- shared helpers for the OpenArm cartesian test harness.
# Source this from the stage scripts: source "$(dirname "$0")/common.sh"

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- config (override via env) ----------------------------------------------
: "${ARM:=left}"                         # left | right
: "${CTRL:=${ARM}_cartesian_impedance_controller}"
: "${ACTION:=/${CTRL}/follow_cartesian_trajectory}"
: "${GRIPPER:=${ARM}_gripper_controller}"

ACTION_TYPE="cartesian_control_msgs/action/FollowCartesianTrajectory"

# ---- pretty printing --------------------------------------------------------
c_red()   { printf '\033[31m%s\033[0m\n' "$*"; }
c_grn()   { printf '\033[32m%s\033[0m\n' "$*"; }
c_yel()   { printf '\033[33m%s\033[0m\n' "$*"; }
hr()      { printf '%s\n' "----------------------------------------------------------------------"; }
banner()  { hr; printf '  %s\n' "$*"; hr; }

confirm() {
  # Pause for the operator. Any torque-on test should gate on this.
  local msg="${1:-Proceed?}"
  c_yel ">> $msg"
  read -r -p "   [enter] to continue, Ctrl-C to abort: " _
}

# ---- pose helpers -----------------------------------------------------------
# Read current TCP pose -> sets globals PX PY PZ QW QX QY QZ
read_pose() {
  local out
  out="$(python3 "$HERE/fk_pose.py" --arm "$ARM")" || {
    c_red "fk_pose.py failed"; return 1; }
  read -r PX PY PZ QW QX QY QZ <<< "$out"
  printf 'current TCP: pos=(%.4f %.4f %.4f) quat=(%.4f %.4f %.4f %.4f)\n' \
    "$PX" "$PY" "$PZ" "$QW" "$QX" "$QY" "$QZ"
}

# Send a single-waypoint goal at an absolute pose held for $TSEC seconds.
# args: px py pz qw qx qy qz tsec
send_hold() {
  local px=$1 py=$2 pz=$3 qw=$4 qx=$5 qy=$6 qz=$7 tsec=$8
  local sec=${tsec%.*}
  local goal
  goal=$(cat <<YAML
trajectory:
  points:
    - pose:
        position: {x: $px, y: $py, z: $pz}
        orientation: {w: $qw, x: $qx, y: $qy, z: $qz}
      twist:
        linear:  {x: 0.0, y: 0.0, z: 0.0}
        angular: {x: 0.0, y: 0.0, z: 0.0}
      time_from_start: {sec: $sec, nanosec: 0}
YAML
)
  echo "Sending goal (hold ${tsec}s):"
  echo "  pos=($px $py $pz) quat=($qw $qx $qy $qz)"
  ros2 action send_goal "$ACTION" "$ACTION_TYPE" "$goal" --feedback
}

# Send a goal at current pose offset along one axis.
# args: axis offset tsec    (axis: x y z rx ry rz; offset m or deg)
send_offset() {
  local axis=$1 offset=$2 tsec=$3
  read_pose || return 1
  python3 - "$axis" "$offset" "$PX" "$PY" "$PZ" "$QW" "$QX" "$QY" "$QZ" <<'PY' > /tmp/_off.$$
import sys, math
axis, off = sys.argv[1], float(sys.argv[2])
px,py,pz,qw,qx,qy,qz = map(float, sys.argv[3:10])
def qmul(a,b):
    aw,ax,ay,az=a; bw,bx,by,bz=b
    return (aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw)
if axis in ("x","y","z"):
    d={"x":0,"y":1,"z":2}[axis]; p=[px,py,pz]; p[d]+=off; px,py,pz=p
elif axis in ("rx","ry","rz"):
    d={"rx":0,"ry":1,"rz":2}[axis]; h=math.radians(off)/2
    v=[0,0,0]; v[d]=math.sin(h); dq=(math.cos(h),*v)
    qw,qx,qy,qz=qmul(dq,(qw,qx,qy,qz))
else:
    sys.exit(f"bad axis {axis}")
print(f"{px:.6f} {py:.6f} {pz:.6f} {qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}")
PY
  read -r npx npy npz nqw nqx nqy nqz < /tmp/_off.$$ ; rm -f /tmp/_off.$$
  send_hold "$npx" "$npy" "$npz" "$nqw" "$nqx" "$nqy" "$nqz" "$tsec"
}

# Runtime gain set (no rebuild). args: comma-separated gains, e.g. "0,0,0,0,0,0"
# Forces every element to float form so ROS sees a double_array (not int_array).
_floatify() {
  # turn "0,80,8" -> "0.0, 80.0, 8.0"
  python3 -c "import sys; print(', '.join(f'{float(x):.6f}' for x in sys.argv[1].split(',')))" "$1"
}
set_k() { ros2 param set "/$CTRL" k_gains "[$(_floatify "$1")]"; }
set_d() { ros2 param set "/$CTRL" d_gains "[$(_floatify "$1")]"; }

# Watch clamp / controller stderr in another terminal:
#   ros2 topic echo /rosout | grep -i clamp