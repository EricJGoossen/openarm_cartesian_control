#!/usr/bin/env bash
# stage23_impedance.sh -- Stage 2 (single-axis impedance) + Stage 3 (tuning).
#
# Brings gains up, but isolates one Cartesian DOF at a time. Start at LOW
# stiffness (well below the 500 N/m config) and ramp. Keep an e-stop in hand.
#
# Run:  ARM=left ./stage23_impedance.sh
source "$(dirname "$0")/common.sh"

# Low starting gains for safe single-axis tests.
K_LOW="200,200,0,0,0,0"
D_LOW="5,5,0,0,0,0"

banner "Stage 2 setup -- low stiffness"
echo "Setting conservative gains for single-axis tests:"
echo "  k_gains = [$K_LOW]"
echo "  d_gains = [$D_LOW]"
set_k "$K_LOW"
set_d "$D_LOW"
confirm "Gains set. Ready to command single-axis offsets?"

run_axis() {
  local axis=$1 off=$2 label=$3
  banner "Stage 2 -- $label"
  echo "Commanding $axis offset of $off from current pose. Expect motion in"
  echo "$axis ONLY -- cross-axis motion means a frame/Jacobian ordering bug."
  confirm "Send the $label goal?"
  send_offset "$axis" "$off" 4.0
  c_grn "$label done."
  echo
}

# 2a/2b: translational, one axis at a time, small steps.
run_axis x  0.3 "single +x translation (3 cm)"
run_axis y  0.3 "single +y translation (3 cm)"
run_axis z -0.3 "single -z translation (3 cm)  [watch floor clamp at z=0]"

# 2c: rotational, one axis at a time, small angle.
run_axis rx 10 "single rotation about x (10 deg)"
run_axis ry 10 "single rotation about y (10 deg)  [check SHORT-path rotation]"
run_axis rz 10 "single rotation about z (10 deg)"

banner "Stage 3a -- stiffness sweep"
echo "At a fixed setpoint (current pose held), push the TCP off-target by hand"
echo "and feel the restoring force grow as stiffness ramps toward the 500 config."
for k in "150,150,150,15,15,15" "300,300,300,30,30,30" "500,500,500,50,50,50"; do
  echo; echo "k_gains = [$k]"
  set_k "$k"
  send_offset x 0.0 5.0    # hold current pose for the push test
  confirm "Restoring force scales with stiffness? No buzz/oscillation?"
done
c_grn "Stiffness sweep done."

banner "Stage 3b/3c -- damping + step response"
echo "Restoring config damping. Displace TCP by hand and release: return should"
echo "be smooth, not springy. Then a discrete 5 cm step to characterize overshoot."
set_k "500,500,500,50,50,50"
set_d "50,50,50,5,5,5"
confirm "Displace-and-release: smooth (well-damped) return?"
echo "Now a 5 cm step in +x:"
send_offset x 0.05 4.0
confirm "Step response: acceptable overshoot/settling?"
c_grn "Stage 2-3 complete. Impedance validated and tuned."