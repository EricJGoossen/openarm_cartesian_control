#!/usr/bin/env bash
# stage01_gravity.sh -- Stage 0 (static checks) and Stage 1 (gravity comp).
#
# These are the foundation. Do not proceed past a failing test here -- every
# later stage assumes the model, frame, and gravity term are correct.
#
# Run:  ARM=left ./stage01_gravity.sh
source "$(dirname "$0")/common.sh"

banner "Stage 0a -- joint ordering (PREREQUISITE)"
echo "If this fails, STOP. The controller needs an index map before any torque test."
python3 "$HERE/check_ordering.py" --arm "$ARM"
if [[ $? -ne 0 ]]; then
  c_red "Ordering check FAILED -- aborting. Fix ordering before continuing."
  exit 1
fi
c_grn "Ordering OK."
echo

banner "Stage 0b -- frame / pose sanity at home"
echo "Arm homes to zero on activation. The pose below should match where the"
echo "TCP physically sits at the zero configuration. If it's wildly off, the EE"
echo "frame or base frame is wrong."
read_pose
confirm "Does the printed pose match the real arm's home TCP location?"
c_grn "Pose sanity acknowledged."
echo

banner "Stage 0c / 1a -- gravity-comp only, fully backdrivable"
echo "Zeroing k_gains and d_gains. Arm should hold against gravity and be"
echo "freely backdrivable by hand -- no spring, no sag, no drift."
set_k "0,0,0,0,0,0"
set_d "0,0,0,0,0,0"
confirm "Push each link by hand. Weightless? Holds where left? No drift?"
c_grn "Backdrive / gravity-comp acknowledged."
echo

banner "Stage 1b -- gravity hold across the workspace"
echo "Move the arm by hand to several distinct poses (extended, folded, up,"
echo "across) and release each. It should hold every one. Sag only when extended"
echo "points at wrong URDF inertias / mass."
confirm "Holds in all tested poses?"
c_grn "Stage 0-1 complete. Gravity compensation validated."