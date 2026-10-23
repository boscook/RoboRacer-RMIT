#!/usr/bin/env bash
# Usage: ./set_speed.sh LAP1 TOP GRIP [COAST] [ACCEL] [SAFETY]
#   COAST  how fast the plan expects the car to slow down (m/s^2, no brakes)
#   SAFETY braking the safety layer assumes (sim backstop), default 2.5
#   LAP1  lap-1 exploration speed (m/s)        e.g. 1.2
#   TOP   top speed on laps 2+ (m/s)            e.g. 3.0
#   GRIP  corner grip, sideways m/s^2           e.g. 2.0
#   ACCEL how hard the car can accelerate m/s^2 default 1.0
# Edits the repo, then (if rr_twin is running) copies the files into it.
# Press r in 5-CMD afterwards to restart with the new values.
set -e
L1=${1:?lap-1 speed}; L2=${2:?top speed}; ALAT=${3:-1.5}
PBRK=${4:-0.8}; ACC=${5:-1.0}; BRK=${6:-2.5}
R=${R:-$HOME/roboracer_twin/roboracer-rmit}
TC=src/roboracer_track_detector/roboracer_track_detector/track_controller.py
OP=src/roboracer_track_memory/roboracer_track_memory/scan_trajectory_optimizer.py
TF=src/roboracer_track_memory/roboracer_track_memory/trajectory_follower.py
SS=src/roboracer_track_memory/roboracer_track_memory/safety_supervisor.py
cd "$R"
N='[0-9]+(\.[0-9]+)?'
calc() { python3 -c "print(round($1, 2))"; }
LA=$(calc "min(2.5, max(0.8, 0.5 + 0.5*$L2))")
RISE=$(calc "$ACC + 0.5")
# Lap 1 (exploration)
sed -E -i "s/(self\.max_speed = )$N/\1$L1/" $TC
# Lap 2+ speed plan
sed -E -i "s/('--vmax', type=float, default=)$N/\1$L2/; s/('--alat', type=float, default=)$N/\1$ALAT/; s/('--aacc', type=float, default=)$N/\1$ACC/; s/('--abrk', type=float, default=)$N/\1$PBRK/" $OP
# Follower: cap + lookahead
sed -E -i "s/(declare_parameter\('max_speed', )$N/\1$L2/" $TF
sed -E -i "/'lookahead_distance',/{n;s/$N/$LA/}" $TF
# Safety supervisor: braking assumption + how fast speed may rise
sed -E -i "s/(self\.braking_deceleration = )$N/\1$BRK/" $SS
sed -E -i "s/(self\.speed_rise_rate = )$N/\1$RISE/" $SS
echo "lap1=$L1 top=$L2 grip=$ALAT coast=$PBRK safety_brake=$BRK accel=$ACC rise=$RISE lookahead=$LA"
grep -nE "self\.max_speed = " $TC
grep -nE "'--(vmax|alat|aacc|abrk)'" $OP
grep -nE "braking_deceleration = |speed_rise_rate = " $SS
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx rr_twin; then
  for f in $TC $OP $TF $SS; do docker cp "$f" "rr_twin:/root/roboracer_ws/$f"; done
  echo "Copied into rr_twin. Press r in 5-CMD to restart with these values."
else
  echo "rr_twin not running: values are in the repo. Rebuild the image to bake them in."
fi
