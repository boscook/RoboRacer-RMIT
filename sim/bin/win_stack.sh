#!/usr/bin/env bash
printf '\033]0;3-STACK\007'
echo "Waiting for the bridge in 2-BRIDGE to start..."
for i in $(seq 1 90); do
  [ -e /tmp/roborace_quit ] && exit 0
  docker exec rr_twin bash -ic 'ros2 topic list' 2>/dev/null | grep -q '/autodrive/roboracer_1/odom' && break
  sleep 1
done
echo "Bridge is up. Starting the stack. Click Connected in the simulator any time."
while [ ! -e /tmp/roborace_quit ]; do
  echo "== Resetting the car and clearing track memory =="
  docker exec rr_twin bash -ic 'rm -rf ~/roboracer_ws/track_memory/*; ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool "{data: true}" >/dev/null; sleep 1; ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool "{data: false}" >/dev/null' >/dev/null 2>&1
  echo "== Starting the RoboRacer stack (disarmed; arm it from 5-CMD) =="
  docker exec -it rr_twin bash -ic 'export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:/root/roboracer_ws/install/roboracer_autodrive_bridge; ros2 launch roboracer_autodrive_bridge sim.launch.py steer_sign:=1.0 max_steer_angle_rad:=0.5236'
  [ -e /tmp/roborace_quit ] && break
  if ! docker ps --format '{{.Names}}' | grep -qx rr_twin; then echo "The rr_twin container is gone."; break; fi
  echo "Stack stopped. Restarting in 3 s (press q in 5-CMD to quit everything)."
  sleep 3
done
read -r -p "Press Enter to close this window."
