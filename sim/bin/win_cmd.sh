#!/usr/bin/env bash
printf '\033]0;5-CMD\007'
pub() {
  if timeout 8 docker exec rr_twin bash -ic "timeout 6 ros2 topic pub --once /roboracer/drive_enable std_msgs/msg/Bool \"{data: $1}\"" 2>&1 | grep -q 'publishing #1'; then
    echo "drive_enable = $1"
  else
    echo "NOT SENT: the stack is not running yet. Wait until 3-STACK is printing log lines."
  fi
}
while true; do
  echo
  echo "  [a] ARM the car      [d] DISARM the car"
  echo "  [r] RESTART stack (resets car to start, clears track memory)"
  echo "  [m] show the moving part of the last run (from the log)"
  echo "  [q] QUIT everything (stops both containers)"
  read -r -s -n 1 -p "> " key; echo
  case "$key" in
    a) pub true ;;
    d) pub false ;;
    r) pub false
       docker exec rr_twin pkill -INT -f sim.launch.py >/dev/null 2>&1
       echo "Restarting: 3-STACK resets the car and relaunches by itself (about 15 s)." ;;
    m) docker exec rr_twin bash -c "grep -v 'cmd_v=0.00.*sim_v=0.00' /root/log.txt | grep -v nan | tail -n 30" ;;
    q) touch /tmp/roborace_quit
       docker stop rr_twin autodrive_roboracer_sim >/dev/null 2>&1
       echo "Stopped. You can close the other windows."; sleep 2; exit 0 ;;
    *) echo "unknown key" ;;
  esac
done
