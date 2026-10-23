#!/usr/bin/env bash
# One-click launcher for the RoboRacer AutoDRIVE digital twin.
# Opens five named terminal windows: 1-SIM, 2-BRIDGE, 3-STACK, 4-LOG, 5-CMD.
BIN="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal was not found. Install it or run the steps in the setup guide by hand."
  read -r -p "Press Enter to close."; exit 1
fi
if ! docker info >/dev/null 2>&1; then
  gnome-terminal --title="RoboRacer: Docker problem" -- bash -c 'echo "Docker is not running or you are not in the docker group."; echo "Test with:  docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L"; read -r -p "Press Enter to close."'
  exit 1
fi

xhost local:root >/dev/null 2>&1
rm -f /tmp/roborace_quit
docker rm -f rr_twin autodrive_roboracer_sim >/dev/null 2>&1

win() { gnome-terminal --title="$1" -- bash "$BIN/$2"; }
win "1-SIM"    win_sim.sh;    sleep 2
win "2-BRIDGE" win_bridge.sh; sleep 2
win "3-STACK"  win_stack.sh;  sleep 1
win "4-LOG"    win_log.sh;    sleep 1
win "5-CMD"    win_cmd.sh
