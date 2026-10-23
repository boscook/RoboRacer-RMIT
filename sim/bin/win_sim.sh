#!/usr/bin/env bash
printf '\033]0;1-SIM\007'
echo "Starting the AutoDRIVE simulator (2026-iros-practice track)..."
docker run --name autodrive_roboracer_sim --rm -it --entrypoint /bin/bash \
  --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY \
  --privileged --gpus all \
  autodriveecosystem/autodrive_roboracer_sim:2026-iros-practice \
  -c './"AutoDRIVE Simulator.x86_64"'
echo; echo "Simulator stopped."; read -r -p "Press Enter to close this window."
