#!/usr/bin/env bash
printf '\033]0;2-BRIDGE\007'
echo "Starting the stack container and the AutoDRIVE bridge..."
echo "When the simulator window is up, click its Disconnected button until it says Connected."
docker run --name rr_twin --rm -it --entrypoint /bin/bash \
  --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY \
  --privileged --gpus all roboracer-twin:dev \
  -ic 'ros2 launch autodrive_roboracer bringup_headless.launch.py'
echo; echo "Bridge stopped (the rr_twin container is gone)."; read -r -p "Press Enter to close this window."
