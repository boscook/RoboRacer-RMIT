#!/usr/bin/env bash
printf '\033]0;4-LOG\007'
LOGPY="$HOME/roboracer_twin/log.py"
echo "Waiting for the rr_twin container..."
until docker exec rr_twin true >/dev/null 2>&1; do [ -e /tmp/roborace_quit ] && exit 0; sleep 2; done
docker cp "$LOGPY" rr_twin:/root/log.py
docker exec -it rr_twin bash -ic 'python3 /root/log.py 2>&1 | tee /root/log.txt'
echo; read -r -p "Logger stopped. Press Enter to close this window."
