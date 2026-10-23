# AutoDRIVE simulator twin

Runs this repository's autonomy stack against the AutoDRIVE RoboRacer simulator
(IROS 2026 practice track) in Docker, with no hardware.

Best simulator lap so far: **12.6 s** (started at 23.6 s), 3.5 m/s top speed.

> **Simulator only.** The speeds, grip, braking and safety margins in this branch
> are tuned for the simulator. Do not run them on the physical car. Start the
> car from the original values, test with the wheels raised first, then raise
> speed in small steps.

## Layout on the Linux machine

The Docker build expects this folder layout:

```
~/roboracer_twin/
  roboracer-rmit/               <- this repository
  roboracer_autodrive_bridge/   <- copy of sim/roboracer_autodrive_bridge
  roboracer.Dockerfile  bin/  log.py  set_speed.sh  lap_report.py ...
```

```bash
mkdir -p ~/roboracer_twin && cd ~/roboracer_twin
git clone -b sim-twin-fast https://github.com/boscook/RoboRacer-RMIT roboracer-rmit
cp -r roboracer-rmit/sim/. .
docker build -t roboracer-twin:dev -f roboracer.Dockerfile .
bash install_desktop_launcher.sh      # then double-click "RoboRacer Sim"
```

Simulator image: `autodriveecosystem/autodrive_roboracer_sim:2026-iros-practice`.
In the 5-CMD window: **a** arm, **d** disarm, **r** restart (clears track memory), **q** quit.

## Speed settings

```bash
~/roboracer_twin/set_speed.sh LAP1 TOP GRIP [COAST] [ACCEL] [SAFETY]
~/roboracer_twin/set_speed.sh 1.0 3.5 4.0 3.0 2.5 4.0     # 12.6 s setup
```

| Argument | Meaning |
|---|---|
| LAP1 | Lap-1 exploration speed, m/s |
| TOP | Top speed on laps 2+, m/s |
| GRIP | Allowed sideways acceleration in corners, m/s² |
| COAST | Deceleration the speed plan assumes at zero throttle, m/s² |
| ACCEL | Acceleration the speed plan assumes, m/s² |
| SAFETY | Braking the safety layer assumes, m/s² |

The script edits the repository and copies the files into a running `rr_twin`
container; press **r** then **a** to use them. Rebuild the image to keep them.

`lap_report.py` prints track length and planned lap time:
`docker cp lap_report.py rr_twin:/root/ && docker exec rr_twin python3 /root/lap_report.py`

## What changed compared with `main`

Simulator kit tuning (from the original student kit): smaller distances and
thresholds for the 1.3 m corridor, lap detector and map-saving fixes, reverse
"unstick" manoeuvre, side-wall distances recorded on lap 1.

Speed work:

- **`scan_trajectory_optimizer.py`** (new): builds the racing line from the lap-1
  path and side-wall distances, without needing the SLAM map. Drops lap-1 points
  where the car was not moving forward, ignores kinks tighter than 0.5 m, and
  plans speed from corner curvature with acceleration/deceleration limits.
- **`trajectory_follower.py`**: uses the full 0.52 rad sim steering; lookahead
  grows with speed (prevents weaving above 3 m/s); steers for the pose predicted
  0.2 s ahead to cover steering delay; learns lap by lap, shifting the line where
  the car ran wide and slowing (at most 25 %) where it missed badly.
- **`safety_supervisor.py`**: stopping zone sized from measured speed; smooth
  speed limit from free distance; free distance measured along the car's path in
  a car-wide strip (straight or along the steering arc, whichever is longer), so
  side walls of a narrow corridor no longer cap the speed; hard stop only when
  something is within ±10° ahead; LiDAR timeout 0.5 s.
- **`trajectory_recorder.py`**: records lap-1 points only when moving forward.
- **`sim/roboracer_autodrive_bridge/sim_adapter.py`**: braking with negative
  throttle is available but switched off (`brake_max` 0.0); the car slows on its
  own at zero throttle.
