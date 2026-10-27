# RoboRacer Autonomous

Autonomous RoboRacer project for real-time LiDAR mapping, unknown-track exploration, physical vehicle control, safety supervision, lap detection, and learned trajectory following.

## Separate F1TENTH simulation setup

- [F1TENTH setup page and downloads](https://boscook.github.io/RoboRacer-RMIT/f1tenth/)
- [Custom single-file F1TENTH application](docs/f1tenth/downloads/F1TENTH_RACER.py)
- [Source-only setup, vendor sources, maps and licenses](docs/f1tenth/downloads/RoboRacer_F1TENTH_Source_eaad55f5d6a8.zip)
- [Full offline image/setup release](https://github.com/boscook/RoboRacer-RMIT/releases/tag/f1tenth-sandbox-20261027T184947163160Z)
- [Machine-readable validation report](docs/f1tenth/downloads/VALIDATION_REPORT.json)

This **separate 2D F1TENTH Gym/ROS 2 sandbox** includes 23 supplied track datasets,
Windows Start/Stop launchers, a local live browser view and a native Linux shortcut
installer. The approximately 450 MB offline kit is a GitHub Release asset, not a
large Git/Pages file. The smaller source ZIP builds its dependency image on first
launch and needs internet. Docker/Python/browser prerequisites remain; no host ROS
or GPU is required. Pages is a download site, not an online simulation service.

All 23 maps passed data/clearance checks, while Melbourne completed a verified
zero-collision full lap (79.790 simulated / 84.579 wall seconds) and actual STOP,
reset/recovery and queued-start cancellation tests. Native Ubuntu on the other PC
remains untested. This uses supplied map/line priors and actual Gym odometry, not
fresh SLAM or LiDAR-only autonomy. **Do not compare its different track/physics with
the AutoDRIVE 5.8218-second result, or use it on a physical car.** Upstream sources,
hashes, GPL map license/attribution and MIT simulator/bridge licenses are retained.

## Latest simulation-only code and GitHub Pages

- [Project website](https://boscook.github.io/RoboRacer-RMIT/)
- [Simulator benchmark downloads, track maps and evidence](https://boscook.github.io/RoboRacer-RMIT/simulator-benchmark.html)
- [Latest single custom source file](docs/downloads/ONEFILE_RACER.py)
- [Executable one-file Linux transfer archive](docs/downloads/ONEFILE_RACER.tar.gz)
- [Download checksums and measurement provenance](docs/downloads/benchmark-evidence.json)

The latest single-file release includes joint line/profile planning, dense-feasible fallback retention, repeated-clean-lap median tuning, zero-propulsion coasting/STOP handling and Windows startup/window guards. It is **separate from the physical ROS packages below**. Its historical best eligible clean full lap is **5.8218 seconds**, not an under-five-second result or a guaranteed time on restart. Nine later bounded candidates did not beat that record; robustness fixes are not a new speed-record claim. Measurements used the Windows simulator with Linux ROS Docker. Native Ubuntu GPU/UI driving and lap times remain unvalidated.

- [Exact follow-up results and rejected/slower attempts](docs/downloads/FASTER_FOLLOWUP_RESULTS.md)
- [Unscaled planned line/speed-profile CSV](docs/downloads/SPEED_PROFILE.csv)
- [Earlier 6.9607-second release and original evidence](https://boscook.github.io/RoboRacer-RMIT/releases/2026-10-27-windows-startup-fix/)

This mode has **no imposed command-speed ceiling by default** (<code>--max-speed 0</code>); physical/model constraints and finite actuator commands remain. It uses privileged simulator pose, genuine prior track memory and direct actuator control. Crash-triggered stopping and proximity/predictive governors are disabled; STOP, stale-data and host-liveness checks remain. Collision/teleport/warmup/flagged-transition laps are ineligible, while other clean full laps from a session with a collision remain individual-clean-lap results. **Do not use this benchmark to control a physical car.** ROS, Docker images, official simulator and saved map/track data remain dependencies; GitHub Pages serves downloads, not a browser-based simulator.

## Project Goal

The vehicle is designed to be placed on an unknown closed track and:

1. Read the environment using a Hokuyo 2D LiDAR.
2. Estimate motion using RF2O laser odometry.
3. Build a live map using SLAM Toolbox.
4. Detect drivable free space and drive autonomously.
5. Detect completion of a lap.
6. Save the map and first-lap trajectory.
7. Build a learned trajectory for later laps.
8. Continue driving with an independent LiDAR safety layer.

## Hardware

- NVIDIA Jetson Orin Nano
- Hokuyo UTM/UST-series 2D LiDAR
- VESC 6 MK VI
- Rlaarlo RC chassis
- Brushless motor
- Steering servo
- LiPo battery

## Software

- Ubuntu 22.04
- ROS 2 Humble
- Autoware control messages
- RF2O Laser Odometry
- SLAM Toolbox
- Hokuyo `urg_node`
- Custom RoboRacer ROS 2 packages

## Architecture

```text
Hokuyo LiDAR
    |
    v
  /scan
    |
    +--------------------+
    |                    |
    v                    v
RF2O Odometry      Track Detector
    |                    |
  /odom                  v
    |              Exploration Controller
    v                    |
SLAM Toolbox             v
    |              Controller Selector
    v                    |
Live Map                 v
                  Safety Supervisor
                         |
                         v
                Safe Control Adapter
                         |
                         v
               Autoware Control Msg
                         |
                         v
                   VESC Bridge
                         |
                         v
                 Physical Vehicle
```

## Lap Memory Flow

```text
Lap 1:
Explore + Map + Record
        |
        v
Lap Complete
        |
        v
BUILDING_MEMORY
        |
        v
Stop Vehicle
        |
        +--> Save map + pose graph
        +--> Save exploration trajectory
        +--> Optimise trajectory
        |
        v
MEMORY_READY
        |
        v
Lap 2+:
Learned Trajectory Follower
```

## Main Custom Packages

The repository is intended to contain the project's custom ROS 2 packages, including:

- `roboracer_track_detector`
- `roboracer_track_memory`
- `roboracer_autoware_interface`
- `roboracer_localization_adapter`
- `roboracer_bringup`
- `roboracer_description`

Configuration files and physical startup scripts are stored separately under `config/` and `scripts/`.

## Physical Safety

The physical vehicle should start disarmed. The final control path uses:

- manual drive enable
- automatic drive enable
- independent LiDAR safety state
- VESC bridge command timeout/watchdog

Initial tests should be performed with the wheels raised before any ground testing.

## Repository Structure

```text
RoboRacer-Autonomous/
├── README.md
├── .gitignore
├── src/
├── config/
├── scripts/
├── docs/
└── .github/workflows/
```

## Development Status

Current work focuses on full physical integration:

- LiDAR scan streaming
- RF2O odometry
- live SLAM
- gap-based track detection
- autonomous steering and speed control
- safety supervision
- lap memory
- learned trajectory following
- staged physical testing

## Team Members 
- Bosco Tam
- Sebastian Schlegel
- Jerrell Isiderio
- Brendan Duong

## Acknowledgements

This project uses and builds on open-source ROS 2, Autoware, SLAM Toolbox, RF2O, Hokuyo ROS drivers, VESC ROS drivers, and RoboRacer/F1TENTH ecosystem work.
