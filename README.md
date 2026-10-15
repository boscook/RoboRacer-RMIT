# RoboRacer Autonomous

Autonomous RoboRacer project for real-time LiDAR mapping, unknown-track exploration, physical vehicle control, safety supervision, lap detection, and learned trajectory following.

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

## Acknowledgements

This project uses and builds on open-source ROS 2, Autoware, SLAM Toolbox, RF2O, Hokuyo ROS drivers, VESC ROS drivers, and RoboRacer/F1TENTH ecosystem work.
