#!/usr/bin/env bash

set -e

# ============================================================
# ROBORACER PHYSICAL AUTONOMOUS TEST
#
# Full chain:
#
# Hokuyo
#   -> RF2O
#   -> SLAM Toolbox
#   -> localization adapter
#   -> race track detector
#   -> race controller
#   -> lap detector / trajectory recorder
#   -> memory manager
#   -> trajectory follower
#   -> mode manager
#   -> controller selector
#   -> dynamic safety supervisor
#   -> safe control adapter
#   -> Autoware Control
#   -> VESC bridge
#   -> VESC driver
#
# Starts DISARMED.
# Type ARM manually before motor can move.
# ============================================================

ROS_SETUP="/opt/ros/humble/setup.bash"
AUTOWARE_SETUP="$HOME/autoware_ws/install/setup.bash"
ROBORACER_SETUP="$HOME/roboracer_ws/install/setup.bash"

RF2O_CONFIG="$HOME/roboracer_ws/config/rf2o.yaml"
SLAM_CONFIG="$HOME/roboracer_ws/config/slam_toolbox.yaml"

VESC_PORT="/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"

HOKUYO_IP="192.168.0.10"

LOG_ROOT="$HOME/roboracer_logs"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"

mkdir -p "$LOG_DIR"

# ============================================================
# ENVIRONMENT
# ============================================================

set +u
source "$ROS_SETUP"
source "$AUTOWARE_SETUP"
source "$ROBORACER_SETUP"
set -u

export RCUTILS_COLORIZED_OUTPUT=1

echo
echo "============================================================"
echo " ROBORACER PHYSICAL AUTONOMOUS TEST"
echo " Run ID: $RUN_ID"
echo " Logs:   $LOG_DIR"
echo "============================================================"
echo

# ============================================================
# CHILD PROCESS MANAGEMENT
# ============================================================

PIDS=()

start_node()
{
    NAME="$1"
    shift

    echo "[START] $NAME"

    "$@" >"$LOG_DIR/$NAME.log" 2>&1 &

    PID=$!
    PIDS+=("$PID")

    echo "        PID=$PID"
}

disarm()
{
    echo
    echo "========== MOTOR DISARM =========="

    timeout 3 ros2 topic pub --once \
        /roboracer/drive_enable \
        std_msgs/msg/Bool \
        "{data: false}" \
        >/dev/null 2>&1 || true

    # Direct final motor-zero backup.
    timeout 3 ros2 topic pub --once \
        /commands/motor/speed \
        std_msgs/msg/Float64 \
        "{data: 0.0}" \
        >/dev/null 2>&1 || true

    echo "Motor command = 0"
}

cleanup()
{
    echo
    echo "============================================================"
    echo " SHUTTING DOWN"
    echo "============================================================"

    disarm

    sleep 0.3

    for PID in "${PIDS[@]}"; do
        kill -INT "$PID" 2>/dev/null || true
    done

    sleep 2

    for PID in "${PIDS[@]}"; do
        kill "$PID" 2>/dev/null || true
    done

    echo "Shutdown complete."
}

trap cleanup EXIT INT TERM

# ============================================================
# BASIC HARDWARE CHECK
# ============================================================

echo "Checking VESC serial..."

if [ ! -e "$VESC_PORT" ]; then
    echo
    echo "ERROR: VESC not found:"
    echo "$VESC_PORT"
    exit 1
fi

echo "VESC serial OK."

echo
echo "Checking Hokuyo..."

if ! ping -c 2 -W 1 "$HOKUYO_IP" >/dev/null 2>&1; then
    echo
    echo "ERROR: Hokuyo $HOKUYO_IP not reachable."
    exit 1
fi

echo "Hokuyo network OK."

# ============================================================
# PREVENT DUPLICATE PHYSICAL STACK
# ============================================================

echo
echo "Cleaning old software-test nodes..."

pkill -INT -f "controller_selector" 2>/dev/null || true
pkill -INT -f "mode_manager" 2>/dev/null || true
pkill -INT -f "safe_control_adapter" 2>/dev/null || true
pkill -INT -f "trajectory_follower" 2>/dev/null || true

sleep 2

echo
echo
echo "Cleaning old Hokuyo session..."

pkill -INT -f "urg_node_driver" 2>/dev/null || true
sleep 3

pkill -9 -f "urg_node_driver" 2>/dev/null || true
sleep 2

echo
echo "Checking for already-running RoboRacer control nodes..."

EXISTING="$(ros2 node list 2>/dev/null | grep -E \
'rf2o|slam_toolbox|roboracer_|vesc_driver' || true)"

if [ -n "$EXISTING" ]; then
    echo
    echo "ERROR: RoboRacer control nodes already exist:"
    echo "$EXISTING"
    echo
    echo "Stop old sessions first."
    exit 1
fi

echo "No duplicate control stack detected."

# ============================================================
# BACK UP PREVIOUS TRACK MEMORY
# ============================================================

MEMORY="$HOME/roboracer_ws/track_memory"
ARCHIVE="$MEMORY/archive/$RUN_ID"

mkdir -p "$MEMORY/archive"

if [ -d "$MEMORY/active_track" ]; then

    if find "$MEMORY/active_track" -type f -print -quit 2>/dev/null | grep -q .; then

        mkdir -p "$ARCHIVE"

        cp -a \
            "$MEMORY/active_track" \
            "$ARCHIVE/" \
            2>/dev/null || true

        echo "Previous active track backed up to:"
        echo "$ARCHIVE"
    fi
fi

# Critical:
# don't allow memory_manager to accidentally use an old Lap-1 trajectory.

rm -f "$MEMORY/exploration_trajectory.csv"

mkdir -p "$MEMORY/active_track"

rm -f \
    "$MEMORY/active_track/map.yaml" \
    "$MEMORY/active_track/map.pgm" \
    "$MEMORY/active_track/track.posegraph" \
    "$MEMORY/active_track/track.data" \
    "$MEMORY/active_track/exploration_trajectory.csv" \
    "$MEMORY/active_track/learned_track.csv"

echo "Fresh-track memory prepared."

# ============================================================
# DISCOVER CUSTOM EXECUTABLE NAMES
# ============================================================

LOC_EXE="$(
    ros2 pkg executables roboracer_localization_adapter 2>/dev/null \
    | awk 'NR==1 {print $2}'
)"

BRIDGE_EXE="$(
    ros2 pkg executables roboracer_autoware_interface 2>/dev/null \
    | awk '$2 ~ /vesc.*bridge|bridge.*vesc/ {print $2; exit}'
)"

if [ -z "${BRIDGE_EXE:-}" ]; then
    BRIDGE_EXE="$(
        ros2 pkg executables roboracer_autoware_interface 2>/dev/null \
        | awk 'NR==1 {print $2}'
    )"
fi

if [ -z "${LOC_EXE:-}" ]; then
    echo "ERROR: localization adapter executable not found."
    exit 1
fi

if [ -z "${BRIDGE_EXE:-}" ]; then
    echo "ERROR: VESC bridge executable not found."
    exit 1
fi

echo
echo "Localization executable: $LOC_EXE"
echo "VESC bridge executable:  $BRIDGE_EXE"

# ============================================================
# 1. BASE -> LASER STATIC TF
# ============================================================

start_node laser_tf \
    ros2 run tf2_ros static_transform_publisher \
    --x 0.20 \
    --y 0.0 \
    --z 0.15 \
    --roll 0.0 \
    --pitch 0.0 \
    --yaw 0.0 \
    --frame-id base_link \
    --child-frame-id laser

sleep 1

# ============================================================
# 2. HOKUYO
# ============================================================

echo
echo "Starting Hokuyo..."

start_node hokuyo \
    ros2 run urg_node urg_node_driver \
    --ros-args \
    -p ip_address:="$HOKUYO_IP" \
    -p frame_id:=laser \
    -p calibrate_time:=false

echo "Waiting for /scan..."

SCAN_OK=false

for i in $(seq 1 30); do

    if timeout 2 ros2 topic echo --once /scan \
        >/dev/null 2>&1; then

        SCAN_OK=true
        break
    fi

    sleep 0.5
done

# If first connection failed, release the Hokuyo session and retry once.
if [ "$SCAN_OK" != true ]; then

    echo
    echo "First Hokuyo connection failed."
    echo "Restarting driver after Ethernet release delay..."

    if [ -f "$LOG_DIR/hokuyo.log" ]; then
        echo
        echo "===== FIRST HOKUYO LOG ====="
        cat "$LOG_DIR/hokuyo.log"
    fi

    kill -INT "${PIDS[-1]}" 2>/dev/null || true
    sleep 3
    kill -9 "${PIDS[-1]}" 2>/dev/null || true

    pkill -9 -f "urg_node_driver" 2>/dev/null || true

    echo "Waiting 5 seconds for Hokuyo TCP session release..."
    sleep 5

    start_node hokuyo_retry \
        ros2 run urg_node urg_node_driver \
        --ros-args \
        -p ip_address:="$HOKUYO_IP" \
        -p frame_id:=laser \
        -p calibrate_time:=false

    for i in $(seq 1 30); do

        if timeout 2 ros2 topic echo --once /scan \
            >/dev/null 2>&1; then

            SCAN_OK=true
            break
        fi

        sleep 0.5
    done
fi

if [ "$SCAN_OK" != true ]; then

    echo
    echo "ERROR: Hokuyo /scan unavailable."

    echo
    echo "===== HOKUYO LOGS ====="

    cat "$LOG_DIR/hokuyo.log" 2>/dev/null || true
    cat "$LOG_DIR/hokuyo_retry.log" 2>/dev/null || true

    exit 1
fi

echo "/scan OK."

echo
echo "Measuring LiDAR briefly..."

timeout 3 ros2 topic hz /scan || true

# ============================================================
# 3. VESC DRIVER
#
# User-selected ERPM limit:
# +/- 50,000 ERPM
# ============================================================

start_node vesc_driver \
    ros2 run vesc_driver vesc_driver_node \
    --ros-args \
    -p port:="$VESC_PORT" \
    -p duty_cycle_min:=-0.10 \
    -p duty_cycle_max:=0.10 \
    -p speed_min:=-50000.0 \
    -p speed_max:=50000.0 \
    -p servo_min:=0.26 \
    -p servo_max:=0.75

sleep 2

# ============================================================
# 4. RF2O LASER ODOMETRY
# ============================================================

start_node rf2o \
    ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
    --ros-args \
    --params-file "$RF2O_CONFIG"

echo "Waiting for /odom..."

ODOM_OK=false

for i in $(seq 1 30); do

    PUB_COUNT="$(
        ros2 topic info /odom 2>/dev/null |
        awk '/Publisher count:/ {print $3}'
    )"

    if [ "${PUB_COUNT:-0}" -ge 1 ] 2>/dev/null; then
        ODOM_OK=true
        echo "/odom publisher detected."
        break
    fi

    sleep 0.5
done

if [ "$ODOM_OK" != true ]; then
    echo
    echo "ERROR: RF2O /odom publisher not detected."
    echo
    echo "===== RF2O LOG ====="
    cat "$LOG_DIR/rf2o.log" 2>/dev/null || true
    exit 1
fi

echo "Checking /odom data..."

if timeout 5 ros2 topic echo --once /odom >/dev/null 2>&1; then
    echo "/odom data OK."
else
    echo "WARNING: publisher exists but one-shot echo timed out."
    echo "Continuing because RF2O publisher is present."
fi

# ============================================================
# 5. SLAM TOOLBOX
# ============================================================

start_node slam \
    ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args \
    --params-file "$SLAM_CONFIG"

echo "Waiting for /map..."

for i in $(seq 1 40); do

    if timeout 1 ros2 topic echo --once /map \
        >/dev/null 2>&1; then

        echo "/map OK."
        break
    fi

    if [ "$i" -eq 40 ]; then
        echo "ERROR: SLAM /map did not start."
        exit 1
    fi

    sleep 0.5
done

# ============================================================
# 6. LOCALIZATION ADAPTER
# ============================================================

start_node localization \
    ros2 run roboracer_localization_adapter "$LOC_EXE"

echo "Waiting for /localization/kinematic_state..."

for i in $(seq 1 30); do

    if timeout 1 ros2 topic echo --once \
        /localization/kinematic_state \
        >/dev/null 2>&1; then

        echo "/localization/kinematic_state OK."
        break
    fi

    if [ "$i" -eq 30 ]; then
        echo "ERROR: localization output did not start."
        exit 1
    fi

    sleep 0.5
done

# ============================================================
# 7. VESC / AUTOWARE BRIDGE
# ============================================================

start_node vesc_bridge \
    ros2 run roboracer_autoware_interface "$BRIDGE_EXE"

sleep 1

# ============================================================
# 8. RACE GAP DETECTOR
# ============================================================

start_node track_detector \
    ros2 run roboracer_track_detector track_detector

# ============================================================
# 9. RACE CONTROLLER
# ============================================================

start_node track_controller \
    ros2 run roboracer_track_detector track_controller

# ============================================================
# 10. TRACK MEMORY
# ============================================================

start_node lap_detector \
    ros2 run roboracer_track_memory lap_detector

start_node trajectory_recorder \
    ros2 run roboracer_track_memory trajectory_recorder

start_node memory_manager \
    ros2 run roboracer_track_memory memory_manager

start_node trajectory_follower \
    ros2 run roboracer_track_memory trajectory_follower

# ============================================================
# 11. MODE + CONTROLLER SELECTION
# ============================================================

start_node mode_manager \
    ros2 run roboracer_track_memory mode_manager

start_node controller_selector \
    ros2 run roboracer_track_memory controller_selector

# ============================================================
# 12. INDEPENDENT SAFETY
# ============================================================

start_node safety_supervisor \
    ros2 run roboracer_track_memory safety_supervisor

# ============================================================
# 13. FINAL CONTROL ADAPTER
# ============================================================

start_node safe_control_adapter \
    ros2 run roboracer_track_memory safe_control_adapter

sleep 3

# ============================================================
# FORCE DISARM
# ============================================================

disarm

# ============================================================
# HEALTH CHECK
# ============================================================

echo
echo "============================================================"
echo " HEALTH CHECK"
echo "============================================================"

check_topic()
{
    TOPIC="$1"

    if timeout 3 ros2 topic echo --once "$TOPIC" \
        >/dev/null 2>&1; then

        echo "[PASS] $TOPIC"

    else

        echo "[FAIL] $TOPIC"
    fi
}

check_topic /scan
check_topic /odom
check_topic /map
check_topic /localization/kinematic_state
check_topic /roboracer/target_point
check_topic /roboracer/exploration/speed
check_topic /roboracer/exploration/steering
check_topic /roboracer/drive_mode
check_topic /roboracer/automatic_drive_enable
check_topic /roboracer/selected/controller
check_topic /roboracer/safety/state
check_topic /roboracer/safety/speed
check_topic /control/command/control_cmd
check_topic /commands/motor/speed
check_topic /commands/servo/position

echo
echo "========== CURRENT STATE =========="

timeout 3 ros2 topic echo --once \
    /roboracer/drive_mode \
    2>/dev/null || true

timeout 3 ros2 topic echo --once \
    /roboracer/selected/controller \
    2>/dev/null || true

timeout 3 ros2 topic echo --once \
    /roboracer/safety/state \
    2>/dev/null || true

timeout 3 ros2 topic echo --once \
    /roboracer/safety/front_distance \
    2>/dev/null || true

echo
echo "========== FINAL AUTOWARE COMMAND =========="

timeout 3 ros2 topic echo --once \
    /control/command/control_cmd \
    2>/dev/null || true

echo
echo "========== MOTOR COMMAND =========="

timeout 3 ros2 topic echo --once \
    /commands/motor/speed \
    2>/dev/null || true

echo
echo "============================================================"
echo " SYSTEM STARTED -- VEHICLE IS DISARMED"
echo "============================================================"
echo
echo "FIRST PHYSICAL TEST:"
echo "  1. Wheels OFF ground."
echo "  2. Confirm steering has room to move."
echo "  3. Hokuyo unobstructed."
echo "  4. Be ready to type DISARM."
echo
echo "Commands:"
echo
echo "  STATUS   show current controller/safety/motor status"
echo "  ARM      enable physical autonomous drive"
echo "  DISARM   immediately disable motor"
echo "  QUIT     stop everything"
echo

# ============================================================
# COMMAND LOOP
# ============================================================

while true; do

    echo
    read -r -p "RoboRacer [STATUS/ARM/DISARM/QUIT] > " CMD

    CMD="$(echo "$CMD" | tr '[:lower:]' '[:upper:]')"

    case "$CMD" in

        ARM)

            echo
            echo "============================================"
            echo " ARM REQUEST"
            echo "============================================"

            echo "Checking safety state..."

            SAFETY="$(
                timeout 3 ros2 topic echo --once \
                /roboracer/safety/state \
                --field data \
                2>/dev/null \
                | head -n1
            )"

            MODE="$(
                timeout 3 ros2 topic echo --once \
                /roboracer/drive_mode \
                --field data \
                2>/dev/null \
                | head -n1
            )"

            AUTO="$(
                timeout 3 ros2 topic echo --once \
                /roboracer/automatic_drive_enable \
                --field data \
                2>/dev/null \
                | head -n1
            )"

            echo "Safety: $SAFETY"
            echo "Mode:   $MODE"
            echo "Auto:   $AUTO"

            if [ "$SAFETY" != "CLEAR" ] && \
               [ "$SAFETY" != "SLOW" ]; then

                echo
                echo "ARM REFUSED."
                echo "Safety state is not CLEAR/SLOW."
                continue
            fi

            if [ "$AUTO" != "true" ]; then

                echo
                echo "ARM REFUSED."
                echo "Automatic drive permission is not true."
                continue
            fi

            echo
            echo "Type exactly:"
            echo
            echo "ARM PHYSICAL CAR"
            echo

            read -r CONFIRM

            if [ "$CONFIRM" != "ARM PHYSICAL CAR" ]; then

                echo "Arm cancelled."
                continue
            fi

            ros2 topic pub --once \
                /roboracer/drive_enable \
                std_msgs/msg/Bool \
                "{data: true}"

            echo
            echo "******** VEHICLE ARMED ********"
            echo "Type DISARM immediately to stop."
            ;;

        DISARM)

            disarm
            ;;

        STATUS)

            echo
            echo "========== MODE =========="
            timeout 2 ros2 topic echo --once \
                /roboracer/drive_mode \
                2>/dev/null || true

            echo "========== CONTROLLER =========="
            timeout 2 ros2 topic echo --once \
                /roboracer/selected/controller \
                2>/dev/null || true

            echo "========== SAFETY =========="
            timeout 2 ros2 topic echo --once \
                /roboracer/safety/state \
                2>/dev/null || true

            echo "========== FRONT DISTANCE =========="
            timeout 2 ros2 topic echo --once \
                /roboracer/safety/front_distance \
                2>/dev/null || true

            echo "========== SELECTED SPEED =========="
            timeout 2 ros2 topic echo --once \
                /roboracer/selected/speed \
                2>/dev/null || true

            echo "========== SAFE SPEED =========="
            timeout 2 ros2 topic echo --once \
                /roboracer/safety/speed \
                2>/dev/null || true

            echo "========== CONTROL =========="
            timeout 2 ros2 topic echo --once \
                /control/command/control_cmd \
                2>/dev/null || true

            echo "========== MOTOR ERPM =========="
            timeout 2 ros2 topic echo --once \
                /commands/motor/speed \
                2>/dev/null || true
            ;;

        QUIT)

            exit 0
            ;;

        *)

            echo "Use STATUS, ARM, DISARM, or QUIT."
            ;;

    esac
done
