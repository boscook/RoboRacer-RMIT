#!/usr/bin/env python3

import json
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32, String, Bool


class MemoryManager(Node):

    EXPLORING = 'EXPLORING'
    BUILDING_MEMORY = 'BUILDING_MEMORY'
    MEMORY_READY = 'MEMORY_READY'
    LEARNING = 'LEARNING'

    def __init__(self):

        super().__init__(
            'roboracer_memory_manager'
        )

        self.state = self.EXPLORING
        self.current_lap = 0

        self.workspace = (
            Path.home() /
            'roboracer_ws'
        )

        self.memory_root = (
            self.workspace /
            'track_memory'
        )

        self.memory_root.mkdir(
            parents=True,
            exist_ok=True
        )

        self.active_track = (
            self.memory_root /
            'active_track'
        )

        self.active_track.mkdir(
            parents=True,
            exist_ok=True
        )

        self.state_pub = self.create_publisher(
            String,
            '/roboracer/memory_state',
            10
        )

        self.ready_pub = self.create_publisher(
            Bool,
            '/roboracer/memory_ready',
            10
        )

        self.lap_sub = self.create_subscription(
            UInt32,
            '/roboracer/lap_count',
            self.lap_callback,
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_state
        )

        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            'RoboRacer Track Memory Manager'
        )

        self.get_logger().info(
            'STATE: EXPLORING'
        )

        self.get_logger().info(
            '================================='
        )

        self.save_status()

    # ==================================================
    # STATE
    # ==================================================

    def set_state(
        self,
        new_state
    ):

        if new_state == self.state:
            return

        old_state = self.state

        self.state = new_state

        self.get_logger().warn(
            f'STATE: '
            f'{old_state} -> {new_state}'
        )

        self.save_status()

        # Publish immediately so mode_manager sees transitions
        # before any blocking map/optimizer work starts.
        self.publish_state()

    def publish_state(self):

        msg = String()
        msg.data = self.state

        self.state_pub.publish(msg)

    # ==================================================
    # LAP EVENT
    # ==================================================

    def lap_callback(
        self,
        msg
    ):

        lap = int(
            msg.data
        )

        if lap <= self.current_lap:
            return

        self.current_lap = lap

        self.get_logger().warn(
            f'LAP {lap} RECEIVED'
        )

        # ------------------------------------------
        # LAP 1:
        # Build first track memory
        # ------------------------------------------

        if (
            lap == 1 and
            self.state == self.EXPLORING
        ):

            self.set_state(
                self.BUILDING_MEMORY
            )

            success = (
                self.build_memory()
            )

            if success:

                self.set_state(
                    self.MEMORY_READY
                )

                ready = Bool()
                ready.data = True

                self.ready_pub.publish(
                    ready
                )

                self.get_logger().warn(
                    '================================='
                )

                self.get_logger().warn(
                    'TRACK MEMORY READY'
                )

                self.get_logger().warn(
                    'Lap 2 may use remembered track'
                )

                self.get_logger().warn(
                    '================================='
                )

            else:

                self.get_logger().error(
                    'TRACK MEMORY BUILD FAILED'
                )

                # Remain in BUILDING_MEMORY.
                # Do NOT falsely announce MEMORY_READY.

        # ------------------------------------------
        # LAP 2+
        # ------------------------------------------

        elif (
            lap >= 2 and
            self.state ==
            self.MEMORY_READY
        ):

            self.set_state(
                self.LEARNING
            )

        self.save_status()

    # ==================================================
    # BUILD MEMORY
    # ==================================================

    def build_memory(self):

        self.get_logger().info(
            'Building track memory...'
        )

        trajectory = (
            self.memory_root /
            'exploration_trajectory.csv'
        )

        # ------------------------------------------
        # Wait briefly for trajectory recorder
        # to finish writing after same Lap 1 event.
        # ------------------------------------------

        for _ in range(20):

            if (
                trajectory.exists() and
                trajectory.stat().st_size > 50
            ):
                break

            time.sleep(0.1)

        if not trajectory.exists():

            self.get_logger().error(
                'Exploration trajectory not found: '
                f'{trajectory}'
            )

            return False

        self.get_logger().info(
            f'Trajectory found: {trajectory}'
        )

        # ------------------------------------------
        # Save SLAM occupancy map
        # ------------------------------------------

        map_base = (
            self.active_track /
            'map'
        )

        save_map_command = [
            'python3',
            str(
                self.workspace / 'src' / 'roboracer_track_memory' /
                'roboracer_track_memory' / 'save_map_topic.py'
            ),
            str(map_base)
        ]

        map_ok = self.run_command(
            save_map_command,
            'SLAM occupancy map'
        )

        # ------------------------------------------
        # Serialize pose graph
        # ------------------------------------------

        posegraph_base = (
            self.active_track /
            'track'
        )

        posegraph_command = [
            'ros2',
            'service',
            'call',
            '/slam_toolbox/serialize_map',
            'slam_toolbox/srv/SerializePoseGraph',
            (
                "{filename: '" +
                str(posegraph_base) +
                "'}"
            )
        ]

        posegraph_ok = False  # skipped: service hangs, not needed

        # ------------------------------------------
        # Verify actual files
        # ------------------------------------------

        map_yaml = (
            self.active_track /
            'map.yaml'
        )

        map_pgm = (
            self.active_track /
            'map.pgm'
        )

        posegraph = (
            self.active_track /
            'track.posegraph'
        )

        posegraph_data = (
            self.active_track /
            'track.data'
        )

        if not (
            map_ok and
            map_yaml.exists() and
            map_pgm.exists()
        ):

            self.get_logger().warn(
                'Occupancy map was not created (not required).'
            )

        if not (
            posegraph_ok and
            posegraph.exists() and
            posegraph_data.exists()
        ):

            self.get_logger().warn(
                'Pose graph not created (not required).'
            )

        # ------------------------------------------
        # Copy exploration trajectory into
        # active track directory
        # ------------------------------------------

        track_trajectory = (
            self.active_track /
            'exploration_trajectory.csv'
        )

        try:

            track_trajectory.write_bytes(
                trajectory.read_bytes()
            )

        except Exception as e:

            self.get_logger().error(
                f'Cannot copy trajectory: {e}'
            )

            return False

        # ------------------------------------------
        # Run map-aware optimizer
        # ------------------------------------------

        optimizer = (
            self.workspace /
            'src' /
            'roboracer_track_memory' /
            'roboracer_track_memory' /
            'scan_trajectory_optimizer.py'
        )

        learned_track = (
            self.active_track /
            'learned_track.csv'
        )

        optimizer_command = [
            'python3',
            str(optimizer),
            str(track_trajectory),
            str(map_yaml),
            '--output',
            str(learned_track)
        ]

        optimizer_ok = self.run_command(
            optimizer_command,
            'trajectory optimizer'
        )

        if not (
            optimizer_ok and
            learned_track.exists() and
            learned_track.stat().st_size > 50
        ):

            self.get_logger().error(
                'Learned trajectory was not created.'
            )

            return False

        self.get_logger().info(
            f'Learned track: {learned_track}'
        )

        return True

    # ==================================================
    # COMMAND
    # ==================================================

    def run_command(
        self,
        command,
        description
    ):

        self.get_logger().info(
            f'Running: {description}'
        )

        try:

            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=120
            )

            if result.stdout:

                for line in (
                    result.stdout.strip().splitlines()
                ):

                    self.get_logger().info(
                        f'[{description}] {line}'
                    )

            if result.stderr:

                for line in (
                    result.stderr.strip().splitlines()
                ):

                    self.get_logger().warn(
                        f'[{description}] {line}'
                    )

            if result.returncode != 0:

                self.get_logger().error(
                    f'{description} failed '
                    f'with code '
                    f'{result.returncode}'
                )

                return False

            # ros2 service call itself can return 0
            # even when the service result says failure.
            if (
                description ==
                'SLAM occupancy map' and
                'result=0' not in
                result.stdout.replace(' ', '')
            ):

                self.get_logger().error(
                    'save_map service did not '
                    'report RESULT_SUCCESS'
                )

                return False

            if (
                description ==
                'SLAM pose graph' and
                'result=0' not in
                result.stdout.replace(' ', '')
            ):

                self.get_logger().error(
                    'serialize_map did not '
                    'report RESULT_SUCCESS'
                )

                return False

            self.get_logger().info(
                f'{description}: OK'
            )

            return True

        except subprocess.TimeoutExpired:

            self.get_logger().error(
                f'{description}: TIMEOUT'
            )

            return False

        except Exception as e:

            self.get_logger().error(
                f'{description}: {e}'
            )

            return False

    # ==================================================
    # STATUS
    # ==================================================

    def save_status(self):

        status = {
            'state':
                self.state,

            'completed_laps':
                self.current_lap,

            'memory_directory':
                str(
                    self.active_track
                ),

            'timestamp':
                time.time()
        }

        path = (
            self.memory_root /
            'status.json'
        )

        path.write_text(
            json.dumps(
                status,
                indent=2
            )
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = MemoryManager()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
