#!/usr/bin/env python3

from pathlib import Path
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, String, Bool


class MemoryManager(Node):

    EXPLORING = 'EXPLORING'
    BUILDING_MEMORY = 'BUILDING_MEMORY'
    MEMORY_READY = 'MEMORY_READY'
    LEARNING = 'LEARNING'

    def __init__(self):
        super().__init__('roboracer_memory_manager')

        self.state = self.EXPLORING
        self.current_lap = 0

        self.root = (
            Path.home() /
            'roboracer_ws' /
            'track_memory'
        )

        self.root.mkdir(
            parents=True,
            exist_ok=True
        )

        self.state_pub = self.create_publisher(
            String,
            '/roboracer/memory_state',
            10
        )

        self.memory_ready_pub = self.create_publisher(
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
            'Track Memory Manager started'
        )

        self.get_logger().info(
            'STATE: EXPLORING'
        )

    def set_state(self, new_state):

        if new_state == self.state:
            return

        old = self.state
        self.state = new_state

        self.get_logger().warn(
            f'MEMORY STATE: {old} -> {new_state}'
        )

        self.save_status()

    def lap_callback(self, msg):

        lap = int(msg.data)

        if lap <= self.current_lap:
            return

        self.current_lap = lap

        self.get_logger().warn(
            f'Received completed lap: {lap}'
        )

        # First completed lap means the car has
        # discovered one complete circuit.
        if (
            lap == 1 and
            self.state == self.EXPLORING
        ):

            self.set_state(
                self.BUILDING_MEMORY
            )

            # Later this state will trigger:
            #
            # 1. save SLAM map
            # 2. serialize pose graph
            # 3. extract track corridor
            # 4. generate centerline
            # 5. generate initial speed profile
            #
            # For now we only prove the state machine.

            self.set_state(
                self.MEMORY_READY
            )

            ready = Bool()
            ready.data = True

            self.memory_ready_pub.publish(
                ready
            )

        elif (
            lap >= 2 and
            self.state == self.MEMORY_READY
        ):

            self.set_state(
                self.LEARNING
            )

        self.save_status()

    def publish_state(self):

        msg = String()
        msg.data = self.state

        self.state_pub.publish(msg)

    def save_status(self):

        status = {
            'state': self.state,
            'completed_laps': self.current_lap,
            'timestamp': time.time()
        }

        path = self.root / 'status.json'

        path.write_text(
            json.dumps(
                status,
                indent=2
            )
        )


def main(args=None):

    rclpy.init(args=args)

    node = MemoryManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
