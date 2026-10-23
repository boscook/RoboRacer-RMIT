#!/usr/bin/env python3
"""Save the latest /map OccupancyGrid as map.pgm + map.yaml (no SLAM service)."""
import sys

import numpy as np


def write_map(base, width, height, resolution, ox, oy, data):
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full((height, width), 205, dtype=np.uint8)
    img[(grid >= 0) & (grid < 25)] = 254
    img[grid >= 65] = 0
    img = np.flipud(img)
    with open(base + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (width, height))
        f.write(img.tobytes())
    name = base.rsplit('/', 1)[-1]
    with open(base + '.yaml', 'w') as f:
        f.write(
            f'image: {name}.pgm\nmode: trinary\n'
            f'resolution: {resolution}\n'
            f'origin: [{ox}, {oy}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def main():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    base = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('save_map_topic')
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', got.append, qos)
    end = node.get_clock().now().nanoseconds + 8_000_000_000
    while not got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        print('no /map received')
        sys.exit(1)
    m = got[0]
    write_map(base, m.info.width, m.info.height, m.info.resolution,
              m.info.origin.position.x, m.info.origin.position.y, m.data)
    print(f'saved {m.info.width}x{m.info.height} map RESULT=0 result=0')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
