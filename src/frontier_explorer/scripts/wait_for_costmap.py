#!/usr/bin/env python3
"""Block until the Nav2 global costmap holds real free space, then exit 0.

explore_lite (this build) is one-shot: the first time its frontier search
returns an empty list it calls stop() and never resumes.  For the first
second or two after Nav2 activates, ``/global_costmap/costmap`` is published
but still all-unknown -- an explore_lite started into that window quits
immediately and the robot never moves.

This gate subscribes to the costmap (latched, transient-local QoS) and exits
0 once it sees a grid with at least ``--min-free-cells`` free cells, or after
``TIMEOUT_SEC`` (exit 0 anyway so the stack still comes up and the problem is
visible in the logs).
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

TIMEOUT_SEC = 90.0
MIN_FREE_CELLS = 200


class CostmapWaiter(Node):
    def __init__(self):
        super().__init__('wait_for_costmap')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('min_free_cells', MIN_FREE_CELLS)
        topic = self.get_parameter('costmap_topic').value
        self._min_free = int(self.get_parameter('min_free_cells').value)

        # Nav2 costmaps publish with transient-local (latched) durability.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, topic, self._on_costmap, qos)

        self._start = time.monotonic()
        self._topic = topic
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'waiting for free space on {topic}')

    def _on_costmap(self, msg):
        free = sum(1 for v in msg.data if 0 <= v < 50)
        elapsed = time.monotonic() - self._start
        if free >= self._min_free:
            self.get_logger().info(
                f'{self._topic} has {free} free cells -> ready ({elapsed:.0f}s)')
            raise SystemExit(0)
        self.get_logger().info(
            f'{self._topic}: {free} free cells so far (need {self._min_free})')

    def _tick(self):
        if time.monotonic() - self._start > TIMEOUT_SEC:
            self.get_logger().warn('timed out waiting for costmap; continuing anyway')
            raise SystemExit(0)


def main():
    rclpy.init()
    node = CostmapWaiter()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code) if exc.code is not None else 0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
