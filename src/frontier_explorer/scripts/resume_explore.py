#!/usr/bin/env python3
"""Nudge explore_lite back to life if it stops while the map is still growing.

This build of explore_lite calls stop() and cancels its timer the first time
a frontier search returns empty -- which also happens on a transient bad
read (a costmap mid-rebuild, a a scan gap), not only on genuine completion.
explore_lite listens on ``explore/resume`` (std_msgs/Bool) and re-arms on
``true``.

Strategy: watch ``/map``.  Every ``period`` seconds, compare the known-cell
count to the previous check.
  * still growing  -> exploration is working, do nothing.
  * flat for two checks running -> either done or wedged; publish one
    ``explore/resume``.  If it was really done, explore_lite just re-checks,
    finds nothing and stops again (robot is already home) -- harmless.

Gives up after ``max_resumes`` nudges so a truly-finished run goes quiet.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid


class ResumeExplore(Node):
    def __init__(self):
        super().__init__('resume_explore')
        self.declare_parameter('period', 20.0)
        self.declare_parameter('max_resumes', 15)
        self.declare_parameter('grow_threshold', 30)   # known cells
        period = float(self.get_parameter('period').value)
        self._max = int(self.get_parameter('max_resumes').value)
        self._grow = int(self.get_parameter('grow_threshold').value)

        self._pub = self.create_publisher(Bool, 'explore/resume', 1)
        self._known = None
        self._flat_streak = 0
        self._resumes = 0

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'watchdog up: check every {period:.0f}s, up to {self._max} resumes')

    def _on_map(self, msg):
        self._latest_known = sum(1 for v in msg.data if v >= 0)

    def _tick(self):
        known = getattr(self, '_latest_known', None)
        if known is None:
            return

        if self._known is None:
            self._known = known
            return

        grew = known - self._known
        self._known = known

        if grew > self._grow:
            self._flat_streak = 0
            self.get_logger().info(f'map +{grew} cells, exploration progressing')
            return

        self._flat_streak += 1
        if self._flat_streak < 2:
            return

        if self._resumes >= self._max:
            self.get_logger().info(
                f'map flat and {self._max} resumes spent -> assuming done, exiting')
            raise SystemExit(0)

        self._resumes += 1
        self._flat_streak = 0
        self.get_logger().warn(
            f'map flat 2 checks -> publishing explore/resume ({self._resumes}/{self._max})')
        self._pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = ResumeExplore()
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
