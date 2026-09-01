#!/usr/bin/env python3
"""Rotate the robot in place once, then exit.

Frontier exploration deadlocks if the robot starts roughly centered in a
small closed room: the free<->unknown boundary is a near-symmetric ring, so
its centroid is ~the robot's own pose, explore_lite sends a goal inside
Nav2's xy_goal_tolerance, Nav2 reports instant success, the robot never
moves, and slam_toolbox (minimum_travel_distance) never updates the map --
so the next frontier is identical. Nothing breaks the symmetry.

One slow 360 deg turn fixes it: slam_toolbox updates on
minimum_travel_heading, the whole starting room gets mapped, and the
doorway becomes a distinct, clearly-nearest frontier. The launch file runs
this once, after the costmap gate and before explore_lite.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

YAW_RATE = 0.5          # rad/s  (gentle -- keeps scan matching happy)
REVOLUTIONS = 1.0
SETTLE_SEC = 1.0


class InitialSpin(Node):
    def __init__(self):
        super().__init__('initial_spin')
        self.declare_parameter('yaw_rate', YAW_RATE)
        self.declare_parameter('revolutions', REVOLUTIONS)
        self._rate = float(self.get_parameter('yaw_rate').value)
        revs = float(self.get_parameter('revolutions').value)

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._duration = abs(revs) * 2.0 * math.pi / abs(self._rate)
        self._start = None
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'spinning {revs:.1f} rev at {self._rate:.2f} rad/s '
            f'(~{self._duration:.0f}s)')

    def _tick(self):
        now = time.monotonic()
        if self._start is None:
            self._start = now
        elapsed = now - self._start

        cmd = Twist()
        if elapsed < self._duration:
            cmd.angular.z = self._rate
            self._pub.publish(cmd)
            return

        # stop, settle, done
        self._pub.publish(cmd)  # zero twist
        if elapsed < self._duration + SETTLE_SEC:
            return
        self.get_logger().info('initial spin complete')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = InitialSpin()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code) if exc.code is not None else 0
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
