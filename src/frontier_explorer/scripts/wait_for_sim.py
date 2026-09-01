#!/usr/bin/env python3
"""Block until the Gazebo simulation is actually ready, then exit 0.

The single-shot launch file starts slam_toolbox / Nav2 / explore_lite only
*after* this process exits.  Staggering those layers with fixed timers is
fragile: on a cold start Gazebo can take 10-40 s to publish ``/clock``, and
slam_toolbox started before ``/clock`` is ticking hangs silently forever
(it logs "Using solver plugin ..." and then never registers the laser).

Readiness here means all of:
  * sim time is advancing -- ``node.get_clock().now()`` is non-zero and
    increases between checks (rclpy's own ``/clock`` handling drives this
    when ``use_sim_time`` is set, so we do not subscribe to ``/clock``
    ourselves and cannot get the QoS wrong)
  * a ``/scan`` message has arrived
  * the ``odom`` -> ``base_footprint`` TF is available

If ``use_sim_time`` is false we only require ``/scan`` + TF.  If the timeout
is hit we still exit 0 so the rest of the stack comes up and the failure is
at least visible in the logs.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

import tf2_ros
from sensor_msgs.msg import LaserScan

TIMEOUT_SEC = 120.0


class SimWaiter(Node):
    def __init__(self):
        super().__init__('wait_for_sim')
        # 'use_sim_time' is auto-declared by rclpy.
        self._use_sim_time = self.get_parameter('use_sim_time').value

        self._got_scan = False
        self._last_sim_ns = 0

        scan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._start = time.monotonic()
        self.create_timer(0.5, self._check)

    def _on_scan(self, _msg):
        self._got_scan = True

    def _have_tf(self):
        try:
            return self._tf_buffer.can_transform(
                'odom', 'base_footprint', rclpy.time.Time())
        except Exception:
            return False

    def _sim_time_advancing(self):
        if not self._use_sim_time:
            return True
        now_ns = self.get_clock().now().nanoseconds
        advancing = now_ns > 0 and now_ns > self._last_sim_ns
        self._last_sim_ns = now_ns
        return advancing

    def _check(self):
        have_tf = self._have_tf()
        clock_ok = self._sim_time_advancing()
        if clock_ok and self._got_scan and have_tf:
            self.get_logger().info(
                'simulation ready: sim-clock advancing + /scan + odom->base_footprint TF')
            raise SystemExit(0)

        elapsed = time.monotonic() - self._start
        self.get_logger().info(
            f'waiting for sim... clock_ok={clock_ok} scan={self._got_scan} '
            f'tf={have_tf} ({elapsed:.0f}s)',
            throttle_duration_sec=5.0)
        if elapsed > TIMEOUT_SEC:
            self.get_logger().warn(
                'timed out waiting for simulation readiness; continuing anyway')
            raise SystemExit(0)


def main():
    rclpy.init()
    node = SimWaiter()
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
