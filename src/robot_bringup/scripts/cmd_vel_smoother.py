#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class CmdVelSmoother(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_smoother')

        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('max_linear_accel', 0.3)
        self.declare_parameter('max_linear_decel', 0.3)
        self.declare_parameter('max_angular_accel', 0.6)
        self.declare_parameter('max_angular_decel', 0.6)

        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self._max_linear_decel = float(self.get_parameter('max_linear_decel').value)
        self._max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self._max_angular_decel = float(self.get_parameter('max_angular_decel').value)

        self._target = Twist()
        self._current = Twist()
        self._last_cmd_time: Optional[rclpy.time.Time] = None
        self._last_update_time: Optional[rclpy.time.Time] = None

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._sub = self.create_subscription(Twist, self._input_topic, self._on_cmd, 10)

        period = 1.0 / max(self._rate_hz, 1.0)
        self.create_timer(period, self._on_timer)

    def _on_cmd(self, msg: Twist) -> None:
        self._target = msg
        self._last_cmd_time = self.get_clock().now()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self._last_update_time is None:
            self._last_update_time = now
            return

        dt = (now - self._last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        if self._last_cmd_time is None or (now - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout_sec:
            self._target = Twist()

        self._current.linear.x = self._step(
            self._current.linear.x,
            self._target.linear.x,
            dt,
            self._max_linear_accel,
            self._max_linear_decel,
        )
        self._current.angular.z = self._step(
            self._current.angular.z,
            self._target.angular.z,
            dt,
            self._max_angular_accel,
            self._max_angular_decel,
        )

        self._pub.publish(self._current)
        self._last_update_time = now

    @staticmethod
    def _step(current: float, target: float, dt: float, accel: float, decel: float) -> float:
        if math.isclose(current, target, abs_tol=1e-6):
            return target
        limit = accel if target > current else decel
        delta = limit * dt
        if target > current:
            return min(current + delta, target)
        return max(current - delta, target)


def main() -> None:
    rclpy.init()
    node = CmdVelSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
