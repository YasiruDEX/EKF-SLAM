#!/usr/bin/env python3

import math
import sys
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class TeleopToggleSmooth(Node):
    def __init__(self) -> None:
        super().__init__('teleop_toggle_smooth')

        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('max_linear_accel', 0.3)
        self.declare_parameter('max_linear_decel', 0.3)
        self.declare_parameter('max_angular_accel', 0.6)
        self.declare_parameter('max_angular_decel', 0.6)

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self._max_linear_decel = float(self.get_parameter('max_linear_decel').value)
        self._max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self._max_angular_decel = float(self.get_parameter('max_angular_decel').value)

        self._target = Twist()
        self._current = Twist()
        self._last_update_time: Optional[rclpy.time.Time] = None

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        period = 1.0 / max(self._rate_hz, 1.0)
        self.create_timer(period, self._on_timer)

        self._settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info(self._help_text())

    def destroy_node(self) -> bool:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return super().destroy_node()

    def _on_timer(self) -> None:
        key = self._get_key()
        if key:
            self._handle_key(key)

        now = self.get_clock().now()
        if self._last_update_time is None:
            self._last_update_time = now
            return

        dt = (now - self._last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

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

    def _handle_key(self, key: str) -> None:
        if key == 'i':
            self._target.linear.x = self._linear_speed
        elif key == ',':
            self._target.linear.x = -self._linear_speed
        elif key == 'j':
            self._target.angular.z = self._angular_speed
        elif key == 'l':
            self._target.angular.z = -self._angular_speed
        elif key == 'k':
            self._target = Twist()
        elif key == 'q':
            self._target.linear.x = self._linear_speed
            self._target.angular.z = self._angular_speed
        elif key == 'e':
            self._target.linear.x = self._linear_speed
            self._target.angular.z = -self._angular_speed
        elif key == 'z':
            self._target.linear.x = -self._linear_speed
            self._target.angular.z = self._angular_speed
        elif key == 'c':
            self._target.linear.x = -self._linear_speed
            self._target.angular.z = -self._angular_speed
        elif key == ' ':
            self._target = Twist()
        elif key == '\x03':
            raise KeyboardInterrupt

    def _get_key(self) -> str:
        return sys.stdin.read(1) if self._key_available() else ''

    @staticmethod
    def _key_available() -> bool:
        import select

        return select.select([sys.stdin], [], [], 0.0)[0] != []

    @staticmethod
    def _step(current: float, target: float, dt: float, accel: float, decel: float) -> float:
        if math.isclose(current, target, abs_tol=1e-6):
            return target
        limit = accel if target > current else decel
        delta = limit * dt
        if target > current:
            return min(current + delta, target)
        return max(current - delta, target)

    @staticmethod
    def _help_text() -> str:
        return (
            "Teleop toggle (press once to keep moving):\n"
            "  i: forward    ,: backward\n"
            "  j: left turn  l: right turn\n"
            "  q/e/z/c: diagonals\n"
            "  k or space: stop\n"
            "  Ctrl-C: quit"
        )


def main() -> None:
    rclpy.init()
    node = TeleopToggleSmooth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
