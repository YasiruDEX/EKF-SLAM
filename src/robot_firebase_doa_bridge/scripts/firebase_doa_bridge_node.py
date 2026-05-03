#!/usr/bin/python3

import json
import math
import time
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from rclpy.node import Node

from robot_rotate_goal.srv import RotateToYaw


class FirebaseDoaBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('firebase_doa_bridge_node')

        self.declare_parameter('firebase_base_url', 'https://classroom-bot-a7454-default-rtdb.asia-southeast1.firebasedatabase.app')
        self.declare_parameter('mic_data_path', 'mic_data')
        self.declare_parameter('firebase_auth_token', '')
        self.declare_parameter('poll_interval_sec', 1.0)
        self.declare_parameter('request_timeout_sec', 2.0)
        self.declare_parameter('service_name', '/rotate_to_yaw')
        self.declare_parameter('min_call_period_sec', 2.0)
        self.declare_parameter('doa_offset_deg', 0.0)
        self.declare_parameter('log_data_updates', True)

        self.firebase_base_url = str(self.get_parameter('firebase_base_url').value).rstrip('/')
        self.mic_data_path = str(self.get_parameter('mic_data_path').value).strip('/')
        self.firebase_auth_token = str(self.get_parameter('firebase_auth_token').value)
        self.poll_interval_sec = float(self.get_parameter('poll_interval_sec').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.service_name = str(self.get_parameter('service_name').value)
        self.min_call_period_sec = float(self.get_parameter('min_call_period_sec').value)
        self.doa_offset_deg = float(self.get_parameter('doa_offset_deg').value)
        self.log_data_updates = bool(self.get_parameter('log_data_updates').value)

        self.rotate_client = self.create_client(RotateToYaw, self.service_name)
        self.pending_future = None
        self.last_call_time = 0.0
        self.last_seen_signature = None

        self.timer = self.create_timer(self.poll_interval_sec, self._poll_and_command)

        self.get_logger().info(
            f'Firebase DOA bridge started. Reading {self.firebase_base_url}/{self.mic_data_path}.json '
            f'and calling {self.service_name} when is_voice_hw is true.'
        )

    def _mic_data_url(self) -> str:
        url = f'{self.firebase_base_url}/{self.mic_data_path}.json'
        if self.firebase_auth_token:
            query = urllib.parse.urlencode({'auth': self.firebase_auth_token})
            return f'{url}?{query}'
        return url

    @staticmethod
    def _normalize_degrees(value: float) -> float:
        normalized = math.fmod(value, 360.0)
        if normalized < 0.0:
            normalized += 360.0
        return normalized

    def _fetch_mic_data(self):
        request = urllib.request.Request(self._mic_data_url(), method='GET')
        with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
            if response.status != 200:
                raise RuntimeError(f'Firebase returned HTTP {response.status}')
            payload = response.read().decode('utf-8')
            return json.loads(payload)

    @staticmethod
    def _to_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            lowered = value.strip().lower()
            return lowered in ('true', '1', 'yes', 'y', 'on')
        return False

    def _poll_and_command(self) -> None:
        if self.pending_future is not None and not self.pending_future.done():
            return

        try:
            mic_data = self._fetch_mic_data()
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, RuntimeError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Failed to fetch mic_data from Firebase: {exc}')
            return

        if not isinstance(mic_data, dict):
            self.get_logger().warn('mic_data payload is not an object.')
            return

        doa_value = mic_data.get('doa', None)
        try:
            doa_deg = float(doa_value)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid DOA value in Firebase: {doa_value}')
            return

        is_voice_hw = self._to_bool(mic_data.get('is_voice_hw', False))
        data_signature = (round(doa_deg, 3), is_voice_hw, mic_data.get('timestamp', None))
        if self.log_data_updates and data_signature != self.last_seen_signature:
            self.get_logger().info(
                f'Firebase update: doa={doa_deg:.1f} deg, is_voice_hw={is_voice_hw}, '
                f'timestamp={mic_data.get("timestamp", "n/a")}'
            )
        self.last_seen_signature = data_signature

        if not is_voice_hw:
            return

        if not self.rotate_client.service_is_ready():
            self.rotate_client.wait_for_service(timeout_sec=0.0)
            self.get_logger().warn(
                f'{self.service_name} not available yet. Skipping command for doa={doa_deg:.1f} deg.'
            )
            return

        target_heading_deg = self._normalize_degrees(doa_deg + self.doa_offset_deg)

        now = time.monotonic()
        if now - self.last_call_time < self.min_call_period_sec:
            return

        request = RotateToYaw.Request()
        request.heading_deg = target_heading_deg

        self.pending_future = self.rotate_client.call_async(request)
        self.pending_future.add_done_callback(self._handle_service_response)
        self.last_call_time = now

        self.get_logger().info(
            f'is_voice_hw=true, doa={doa_deg:.1f} deg -> heading={target_heading_deg:.1f} deg. '
            f'Calling {self.service_name}.'
        )

    def _handle_service_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'rotate_to_yaw service call failed: {exc}')
            return

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FirebaseDoaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
