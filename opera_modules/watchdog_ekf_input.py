#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


def euclid(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class WatchdogEkfInput(Node):
    def __init__(self):
        super().__init__('watchdog_ekf_input')

        self.gnss_topic = self.declare_parameter('gnss_topic', '/zx200/gnss_odom_relay').get_parameter_value().string_value
        self.encoder_topic = self.declare_parameter('encoder_topic', '/zx200/odom_pose').get_parameter_value().string_value
        self.max_interval_sec = float(self.declare_parameter('max_interval_sec', 1.0).value)
        self.topic_absence_sec = float(self.declare_parameter('topic_absence_sec', 3.0).value)
        self.jump_thr_m = float(self.declare_parameter('jump_distance_threshold', 5.0).value)
        self.publish_on_change_only = bool(self.declare_parameter('publish_on_change_only', False).value)
        self.check_rate_hz = float(self.declare_parameter('check_rate_hz', 10.0).value)
        
        if self.check_rate_hz <= 0.0:
            self.get_logger().warn('check_rate_hz <= 0, forcing to 10.0')
            self.check_rate_hz = 10.0

        # ------- Pub -------
        self.emergency_pub = self.create_publisher(Bool, '/zx200/emg_stop_cmd', 10)

        # ------- State -------
        self.emergency = False
        self.last_gnss_time: Optional[Time] = None
        self.last_enc_time: Optional[Time] = None
        self.prev_gnss_xyz: Optional[tuple] = None
        now_t = self.get_clock().now()
        self.last_gnss_pub_seen: Time = now_t
        self.last_enc_pub_seen: Time = now_t

        self._last_warn: Dict[str, Time] = {}

        # ------- Subscriptions -------
        qos = qos_profile_sensor_data
        self.sub_gnss = self.create_subscription(Odometry, self.gnss_topic, self._gnss_cb_odom, qos)
        self.sub_enc  = self.create_subscription(Odometry, self.encoder_topic, self._enc_cb_odom, qos)

        # ------- Timer -------
        period_s = 1.0 / self.check_rate_hz
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            f'Started WatchdogEkfInput. '
            f'GNSS[{self.gnss_topic}:Odometry] ENC[{self.encoder_topic}:Odometry] '
            f'max_interval={self.max_interval_sec:.3f}s jump_thr={self.jump_thr_m:.2f}m '
            f'topic_absence_sec={self.topic_absence_sec:.3f}s'
        )

    # ------- Utils -------
    def _msg_time(self, header_stamp) -> Optional[Time]:
        if header_stamp is None:
            return None
        return Time.from_msg(header_stamp)

    def _touch_time(self, slot_name: str, header_time_opt: Optional[Time]):
        now_t = self.get_clock().now()
        setattr(self, slot_name, header_time_opt if header_time_opt is not None else now_t)

    def _throttled_warn(self, key: str, text: str, period_sec: float = 2.0):
        now_t = self.get_clock().now()
        last = self._last_warn.get(key)
        if last is None or (now_t - last).nanoseconds / 1e9 >= period_sec:
            self.get_logger().warn(text)
            self._last_warn[key] = now_t

    def _set_emergency(self, new_state: bool, reason: str):
        if self.publish_on_change_only and new_state == self.emergency:
            return
        self.emergency = new_state
        self.emergency_pub.publish(Bool(data=self.emergency))
        self.get_logger().info(f'Emergency={"TRUE" if self.emergency else "false"} ({reason})')

    # ------- GNSS (Odometry) -------
    def _gnss_cb_odom(self, msg: Odometry):
        t = self._msg_time(msg.header.stamp)
        self._touch_time('last_gnss_time', t)
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

        # ジャンプ検知
        if self.prev_gnss_xyz is not None:
            if euclid(p, self.prev_gnss_xyz) > self.jump_thr_m:
                self._set_emergency(True, 'GNSS jump (Odometry)')
        self.prev_gnss_xyz = p

    # ------- Encoder (Odometry) -------
    def _enc_cb_odom(self, msg: Odometry):
        t = self._msg_time(msg.header.stamp)
        self._touch_time('last_enc_time', t)

    # ------- Timer -------
    def _on_timer(self):
        now_t = self.get_clock().now()
        violation = False

        # A) トピック存在監視（Publisher数）
        gnss_pub_count = self.count_publishers(self.gnss_topic)
        if gnss_pub_count > 0:
            self.last_gnss_pub_seen = now_t
        else:
            absent = (now_t - self.last_gnss_pub_seen).nanoseconds / 1e9
            if absent > self.topic_absence_sec:
                violation = True
                self._throttled_warn(
                    'gnss_topic_absent',
                    f'GNSS topic absent: no publishers for {absent:.3f}s > {self.topic_absence_sec:.3f}s'
                )

        enc_pub_count = self.count_publishers(self.encoder_topic)
        if enc_pub_count > 0:
            self.last_enc_pub_seen = now_t
        else:
            absent = (now_t - self.last_enc_pub_seen).nanoseconds / 1e9
            if absent > self.topic_absence_sec:
                violation = True
                self._throttled_warn(
                    'enc_topic_absent',
                    f'Encoder topic absent: no publishers for {absent:.3f}s > {self.topic_absence_sec:.3f}s'
                )

        # B) メッセージ未受信タイムアウト
        if self.last_gnss_time is not None:
            dt = (now_t - self.last_gnss_time).nanoseconds / 1e9
            if dt > self.max_interval_sec:
                violation = True
                self._throttled_warn('gnss_timeout', f'GNSS timeout: {dt:.3f}s > {self.max_interval_sec:.3f}s')
        if self.last_enc_time is not None:
            dt = (now_t - self.last_enc_time).nanoseconds / 1e9
            if dt > self.max_interval_sec:
                violation = True
                self._throttled_warn('enc_timeout', f'Encoder timeout: {dt:.3f}s > {self.max_interval_sec:.3f}s')

        # 出力
        self._set_emergency(violation, 'Violation' if violation else 'OK')


def main():
    rclpy.init()
    node = WatchdogEkfInput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
