#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import threading


class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay_gui')

        # self.declare_parameter('input_topic', '/gnss_odom')
        # self.declare_parameter('output_topic', '/gnss_odom_relay')

        self.declare_parameter('input_topic', '/zx200/gnss_odom')
        self.declare_parameter('output_topic', '/zx200/gnss_odom_relay')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.active = True

        gui_thread = threading.Thread(target=self.run_gui, daemon=True)
        gui_thread.start()

        self.get_logger().info(f"Relaying {input_topic} → {output_topic} (GUIでON/OFF切替可能)")

    def listener_callback(self, msg: Odometry):
        if self.active:
            self.publisher.publish(msg)

    def run_gui(self):
        while rclpy.ok():
            # zenity のダイアログを呼び出し
            # OKなら publish ON, Cancelなら OFF
            cmd = ["zenity", "--question", "--title=Topic Relay", "--text=Publish Odometry?"]
            ret = subprocess.call(cmd)

            if ret == 0:  # OK押した
                self.get_logger().info("Publish ENABLED")
                self.active = True
            else:  # キャンセル押した
                self.get_logger().info("Publish DISABLED")
                self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
