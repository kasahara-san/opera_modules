#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import Float64

from com3_msgs.action import SetDumpAngle


class SoilReleaseModule(Node):

    def __init__(self):
        super().__init__('soil_release_module')

        # Publisher: /ic120/vessel/cmd (std_msgs/Float64)
        self._pub = self.create_publisher(
            Float64,
            '/ic120/vessel/cmd',
            qos_profile=10
        )

        # Action server: "set_dump_angle"
        self._action_server = ActionServer(
            self,
            SetDumpAngle,
            'set_dump_angle',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('SoilReleaseModule action server ready')

    def goal_callback(self, goal_request):
        """ゴール受信時に実行。常に受け入れて即実行."""
        self.get_logger().info(
            f'Received request to set angle to {goal_request.target_angle:.2f} rad'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """キャンセル要求を許可."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        ゴールを実行:
          1) 0→target ランプアップ (0.1 rad/s)
          2) 3秒ホールド
          3) target→0 ランプダウン (0.1 rad/s)
          4) 結果(success=True)返却
        """
        target = goal_handle.request.target_angle
        rate = 0.1      # rad/sec
        period = 1.0    # sec between publishes

        msg = Float64()
        current = 0.0

        # 1) ramp up
        while rclpy.ok() and current < target:
            current += rate
            if current > target:
                current = target
            msg.data = current
            self._pub.publish(msg)
            time.sleep(period)

        # 2) hold at target
        time.sleep(3.0)

        # 3) ramp down
        while rclpy.ok() and current > 0.0:
            current -= rate
            if current < 0.0:
                current = 0.0
            msg.data = current
            self._pub.publish(msg)
            time.sleep(period)

        # 4) finish
        result = SetDumpAngle.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info('Completed set_dump_angle action')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SoilReleaseModule()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
