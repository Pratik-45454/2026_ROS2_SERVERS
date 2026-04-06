#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time
import os


class ImuLogger(Node):
    def __init__(self):
        super().__init__('logger')

        self.subscription = self.create_subscription(
            Float32,
            '/imu_data',
            self.callback,
            10
        )

        self.filename = 'imu_data_log.txt'
        file_exists = os.path.exists(self.filename)

        # 'a' = append, creates file if not present
        self.file = open(self.filename, 'a')

        # Optional header (only once)
        if not file_exists:
            self.file.write("# timestamp_sec, imu_deg\n")

        self.get_logger().info(
            f'Appending /imu_data to {self.filename} (rad → deg)'
        )

    def callback(self, msg: Float32):
        rad = msg.data
        deg = math.degrees(rad)

        timestamp = time.time()  # unix time in seconds
        self.file.write(f"{timestamp:.6f}, {deg:.6f}\n")
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
