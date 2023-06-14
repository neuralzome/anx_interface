#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from anx_interface import Anx

class ImuDriver(Node):
    def __init__(self):
        super().__init__("imu_driver")

        self.anx = Anx()

        # declare params
        self.declare_parameter("hz", 30)
        self.imu_hz = self.get_parameter('hz').get_parameter_value().integer_value
        self.imu_pub = self.create_publisher(
            Imu,
            "imu",
            10
        )

        self.anx.start_device_imu(fps=self.imu_hz, cb=self.imu_cb)

    def imu_cb(self, imu_data):
        self.imu_time = self.get_clock().now()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu"

        msg.orientation.w = imu_data.filtered.orientation.w
        msg.orientation.x = imu_data.filtered.orientation.x
        msg.orientation.y = imu_data.filtered.orientation.y
        msg.orientation.z = imu_data.filtered.orientation.z

        msg.angular_velocity.x = imu_data.filtered.angular_velocity.x
        msg.angular_velocity.y = imu_data.filtered.angular_velocity.y
        msg.angular_velocity.z = imu_data.filtered.angular_velocity.z

        msg.linear_acceleration.x = imu_data.filtered.linear_acceleration.x
        msg.linear_acceleration.y = imu_data.filtered.linear_acceleration.y
        msg.linear_acceleration.z = imu_data.filtered.linear_acceleration.z

        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_driver = ImuDriver()
    imu_driver.anx.wait()

if __name__ == "__main__":
    main()