#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from anx_interface import Anx

class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")

        self.anx = Anx()

        # publishers
        self.compressed_img_pub = self.create_publisher(
            CompressedImage,
            "/device_camera/image_raw/compressed",
            10
        )

        # declare parameters
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("pixel_format", 0)

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.pixel_format = self.get_parameter('pixel_format').get_parameter_value().integer_value

        self.running = False
        self.display_thread = None
        
    def start_stream(self, width, height, fps, pixel_format):
        self.anx.start_device_camera(fps=fps, width=width, height=height, pixel_format=pixel_format)
        self.running = True
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.start()

    def display_loop(self):
        while self.running:
            ret, img = self.anx.device_camera.read() 
            if not ret: 
                continue 
            compressed_img_msg = CompressedImage()
            compressed_img_msg.header.frame_id = "device_camera"
            compressed_img_msg.header.stamp = self.get_clock().now().to_msg()

            compressed_img_msg.format = "jpeg"
            compressed_img_msg.data = img

            self.compressed_img_pub.publish(compressed_img_msg)

    def wait(self):
        self.anx.wait() # blocks till interrupt is received
        self.running = False
        self.display_thread.join()
 
def main(args=None): 
    rclpy.init(args=args)
    camera_driver = CameraDriver()
    camera_driver.start_stream(fps=30, width=640, height=480, pixel_format=0)
    camera_driver.wait()

if __name__ == "__main__": 
    main()

