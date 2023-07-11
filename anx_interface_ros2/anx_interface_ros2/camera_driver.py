#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from anx_interface import Anx

class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")

        self.anx = Anx()
        self.bridge = CvBridge()

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

    def start(self):
        self.start_stream(
            width=self.width,
            height=self.height,
            fps=self.fps,
            pixel_format=self.pixel_format
        )
        
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

            compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
            compressed_img_msg.header.frame_id = "device_camera"

            self.compressed_img_pub.publish(compressed_img_msg)

    def wait(self):
        self.anx.wait() # blocks till interrupt is received
        self.running = False
        self.display_thread.join()
 
def main(args=None): 
    rclpy.init(args=args)
    camera_driver = CameraDriver()
    camera_driver.start()
    camera_driver.wait()

if __name__ == "__main__": 
    main()

