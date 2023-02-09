#!/usr/bin/env python3

from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import zmq
from anx_mock.proto import assets_pb2
from anx_mock.utils import Rate

class DeviceCamera:
    def __init__(self, executor):
        self.executor = executor
        self.active = False
        self.frame_index = None
        self.fps = None
        self.width = None
        self.height = None
        self.pixel_format = None
        self.port = None

        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)

    def start(self, fps, width, height, pixel_format, port):
        if self.active:
            if not self.stop():
                return True
            else:
                return False

        self.active = True
        self.frame_index = 0
        self.fps = fps
        self.width = width
        self.height = height
        self.pixel_format = pixel_format
        self.port = port
        self.socket.bind(f"tcp://*:{self.port}")

        self.executor.submit(self.data_thread)
        return True

    def stop(self):
        if not self.active:
            return True

        self.active = False
        self.socket.unbind(f"tcp://localhost:{self.port}")
        self.frame_index = None
        self.fps = None
        self.width = None
        self.height = None
        self.pixel_format = None
        self.port = None
        return True

    def get_device_camera_stream(self, fps, width, height, pixel_format):
        device_camera_stream = assets_pb2.DeviceCameraStream()
        device_camera_stream.fps = fps
        device_camera_stream.width = width
        device_camera_stream.height = height
        device_camera_stream.pixel_format = pixel_format
        return device_camera_stream

    def get_select(self):
        msg = assets_pb2.DeviceCameraSelect()
        msg.camera_streams.append(self.get_device_camera_stream(
            30, 480, 640,
            assets_pb2.DeviceCameraStream.PixelFormat.MJPEG
        ))
        return msg

    def get_data(self):
        msg = assets_pb2.CameraData()
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        img = cv2.putText(
                img,
                f'{self.frame_index}',
                (self.width//2, self.height//2),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
                cv2.LINE_AA
        )
        
        if self.pixel_format == assets_pb2.DeviceCameraStream.PixelFormat.MJPEG:
            msg.image = bytes(cv2.imencode('.jpg', img)[1])

        return msg

    def data_thread(self):
        rate = Rate(self.fps)
        while self.active:
            self.frame_index += 1
            msg = self.get_data()

            msg_bytes = msg.SerializeToString()
            self.socket.send(msg_bytes)
            rate.sleep()
