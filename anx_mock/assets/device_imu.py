#!/usr/bin/env python3

import random
from concurrent.futures import ThreadPoolExecutor

import zmq
from anx_mock.proto import assets_pb2
from anx_mock.utils import Rate

class DeviceImu:
    def __init__(self, executor):
        self.executor = executor
        self.active = False
        self.fps = None
        self.port = None

        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)

    def start(self, fps, port):
        if self.active:
            if not self.stop():
                return False

        self.active = True
        self.fps = fps
        self.port = port
        self.socket.bind(f"tcp://*:{self.port}")

        self.executor.submit(self.data_thread)
        return True

    def stop(self):
        if not self.active:
            return True

        self.active = False
        self.socket.unbind(f"tcp://localhost:{self.port}")
        self.fps = None
        self.port = None
        return True

    def get_select(self):
        msg = assets_pb2.DeviceImuSelect()
        msg.fps.extend([1, 5, 10, 20, 50, 100, 150, 200])
        return msg

    def get_data(self):
        msg = assets_pb2.ImuData()
        msg.filtered.acceleration.x = random.random()
        msg.filtered.acceleration.y = random.random()
        msg.filtered.acceleration.z = random.random()
        msg.filtered.angular_velocity.x = random.random()
        msg.filtered.angular_velocity.y = random.random()
        msg.filtered.angular_velocity.z = random.random()
        msg.filtered.orientation.x = 0;
        msg.filtered.orientation.y = 0;
        msg.filtered.orientation.z = 0;
        msg.filtered.orientation.w = 1;
        msg.raw.acceleration.x = random.random()
        msg.raw.acceleration.y = -10 + random.random()
        msg.raw.acceleration.z = random.random()
        msg.raw.angular_velocity.x = random.random()
        msg.raw.angular_velocity.y = random.random()
        msg.raw.angular_velocity.z = random.random()
        msg.raw.magnetic_field_in_micro_tesla.x = -40 + random.random()
        msg.raw.magnetic_field_in_micro_tesla.x = -20 + random.random()
        msg.raw.magnetic_field_in_micro_tesla.x = random.random()
        return msg


    def data_thread(self):
        rate = Rate(self.fps)
        while self.active:
            msg = self.get_data()

            msg_bytes = msg.SerializeToString()
            self.socket.send(msg_bytes)
            rate.sleep()