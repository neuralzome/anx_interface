#!/usr/bin/env python3

from concurrent.futures import ThreadPoolExecutor

import zmq
from anx_mock.proto import assets_pb2
from anx_mock.utils import Rate

class DeviceGnss:
    def __init__(self, executor):
        self.executor = executor
        self.active = False
        self.fps = 1
        self.port = 10004

        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)

    def start(self):
        if self.active:
            return False

        self.active = True
        self.socket.bind(f"tcp://127.0.0.1:{self.port}")

        self.executor.submit(self.data_thread)
        print("\tdevice_gnss started!!")
        return True

    def stop(self):
        if not self.active:
            return True

        self.active = False
        self.socket.unbind(f"tcp://127.0.0.1:{self.port}")
        print("\tdevice_gnss stopped!!")
        return True

    def get_data(self):
        msg = assets_pb2.GnssData()
        msg.nmea = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76"
        return msg

    def data_thread(self):
        rate = Rate(self.fps)
        while self.active:
            msg = self.get_data()

            msg_bytes = msg.SerializeToString()
            self.socket.send(msg_bytes)
            rate.sleep()
