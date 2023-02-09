import time
from concurrent.futures import ThreadPoolExecutor

import zmq
from anx_interface.proto import assets_pb2, device_pb2, common_pb2

class DeviceImu:
    def __init__(self, executor):
        self.executor = executor

        self.ctx = zmq.Context()

        self.socket = self.ctx.socket(zmq.SUB)
        self.poller = zmq.Poller()

        self.port = None
        self.cb = None
        self.data = None
        self.active = False

    def start(self, port):
        if self.active:
            return False

        self.active = True
        self.port = port
        self.socket.connect(f"tcp://localhost:{self.port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.poller.register(self.socket, zmq.POLLIN)

        self.executor.submit(self.data_thread)
        return True

    def stop(self):
        if not self.active:
            return True

        self.active = False
        self.socket.disconnect(f"tcp://localhost:{self.port}")
        self.poller.unregister(self.socket)
        return True

    def get_last_data(self):
        return self.data

    def register_cb(self, cb):
        self.cb = cb

    def data_thread(self):
        while self.active:
            events = self.poller.poll(100)
            if events:
                msg_bytes = self.socket.recv()
                msg = asset_pb2.ImuData()
                msg.ParseFromString(msg_bytes)
                self.imu_data = imu_data
                if self.cb is not None:
                    self.cb(msg)
