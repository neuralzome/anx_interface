from io import BytesIO
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image
import zmq
from anx_interface.proto import assets_pb2, device_pb2, common_pb2

class DeviceCamera:
    def __init__(self, executor):
        self.executor = executor

        self.ctx = zmq.Context()

        self.socket = self.ctx.socket(zmq.SUB)
        self.poller = zmq.Poller()

        self.port = 10005
        self.cb = None
        self.data = None
        self.active = False

    def start(self, cb=None):
        if self.active:
            return False

        self.active = True
        self.cb = cb
        self.socket.connect(f"tcp://127.0.0.1:{self.port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.poller.register(self.socket, zmq.POLLIN)

        self.executor.submit(self.data_thread)
        return True

    def stop(self):
        if not self.active:
            return True

        self.active = False
        self.socket.disconnect(f"tcp://127.0.0.1:{self.port}")
        self.poller.unregister(self.socket)
        self.cb = None
        self.data = None
        return True

    def data_thread(self):
        while self.active:
            events = self.poller.poll(100)
            if events:
                msg_bytes = self.socket.recv()
                msg = assets_pb2.CameraData()
                msg.ParseFromString(msg_bytes)
                self.data = np.array(Image.open(BytesIO(msg.image)))
                if self.cb is not None:
                    self.cb(self.data)

