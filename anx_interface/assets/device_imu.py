from concurrent.futures import ThreadPoolExecutor

import zmq

import anx_proto.python.assets_pb2 as assets_pb2
import anx_proto.python.device_pb2 as device_pb2
import anx_proto.python.common_pb2 as common_pb2

class DeviceImu:
    def __init__(self, executor):
        self._executor = executor

        self._ctx = zmq.Context()

        self._socket = self._ctx.socket(zmq.SUB)
        self._poller = zmq.Poller()

        self._cb = None
        self.data = None
        self._active = False

    def _start(self, cb=None):
        if self._active:
            return False

        self._active = True
        self._cb = cb
        self._socket.connect("ipc:///ipc/device_imu")
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._poller.register(self._socket, zmq.POLLIN)

        self._executor.submit(self._data_thread)
        return True

    def _stop(self):
        if not self._active:
            return True

        self._active = False
        self._socket.disconnect("ipc:///ipc/device_imu")
        self._poller.unregister(self._socket)
        self._cb = None
        self.data = None
        return True

    def _data_thread(self):
        while self._active:
            events = self._poller.poll(100)
            if events:
                msg_bytes = self._socket.recv()
                msg = assets_pb2.ImuData()
                msg.ParseFromString(msg_bytes)
                self.data = msg
                if self._cb is not None:
                    self._cb(self.data)
