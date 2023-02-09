from concurrent.futures import ThreadPoolExecutor

import zmq
from anx_interface.proto import assets_pb2, device_pb2, common_pb2

class DeviceImu:
    def __init__(self, executor):
        self._executor = executor

        self._ctx = zmq.Context()

        self._socket = self._ctx.socket(zmq.SUB)
        self._poller = zmq.Poller()

        self._port = 10003
        self._cb = None
        self.data = None
        self._active = False

    def _start(self, cb=None):
        if self._active:
            return False

        self._active = True
        self._cb = cb
        self._socket.connect(f"tcp://127.0.0.1:{self._port}")
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._poller.register(self._socket, zmq.POLLIN)

        self._executor.submit(self._data_thread)
        return True

    def _stop(self):
        if not self._active:
            return True

        self._active = False
        self._socket.disconnect(f"tcp://127.0.0.1:{self._port}")
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
