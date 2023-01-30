#!/usr/bin/env python3

import sys
import time
import signal
from concurrent.futures import ThreadPoolExecutor

import zmq
from anx_interface.assets_manager import AssetsManager
from anx_interface.proto import device_pb2, common_pb2

class AnxInterface:
    def __init__(self):
        self.executor = ThreadPoolExecutor()

        self.terminated = False
        signal.signal(signal.SIGINT, self.signal_handler)

        self.ctx = zmq.Context()

        self.socket_rpc = self.ctx.socket(zmq.REQ)
        self.socket_rpc.connect('tcp://localhost:10002')
        self.poller_rpc = zmq.Poller()
        self.poller_rpc.register(self.socket_rpc, zmq.POLLIN)

        self.socket_state = self.ctx.socket(zmq.SUB)
        self.socket_state.connect("tcp://localhost:10001")
        self.socket_state.setsockopt_string(zmq.SUBSCRIBE, "")
        self.poller_state = zmq.Poller()
        self.poller_state.register(self.socket_state, zmq.POLLIN)

        self.state_cb = None
        self.state = None

        self.port_pool = [port for port in range(10010, 10500)]

        self.executor.submit(self.state_thread)
        if not self.wait_until_state_init():
            print("Failed to init device state")

        self.assets_manager = AssetsManager(self.executor, self)

    def ok(self):
        return not self.terminated

    def signal_handler(self, sig, frame):
        self.assets_manager.terminate()
        self.terminated = True

    def wait(self):
        self.executor.shutdown(wait=True)

    def wait_until_state_init(self, timeout_sec=1.0):
        time_start = time.monotonic()
        while self.state is None:
            time.sleep(0.1)
            if time.monotonic() - time_start > timeout_sec:
                return False
        return True

    def register_state_cb(self, state_cb):
        self.state_cb = state_cb

    def get_last_state(self):
        return self.state

    def state_thread(self):
        while not self.terminated:
            events = self.poller_state.poll(100)
            if events:
                msg_bytes = self.socket_state.recv()
                msg = device_pb2.DeviceState()
                msg.ParseFromString(msg_bytes)
                self.state = msg
                if self.state_cb is not None:
                    self.state_cb(msg)

    def republish_asset_state(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"RepublishAssetAtate", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            return rep.success
        else:
            return False

    def get_imei_numbers(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"GetImeiNumbers", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = device_pb2.GetImeiNumbersResponse()
            rep.ParseFromString(rep_bytes)
            return rep.imeis
        else:
            return []

    def get_phone_numbers(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"GetPhoneNumbers", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = device_pb2.GetPhoneNumbersResponse()
            rep.ParseFromString(rep_bytes)
            return rep.phone_numbers
        else:
            return []

    @property
    def device_imu(self):
        return self.assets_manager.device_imu

    def start_device_imu(self, fps):
        if fps not in self.assets_manager.state.device_assets.imu.fps:
            return False

        req = asset_pb2.StartDeviceImu()
        req.fps = fps
        req.port = self.port_pool.pop()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"StartDeviceImu", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.assets_manager.device_imu.start(req.port)

        return False

    def stop_device_imu(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"StopDeviceImu", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.assets_manager.device_imu.stop()

        return False
