#!/usr/bin/env python3

import sys
import time
from concurrent.futures import ThreadPoolExecutor

import zmq
from anx_interface.proto import assets_pb2, device_pb2, common_pb2
from anx_interface.assets.device_imu import DeviceImu

class AssetsManager:
    def __init__(self, executor, anx_interface):
        self.executor = executor
        self.anx_interface = anx_interface

        self.ctx = zmq.Context()

        self.socket = self.ctx.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:10000")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.terminated = False

        self.state_cb = None
        self.state = None

        self.executor.submit(self.state_thread)
        # if not self.wait_until_state_init():
        #     print("Failed to init asset state")

        self.device_imu = DeviceImu(self.executor)

    def terminate(self):
        self.terminated = True

    def wait_until_state_init(self, timeout_sec=1.0):
        if self.state is not None:
            return True

        if not self.anx_interface.republish_asset_state():
            return False

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
            events = self.poller.poll(100)
            if events:
                msg_bytes = self.socket.recv()
                msg = asset_pb2.AssetState()
                msg.ParseFromString(msg_bytes)
                self.asset_state = msg
                if self.asset_state_cb is not None:
                    self.asset_state_cb(msg)
