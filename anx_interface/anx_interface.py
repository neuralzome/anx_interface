#!/usr/bin/env python3

import time
import signal
from concurrent.futures import ThreadPoolExecutor

import zmq
from anx_interface.proto import assets_pb2, device_pb2, common_pb2
from anx_interface.assets.device_imu import DeviceImu
from anx_interface.assets.device_gnss import DeviceGnss
from anx_interface.assets.device_camera import DeviceCamera

class AnxInterface:
    def __init__(self):
        self.executor = ThreadPoolExecutor()

        self.terminated = False
        signal.signal(signal.SIGINT, self.signal_handler)

        self.ctx = zmq.Context()

        self.socket_rpc = self.ctx.socket(zmq.REQ)
        self.socket_rpc.connect('tcp://127.0.0.1:10002')
        self.poller_rpc = zmq.Poller()
        self.poller_rpc.register(self.socket_rpc, zmq.POLLIN)

        self.socket_device_state = self.ctx.socket(zmq.SUB)
        self.socket_device_state.connect("tcp://127.0.0.1:10001")
        self.socket_device_state.setsockopt_string(zmq.SUBSCRIBE, "")
        self.poller_device_state = zmq.Poller()
        self.poller_device_state.register(self.socket_device_state, zmq.POLLIN)

        self.device_state_cb = None
        self.device_state = None

        self.executor.submit(self.device_state_thread)
        if not self.wait_until_device_state_init():
            print("Failed to init device state")

        self.asset_state = self.get_asset_state()

        self.device_imu_started = False
        self.device_gnss_started = False
        self.device_camera_started = False
        self.device_imu = DeviceImu(self.executor)
        self.device_gnss = DeviceGnss(self.executor)
        self.device_camera = DeviceCamera(self.executor)

    def ok(self):
        return not self.terminated

    def signal_handler(self, sig, frame):
        if self.device_imu.active:
            if self.device_imu_started:
                self.stop_device_imu()
            else:
                self.device_imu.stop()

        if self.device_gnss.active:
            if self.device_gnss_started:
                self.stop_device_gnss()
            else:
                self.device_gnss.stop()

        if self.device_camera.active:
            if self.device_camera_started:
                self.stop_device_camera()
            else:
                self.device_camera.stop()

        self.terminated = True

    def wait(self):
        self.executor.shutdown(wait=True)

    def wait_until_device_state_init(self, timeout_sec=1.0):
        time_start = time.monotonic()
        while self.device_state is None:
            time.sleep(0.1)
            if time.monotonic() - time_start > timeout_sec:
                return False
        return True

    def register_device_state_cb(self, device_state_cb):
        self.device_state_cb = device_state_cb

    def device_state_thread(self):
        while not self.terminated:
            events = self.poller_device_state.poll(100)
            if events:
                msg_bytes = self.socket_device_state.recv()
                msg = device_pb2.DeviceState()
                msg.ParseFromString(msg_bytes)
                self.device_state = msg
                if self.device_state_cb is not None:
                    self.device_state_cb(msg)

    def get_asset_state(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"GetAssetState", req_bytes])

        rep = assets_pb2.AssetState()
        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep.ParseFromString(rep_bytes)

        return rep

    def listen_device_imu(self, cb=None):
        self.device_imu.start(cb=cb)

    def listen_device_gnss(self, cb=None):
        self.device_gnss.start(cb=cb)

    def listen_device_camera(self, cb=None):
        self.device_camera.start(cb=cb)

    def start_device_imu(self, fps, cb=None):
        if fps not in self.asset_state.imu.fps:
            return False

        req = assets_pb2.StartDeviceImu()
        req.fps = fps
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"StartDeviceImu", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.device_imu_started = True
                self.device_imu.start(cb=cb)
                return True

        return False

    def start_device_gnss(self, cb=None):
        if not self.asset_state.gnss.available:
            return False

        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"StartDeviceGnss", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.device_gnss_started = True
                self.device_gnss.start(cb=cb)
                return True

        return False

    def start_device_camera(self, fps, width, height, pixel_format, cb=None):
        # TODO: Check if request is valid
        req = assets_pb2.StartDeviceCamera()
        req.camera_stream.fps = fps
        req.camera_stream.width = width
        req.camera_stream.height = height
        req.camera_stream.pixel_format = pixel_format
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"StartDeviceCamera", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.device_camera_started = True
                self.device_camera.start(cb=cb)
                return True

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
                self.device_imu_started = False
                self.device_imu.stop()
                return True

        return False

    def stop_device_gnss(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"StopDeviceGnss", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.device_gnss_started = False
                self.device_gnss.stop()
                return True

        return False

    def stop_device_camera(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"StopDeviceCamera", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep_bytes = self.socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.device_camera_started = False
                self.device_camera.stop()
                return True

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

    def shutdown(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"Shutdown", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self.socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def reboot(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"Reboot", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self.socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def tts(self, msg, lang):
        req = device_pb2.TtsRequest()
        req.message = msg
        req.language = lang
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"Tts", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self.socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def get_available_languages(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"GetAvailableLanguages", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep = device_pb2.GetAvailableLanguagesResponse()
            rep_bytes = self.socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.languages

        return []

    def is_tts_busy(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"IsTtsBusy", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self.socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def set_wifi(self, ssid, password):
        req = device_pb2.SetWifiRequest()
        req.ssid = ssid
        req.password = password
        req_bytes = req.SerializeToString()

        self.socket_rpc.send_multipart([b"SetWifi", req_bytes])

        events = self.poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self.socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False
