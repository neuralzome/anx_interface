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
        self._executor = ThreadPoolExecutor()

        self._terminated = False
        signal.signal(signal.SIGINT, self._signal_handler)

        self._ctx = zmq.Context()

        self._socket_rpc = self._ctx.socket(zmq.REQ)
        self._socket_rpc.connect("ipc:///ipc/device_rpc")
        self._poller_rpc = zmq.Poller()
        self._poller_rpc.register(self._socket_rpc, zmq.POLLIN)

        self.asset_state = self._get_asset_state()

        self._device_imu_started = False
        self._device_gnss_started = False
        self._device_camera_started = False
        self.device_imu = DeviceImu(self._executor)
        self.device_gnss = DeviceGnss(self._executor)
        self.device_camera = DeviceCamera(self._executor)

    def ok(self):
        return not self._terminated

    def _signal_handler(self, sig, frame):
        if self.device_imu._active:
            if self._device_imu_started:
                self.stop_device_imu()
            else:
                self.device_imu._stop()

        if self.device_gnss._active:
            if self._device_gnss_started:
                self.stop_device_gnss()
            else:
                self.device_gnss._stop()

        if self.device_camera._active:
            if self._device_camera_started:
                self.stop_device_camera()
            else:
                self.device_camera._stop()

        self._terminated = True

    def wait(self):
        self._executor.shutdown(wait=True)

    def _get_asset_state(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"GetAssetState", req_bytes])

        rep = assets_pb2.AssetState()
        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)

        return rep

    def listen_device_imu(self, cb=None):
        self.device_imu._start(cb=cb)

    def listen_device_gnss(self, cb=None):
        self.device_gnss._start(cb=cb)

    def listen_device_camera(self, cb=None):
        self.device_camera._start(cb=cb)

    def stop_listening_device_imu(self):
        self.device_imu._stop()

    def stop_listening_device_gnss(self):
        self.device_gnss._stop()

    def stop_listening_device_camera(self):
        self.device_camera._stop()

    def start_device_imu(self, fps, cb=None):
        # Check if request is valid
        if fps not in self.asset_state.imu.fps:
            return False

        req = assets_pb2.StartDeviceImu()
        req.fps = fps
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StartDeviceImu", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self._device_imu_started = True
                self.device_imu._start(cb=cb)
                return True

        return False

    def start_device_gnss(self, cb=None):
        # Check if request is valid
        if not self.asset_state.gnss.available:
            return False

        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StartDeviceGnss", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self._device_gnss_started = True
                self.device_gnss._start(cb=cb)
                return True

        return False

    def start_device_camera(self, fps, width, height, pixel_format, cb=None):
        # Check if request is valid
        is_valid = False
        for camera_stream in self.asset_state.camera.camera_streams:
            if fps == camera_stream.fps and width == camera_stream.width and height == camera_stream.height and pixel_format == camera_stream.pixel_format:
                is_valid = True
                break
        if not is_valid:
            return False

        req = assets_pb2.StartDeviceCamera()
        req.camera_stream.fps = fps
        req.camera_stream.width = width
        req.camera_stream.height = height
        req.camera_stream.pixel_format = pixel_format
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StartDeviceCamera", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self._device_camera_started = True
                self.device_camera._start(cb=cb)
                return True

        return False

    def stop_device_imu(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StopDeviceImu", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.device_imu_started = False
                self.device_imu._stop()
                return True

        return False

    def stop_device_gnss(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StopDeviceGnss", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self._device_gnss_started = False
                self.device_gnss._stop()
                return True

        return False

    def stop_device_camera(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StopDeviceCamera", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep = common_pb2.StdResponse()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self._device_camera_started = False
                self.device_camera._stop()
                return True

        return False

    def get_imei_numbers(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"GetImeiNumbers", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep_bytes = self._socket_rpc.recv()
            rep = device_pb2.GetImeiNumbersResponse()
            rep.ParseFromString(rep_bytes)
            return rep.imeis
        else:
            return []

    def shutdown(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"Shutdown", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def reboot(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"Reboot", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def restart_anx_service(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"RestartAnxService", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def set_wifi(self, ssid, password):
        req = device_pb2.SetWifiRequest()
        req.ssid = ssid
        req.password = password
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"SetWifi", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def set_hotspot(self, ssid, password):
        req = device_pb2.SetWifiRequest()
        req.ssid = ssid
        req.password = password
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"SetHotspot", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def start_android_logs(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StartAndroidLogs", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False

    def stop_android_logs(self):
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"StopAndroidLogs", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.success

        return False
