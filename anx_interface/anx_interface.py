#!/usr/bin/env python3

import time
import signal
from concurrent.futures import ThreadPoolExecutor

import zmq

import anx_proto.python.assets_pb2 as assets_pb2
import anx_proto.python.device_pb2 as device_pb2
import anx_proto.python.common_pb2 as common_pb2
from anx_interface.assets.device_imu import DeviceImu
from anx_interface.assets.device_gnss import DeviceGnss
from anx_interface.assets.device_camera import DeviceCamera
from anx_interface.utils.fifo import FiFo, parse_msg

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

        self._fifo = FiFo("/dev/socket/anx_in", "/dev/socket/anx_out")

        self._device_imu_started = False
        self._device_gnss_started = False
        self._device_camera_started = False
        self.device_imu = DeviceImu(self._executor)
        self.device_gnss = DeviceGnss(self._executor)
        self.device_camera = DeviceCamera(self._executor)

    def ok(self):
        """Return True if anx_interface is active"""
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
        """Waits until everything exist cleanly"""
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
        """
        Listen to device_imu data stream, which has already been started
        Arguments:
            cb -- function which receives data events
        """
        self.device_imu._start(cb=cb)

    def listen_device_gnss(self, cb=None):
        """
        Listen to device_gnss data stream, which has already been started
        Arguments:
            cb -- function which receives data events
        """
        self.device_gnss._start(cb=cb)

    def listen_device_camera(self, cb=None):
        """
        Listen to device_camera data stream, which has already been started
        Arguments:
            cb -- function which receives data events
        """
        self.device_camera._start(cb=cb)

    def stop_listening_device_imu(self):
        """Stop listening to device_imu stream"""
        self.device_imu._stop()

    def stop_listening_device_gnss(self):
        """Stop listening to device_gnss stream"""
        self.device_gnss._stop()

    def stop_listening_device_camera(self):
        """Stop listening to device_camera stream"""
        self.device_camera._stop()

    def start_device_imu(self, fps, cb=None):
        """
        Start streaming device_imu
        Arguments:
            fps -- select supported fps (check asset_state)
            cb -- function which receives data events
        Returns:
            True if successful
        """
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
        """
        Start streaming device_gnss
        Arguments:
            cb -- function which receives data events
        Returns:
            True if successful
        """
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
        """
        Start streaming device_camera
        Arguments:
            fps -- select supported pixel_format (check asset_state)
            width -- select supported pixel_format (check asset_state)
            height -- select supported pixel_format (check asset_state)
            pixel_format -- select supported pixel_format (check asset_state)
            cb -- function which receives data events
        Returns:
            True if successful
        """
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
        """
        Stop streaming device_imu
        Returns:
            True if successful
        """
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
        """
        Stop streaming device_gnss
        Returns:
            True if successful
        """
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
        """
        Stop streaming device_camera
        Returns:
            True if successful
        """
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
        """returns list of IMEI numbers in order of sim slot"""
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
        """Shutdowns the device"""
        response = self._fifo.send_msg("Shutdown")
        return parse_msg(response)

    def reboot(self):
        """Reboots the device"""
        response = self._fifo.send_msg("Reboot")
        return parse_msg(response)

    def restart_anx_service(self):
        """Restart anx service which is responsible for all the rpc and sensor streams"""
        response = self._fifo.send_msg("RestartAnxService")
        return parse_msg(response)
    

    def set_wifi(self, ssid, password) -> tuple[bool, str]:
        """
        Set the wifi network the device should connect to
        Arguments:
            ssid -- ssid of the wifi network

            password -- password of the wifi network
        Returns:
            True if successful.
            
            False and a message otherwise
        """
        if ssid == "" or password == "":
            return (False, "Received empty ssid/password.")

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
            return (rep.success, rep.message)

        return (False, "SetHotspot rpc called timed out")

    def set_hotspot(self, ssid, password) -> tuple[bool, str]:
        """
        Set device hotspot ssid and password
        
        Note : Minimum length of password is 10 characters
        
        Arguments:
            ssid -- ssid of the hotspot
            
            password -- password of the hotspot
        Returns:
            True if successful.
            
            False and a message otherwise
        """
        if ssid == "" or password == "":
            return (False, "Received empty ssid/password.")

        if len(password) < 10:
            return (False, f"Password should have minimum length of 10. Received {len(password)}")

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
            return (rep.success, rep.message)

        return (False, "SetHotspot rpc called timed out")

    def get_floos_version(self):
        """Return flo os version"""
        response = self._fifo.send_msg("GetFloOsVersion")
        return parse_msg(response)

    def get_anx_version(self):
        """Return anx version"""
        req = common_pb2.Empty()
        req_bytes = req.SerializeToString()

        self._socket_rpc.send_multipart([b"GetAnxVersion", req_bytes])

        events = self._poller_rpc.poll(2000)
        if events:
            rep = device_pb2.VersionResponse()
            rep_bytes = self._socket_rpc.recv()
            rep.ParseFromString(rep_bytes)
            return rep.version

        return None

    def start_android_logs(self):
        """Starts logging logs in /.logs/system"""
        response = self._fifo.send_msg("StartAndroidLogs")
        return parse_msg(response)

    def stop_android_logs(self):
        """Stops logging"""
        response = self._fifo.send_msg("StopAndroidLogs")
        return parse_msg(response)
