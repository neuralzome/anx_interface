#!/usr/bin/env python3

import time
from concurrent.futures import ThreadPoolExecutor
import signal

from progress.spinner import PixelSpinner as Spinner
import zmq

from anx_mock.utils import Rate
from anx_mock.proto import assets_pb2, common_pb2, device_pb2
from anx_mock.assets.device_imu import DeviceImu
from anx_mock.assets.device_gnss import DeviceGnss
from anx_mock.assets.device_camera import DeviceCamera

class AnxMock:
    def __init__(self):
        self.active = False
        signal.signal(signal.SIGINT, self.signal_handler)

        self.executor = ThreadPoolExecutor()

        self.ctx = zmq.Context()

        self.socket_rpc = self.ctx.socket(zmq.REP)
        self.socket_rpc.bind("tcp://*:10002")
        self.poller_rpc = zmq.Poller()
        self.poller_rpc.register(self.socket_rpc, zmq.POLLIN)

        self.socket_pub_state = self.ctx.socket(zmq.PUB)
        self.socket_pub_state.bind("tcp://*:10001")

        self.rpc = {
                b"GetAssetState": self.get_asset_state,
                b"StartDeviceImu": self.start_device_imu,
                b"StartDeviceGnss": self.start_device_gnss,
                b"StartDeviceCamera": self.start_device_camera,
                b"StopDeviceImu": self.stop_device_imu,
                b"StopDeviceGnss": self.stop_device_gnss,
                b"StopDeviceCamera": self.stop_device_camera,
                b"GetImeiNumbers": self.get_imei_numbers,
                b"GetPhoneNumbers": self.get_phone_numbers,
                b"Shutdown": self.shutdown,
                b"Reboot": self.reboot,
                b"Tts": self.tts,
                b"GetAvailableLanguages": self.get_available_languages,
                b"IsTtsBusy": self.is_tts_busy,
                b"SetWifi": self.set_wifi
        }

        self.device_imu = DeviceImu(self.executor)
        self.device_gnss = DeviceGnss(self.executor)
        self.device_camera = DeviceCamera(self.executor)

    # Blocking call
    def start(self):
        if self.active:
            return

        self.active = True
        self.executor.submit(self.rpc_handler)
        self.executor.submit(self.pub_state)

        spinner = Spinner('AnxMock running ')
        while self.active:
            spinner.next()
            time.sleep(0.1)

        self.executor.shutdown(wait=True)

    def signal_handler(self, sig, fram):
        self.active = False
        print("\nAnxMock stopped running")

    def pub_state(self):
        msg = device_pb2.DeviceState()
        GB = 1024*1024*1024
        MB = 1024*1024
        msg.uptime_ms = 0
        msg.ram.used=200*MB
        msg.ram.total=4*GB
        msg.storage.used=32*GB
        msg.storage.total=128*GB
        msg.vram.used=0
        msg.vram.total=4*GB
        msg.cpu.processor = "Qualcomm Kryo 385"
        msg.cpu.architecture = "64bit"
        msg.cpu.type = "ARM Cortex-A75"

        freq = device_pb2.Freq()
        M = 1000*1000
        freq.minimum= 1000*M
        freq.maximum= 2400*M
        freq.current= 1500*M

        msg.cpu.freqs.append(freq)
        msg.gpu.renderer = "Qualcomm Adreno 630"

        thermal = device_pb2.Thermal()
        thermal.name = "cpu"
        thermal.temp_degree_celsius = 62.4

        msg.thermals.append(thermal)

        msg.battery.charging = True
        msg.battery.level_in_percentage = 62
        msg.battery.current_mA = 253.2
        msg.battery.voltage_mV = 3862.2

        rate = Rate(10)
        while self.active:
            msg.uptime_ms += 100

            msg_bytes = msg.SerializeToString()
            self.socket_pub_state.send(msg_bytes)
            rate.sleep()

    def rpc_handler(self):
        while self.active:
            events = self.poller_rpc.poll(100)
            if events:
                name, req_bytes = self.socket_rpc.recv_multipart()
                if name in self.rpc.keys():
                    self.rpc[name](req_bytes)

    def get_asset_state(self, req_bytes):
        rep = assets_pb2.AssetState()
        rep.imu = self.device_imu.get_select()
        rep.gnss = self.device_gnss.get_select()
        rep.camera = self.device_camera.get_select()

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def start_device_imu(self, req_bytes):
        req = assets_pb2.StartDeviceImu()
        req.ParseFromString(req_bytes)

        rep = common_pb2.StdResponse()
        rep.success = self.device_imu.start(req.fps, req.port)

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def start_device_gnss(self, req_bytes):
        req = assets_pb2.StartDeviceGnss()
        req.ParseFromString(req_bytes)

        rep = common_pb2.StdResponse()
        rep.success = self.device_gnss.start(req.port)

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def start_device_camera(self, req_bytes):
        req = assets_pb2.StartDeviceCamera()
        req.ParseFromString(req_bytes)

        rep = common_pb2.StdResponse()
        rep.success = self.device_camera.start(
            req.camera_stream.fps,
            req.camera_stream.width,
            req.camera_stream.height,
            req.camera_stream.pixel_format,
            req.port
        )

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def stop_device_imu(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = self.device_imu.stop()

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def stop_device_gnss(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = self.device_gnss.stop()

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def stop_device_camera(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = self.device_camera.stop()

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def get_imei_numbers(self, req_bytes):
        rep = device_pb2.GetImeiNumbersResponse()
        rep.imeis.extend(["869781035780142", "869781035780159"])
        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def get_phone_numbers(self, req_bytes):
        rep = device_pb2.GetPhoneNumbersResponse()
        rep.phone_numbers.extend(["+917320037614", "+919600511722"])
        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def shutdown(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = True
        rep.message = "Shutting down!!"

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def reboot(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = True
        rep.message = "Rebooting!!"

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def tts(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = True
        rep.message = "tts request received"

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def get_available_languages(self, req_bytes):
        rep = device_pb2.GetAvailableLanguagesResponse()
        rep.languages.extend(['en', 'fr'])

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def is_tts_busy(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = False

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def set_wifi(self, req_bytes):
        rep = common_pb2.StdResponse()
        rep.success = True
        rep.message = "wifi set!!"

        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

def main():
    anx_mock = AnxMock()
    anx_mock.start()

if __name__ == '__main__':
    main()
