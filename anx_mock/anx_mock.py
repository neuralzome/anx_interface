#!/usr/bin/env python3

import time
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
import signal

from progress.spinner import PixelSpinner as Spinner
import click
import zmq
from anx_mock.proto import common_pb2, device_pb2

class AnxMock:
    def __init__(
        self,
        dummy_device_imu: bool,
        dummy_device_gnss,
        dummy_device_camera,
        number_usb_camera,
        number_usb_serial
    ):
        self.dummy_device_imu_present = dummy_device_imu
        self.dummy_device_gnss_present = dummy_device_gnss
        self.dummy_device_camera_present = dummy_device_camera
        self.number_usb_camera = number_usb_camera
        self.number_usb_serial = number_usb_serial

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
                b"GetImeiNumbers": self.get_imei_numbers,
                b"GetPhoneNumbers": self.get_phone_numbers
        }

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

        while self.active:
            msg.uptime_ms += 100

            msg_bytes = msg.SerializeToString()
            self.socket_pub_state.send(msg_bytes)
            time.sleep(0.1)

    def rpc_handler(self):
        while self.active:
            events = self.poller_rpc.poll(100)
            if events:
                name, req = self.socket_rpc.recv_multipart()
                if name in self.rpc.keys():
                    self.rpc[name](req)

    def get_imei_numbers(self, req):
        rep = device_pb2.GetImeiNumbersResponse()
        rep.imeis.extend(["869781035780142", "869781035780159"])
        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

    def get_phone_numbers(self, msg):
        rep = device_pb2.GetPhoneNumbersResponse()
        rep.phone_numbers.extend(["+917320037614", "+919600511722"])
        rep_bytes = rep.SerializeToString()
        self.socket_rpc.send(rep_bytes)

@click.command()
@click.option("--dummy_device_imu", is_flag=True)
@click.option("--dummy_device_gnss", is_flag=True)
@click.option("--dummy_device_camera", is_flag=True)
@click.option("--number_usb_camera", default=0, help="Number of UsbCamera")
@click.option("--number_usb_serial", default=0, help="Number of UsbSerial")
def main(dummy_device_imu, dummy_device_gnss, dummy_device_camera, number_usb_camera, number_usb_serial):
    print("anx_mock assets:")
    print(f"  dummy_device_imu: {'Present' if dummy_device_imu else 'Not present'}")
    print(f"  dummy_device_gnss: {'Present' if dummy_device_gnss else 'Not present'}")
    print(f"  dummy_device_camera: {'Present' if dummy_device_camera else 'Not present'}")
    print(f"  number_usb_camera: {number_usb_camera}")
    print(f"  number_usb_serial: {number_usb_serial}")

    anx_mock = AnxMock(dummy_device_imu, dummy_device_gnss, dummy_device_camera, number_usb_camera, number_usb_serial)

    anx_mock.start()

if __name__ == '__main__':
    main()
