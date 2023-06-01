#!/usr/bin/env python3

import click
import time
import numpy as np
from anx_interface import Anx
from anx_logging import PerfLogger

class HzObserver:
    def __init__(self, N):
        self.N = N
        self.timestamps = []

    def ping(self):
        self.timestamps.append(time.time())
        if len(self.timestamps) > self.N:
            self.timestamps.pop(0)
            hz = (self.N - 1) / (self.timestamps[-1] - self.timestamps[0])
            return round(hz)
        return -1

@click.command(name="gnss_config")
def get_gnss_config():
    data = f"{anx.asset_state.gnss}"
    print(f"{data.rstrip()}")

def gnss_cb(gnss_data):
    data = f"{gnss_data}"
    print(f"[{time.time()}] {data.rstrip()}")

@click.command(name="stream_gnss")
def stream_gnss():
    anx.start_device_gnss(cb=gnss_cb)
    anx.wait()

@click.command(name="imu_config")
def get_imu_config():
    data = f"{anx.asset_state.imu}"
    print(f"{data.rstrip()}")

def imu_cb(imu_data):
    data = f"{imu_data}"
    print(f"[hz = {hz_observer.ping()}]\n{data.rstrip()}")

@click.command(name="stream_imu")
@click.option("--fps", default=10, help="IMU fps")
def stream_imu(fps):
    anx.start_device_imu(fps=fps, cb=imu_cb)
    anx.wait()

@click.command(name="camera_config")
def get_camera_config():
    data = f"{anx.asset_state.camera}"
    print(f"{data.rstrip()}")

def camera_cb(camera_data):
    print(f"[hz = {hz_observer.ping()}] {camera_data.shape}")

@click.command(name="stream_camera")
@click.option("--fps", default=30, help="camera fps")
@click.option("--width", default=480, help="image width")
@click.option("--height", default=640, help="image height")
@click.option("--pixel_format", default=0, help="0: MJPEG, 1: YUV420")
def stream_camera(fps, width, height, pixel_format):
    anx.start_device_camera(
            fps=fps,
            width=width,
            height=height,
            pixel_format=pixel_format,
            cb=camera_cb
    )
    anx.wait()

@click.command(name="imei_numbers")
def get_imei_numbers():
    data = f"{anx.get_imei_numbers()}"
    print(f"{data.rstrip()}")

@click.command(name="shutdown")
def shutdown():
    response = anx.shutdown()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in shutting down : {response[1]}")

@click.command(name="reboot")
def reboot():
    response = anx.reboot()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in rebooting : {response[1]}")

@click.command(name="restart_anx_service")
def restart_anx_service():
    response = anx.restart_anx_service()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in restarting anx service : {response[1]}")

@click.command(name="set_wifi")
@click.argument("ssid")
@click.argument("password")
def set_wifi(ssid, password):
    status, msg = anx.set_wifi(ssid, password)
    if status:
        print("Setting Wifi. Run anx connect_wifi")
    else:
        print(f"Error in setting wifi : {msg}")

@click.command(name="connect_wifi")
def connect_wifi():
    status, msg = anx.connect_wifi()
    if status:
        print(msg)
    else:
        print(f"Error in setting wifi : {msg}")

@click.command(name="disconnect_wifi")
def disconnect_wifi():
    status, msg = anx.disconnect_wifi()
    if status:
        print(msg)
    else:
        print(f"Error in setting wifi : {msg}")

@click.command(name="set_hotspot")
@click.argument("ssid")
@click.argument("password")
def set_hotspot(ssid, password):
    status, msg = anx.set_hotspot(ssid, password)
    if status:
        print("Setting hotspot. Check in a few seconds")
    else:
        print(f"Error in setting hotspot : {msg}")

@click.command(name="anx_version")
def get_anx_version():
    data = f"{anx.get_anx_version()}"
    print(f"{data.rstrip()}")

@click.command(name="floos_version")
def get_floos_version():
    response = anx.get_floos_version()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in getting flo os version : {response[1]}")

@click.command(name="start_android_logs")
def start_android_logs():
    response = anx.start_android_logs()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in starting logs : {response[1]}")

@click.command(name="stop_android_logs")
def stop_android_logs():
    response = anx.stop_android_logs()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in stopping logs : {response[1]}")

@click.command(name="wifi_stats")
def wifi_stats():
    response = anx.get_wifi_stats()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in getting wifi stats : {response[1]}")

@click.command(name="hotspot_stats")
def hotspot_stats():
    response = anx.get_hotspot_stats()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in getting hotspot stats : {response[1]}")

@click.command(name="cellular_stats")
def cellular_stats():
    response = anx.get_cellular_stats()
    print(f"{response}")

@click.command(name="reset_fs")
def reset_fs():
    response = anx.reset_fs()
    if response[0]:
        print(response[1])
    else:
        print(f"Error in resettig file system : {response[1]}")

@click.command(name="perf_logger")
def perf_logger():
    perflogger = PerfLogger()
    perflogger.logger_thread()

@click.group()
def cli():
    pass

anx = None
hz_observer = None

def main():
    global anx
    anx = Anx()

    global hz_observer
    hz_observer = HzObserver(10)

    cli.add_command(get_gnss_config)
    cli.add_command(stream_gnss)
    cli.add_command(get_imu_config)
    cli.add_command(stream_imu)
    cli.add_command(get_camera_config)
    cli.add_command(stream_camera)
    cli.add_command(get_imei_numbers)
    cli.add_command(shutdown)
    cli.add_command(reboot)
    cli.add_command(restart_anx_service)
    cli.add_command(set_wifi)
    cli.add_command(connect_wifi)
    cli.add_command(disconnect_wifi)
    cli.add_command(set_hotspot)
    cli.add_command(get_anx_version)
    cli.add_command(get_floos_version)
    cli.add_command(start_android_logs)
    cli.add_command(stop_android_logs)
    cli.add_command(wifi_stats)
    cli.add_command(hotspot_stats)
    cli.add_command(cellular_stats)
    cli.add_command(reset_fs)
    cli.add_command(perf_logger)
    cli()
