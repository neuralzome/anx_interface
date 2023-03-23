#!/usr/bin/env python3

import click
import time
import numpy as np
from anx_interface import Anx


@click.command(name="gnss_config")
def get_gnss_config():
    print(f"{anx.asset_state.gnss}")

def gnss_cb(gnss_date):
    print(gnss_date)

@click.command(name="stream_gnss")
def stream_gnss():
    anx.start_device_gnss(cb=gnss_cb)
    anx.wait()

@click.command(name="imu_config")
def get_imu_config():
    print(f"{anx.asset_state.imu}")

def imu_cb(imu_date):
    print(imu_date)

@click.command(name="stream_imu")
@click.option("--fps", default=10, help="IMU fps")
def stream_imu(fps):
    anx.start_device_imu(fps=fps, cb=imu_cb)
    anx.wait()

@click.command(name="camera_config")
def get_camera_config():
    print(f"{anx.asset_state.camera}")

def camera_cb(camera_date):
    print(f"[{time.time()}] {camera_data.shape}")

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
    print(f"{anx.get_imei_numbers()}")

@click.command(name="shutdown")
def shutdown():
    print("Shutting down!!")
    anx.shutdown()

@click.command(name="reboot")
def reboot():
    print("Rebooting!!")
    anx.reboot()

@click.command(name="restart_anx_service")
def restart_anx_service():
    if anx.restart_anx_service():
        print("Restarting anx service")

@click.command(name="set_wifi")
@click.argument("SSID")
@click.argument("PASSWORD")
def set_wifi(SSID, PASSWORD):
    if anx.set_wifi(SSID, PASSWORD):
        print("Setting Wifi")

@click.command(name="set_hotspot")
@click.argument("SSID")
@click.argument("PASSWORD")
def set_hotspot(SSID, PASSWORD):
    if anx.set_hotspot(SSID, PASSWORD):
        print("Setting hotspot")

@click.command(name="anx_version")
def get_anx_version():
    print(f"{anx.get_anx_version()}")

@click.command(name="floos_version")
def get_floos_version():
    print(f"{anx.get_floos_version()}")

@click.command(name="start_android_logs")
def start_android_logs():
    if anx.start_android_logs():
        print(f"started android logs!")

@click.command(name="stop_android_logs")
def stop_android_logs():
    if anx.stop_android_logs():
        print(f"stopped android logs!")

@click.group()
def cli():
    pass

anx = None

def main():
    global anx
    anx = Anx()
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
    cli.add_command(set_hotspot)
    cli.add_command(get_anx_version)
    cli.add_command(get_floos_version)
    cli.add_command(start_android_logs)
    cli.add_command(stop_android_logs)
    cli()
