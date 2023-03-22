#!/usr/bin/env python3

import click
from anx_interface import Anx

anx = None

@click.command(name="device_gnss_config")
def get_gnss_config():
    print(f"{anx.asset_state.gnss}")

@click.command(name="device_imu_config")
def get_imu_config():
    print(f"{anx.asset_state.imu}")

@click.command(name="device_camera_config")
def get_camera_config():
    print(f"{anx.asset_state.camera}")

@click.command(name="imei_number")
def get_imei_number():
    print(f"{anx.get_imei_number()}")

@click.command(name="shutdown")
def shutdown():
    print("Shutting down!!")
    anx.shutdown()

@click.command(name="reboot")
def reboot():
    print("Rebooting!!")
    anx.reboot()

@click.command(name="anx_version")
def get_anx_version():
    print(f"{anx.get_anx_version()}")

@click.command(name="floos_version")
def get_floos_version():
    print(f"{anx.get_floos_version()}")

@click.group()
def cli():
    pass

def main():
    anx = Anx()
    cli.add_command(get_gnss_config)
    cli.add_command(get_imu_config)
    cli.add_command(get_camera_config)
    cli.add_command(get_imei_number)
    cli.add_command(shutdown)
    cli.add_command(reboot)
    cli.add_command(get_anx_version)
    cli.add_command(get_floos_version)
    cli()
