#!/usr/bin/env python3

import rclpy
from rclpy import Node

import pynmea2
from sensor_msgs.msg import NavSatFix
from anx_interface import Anx

class GnssDriver(Node):
    def __init__(self):
        super().__init__("gnss_driver")

        self.anx = Anx()
        
        self.fix_pub = self.create_publisher(
            NavSatFix,
            "fix",
            10
        )
        self.anx.start_device_gnss(cb=self.gnss_cb)

    def gnss_cb(self, data):
        if data.split(',')[0][3:] == 'GGA':
            gnss_data_parsed = pynmea2.parse(data)
        if gnss_data_parsed.num_sats == '':
            return

        lat = float(gnss_data_parsed.lat[:2]) + (float(gnss_data_parsed.lat[2:])/60.0)
        lat = -lat if gnss_data_parsed.lat_dir == 'S' else lat

        lon = float(gnss_data_parsed.lon[:3]) + (float(gnss_data_parsed.lon[3:])/60.0)
        lon = -lon if gnss_data_parsed.lon_dir == 'W' else lon

        fix = NavSatFix()
        fix.latitude = lat
        fix.longitude = lon
        self.fix_pub.publish(fix)

def main(args=None):
    rclpy.init(args=args)
    gnss_driver = GnssDriver()
    gnss_driver.anx.wait()

if __name__ == "__main__":
    main()