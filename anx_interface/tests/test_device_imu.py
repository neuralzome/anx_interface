#!/usr/bin/env python3

from anx_interface import Anx

def imu_cb(imu_date):
    print(imu_date)

def main():
    anx = Anx()

    anx.start_device_imu(fps=10, cb=imu_cb)

    anx.wait()

if __name__ == "__main__":
    main()
