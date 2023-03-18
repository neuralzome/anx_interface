#!/usr/bin/env python3

from anx_interface import Anx

def gnss_cb(gnss_date):
    print(gnss_date)

def main():
    anx = Anx()

    anx.start_device_gnss(cb=gnss_cb)

    anx.wait()

if __name__ == "__main__":
    main()
