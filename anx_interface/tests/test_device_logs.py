#!/usr/bin/env python3

from anx_interface import Anx

def device_logs_cb(device_log):
    print(device_log)

def main():
    anx = Anx()

    anx.register_device_logs_cb(device_logs_cb)

    anx.wait()

if __name__ == "__main__":
    main()
