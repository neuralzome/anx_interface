#!/usr/bin/env python3

from anx_interface import Anx

class DeviceLogsLogger:
    def __init__(self):
        self.anx = Anx()
        self.anx.register_device_logs_cb(self.device_logs_cb)

    def wait(self):
        self.anx.wait()

    def device_logs_cb(device_log):
        # TODO: Log device logs
        print(device_log)

def main():
    device_logs_logger = DeviceLogsLogger()
    device_logs_logger.wait()

if __name__ == "__main__":
    main()
