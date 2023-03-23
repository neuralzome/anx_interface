#!/usr/bin/env python3

import time
import signal
from datetime import datetime
import os
import psutil
import numpy as np

class PerfLogger:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)

        self.thermal_zones = {
                "cpu0-silver-usr": "thermal_zone11",
                "cpu1-silver-usr": "thermal_zone22",
                "cpu2-silver-usr": "thermal_zone33",
                "cpu3-silver-usr": "thermal_zone44",
                "cpu0-gold-usr": "thermal_zone71",
                "cpu1-gold-usr": "thermal_zone72",
                "cpu2-gold-usr": "thermal_zone73",
                "cpu3-gold-usr": "thermal_zone1"
        }

        self.filepath = "/logs/perflogs/"
        self.filename = f"{datetime.now().strftime('%d-%m-%Y-%H-%M-%S')}.csv"

        self.ok = True

        self.thermals = []
        self.core_freq = []
        self.ram = 0
        self.cpu_percentage = 0
        self.gpu_percentage = 0
        self.V = 0
        self.I = 0

        self.init_logfile()

    def signal_handler(self, sig, frame):
        self.ok = False

    def init_logfile(self):
        if not os.path.exists(self.filepath):
            os.makedirs(self.filepath)
        with open(self.filepath + self.filename, 'w') as fd:
            fd.write('Timestamp,')
            for i in range(4):
                fd.write(f"cpu{i}-silver-usr,")
            for i in range(4):
                fd.write(f"cpu{i}-gold-usr,")

            fd.write("ram,")
            fd.write("cpu%,")
            fd.write("gpu%,")
            fd.write("bat_V%,")
            fd.write("bat_I%")
            fd.write("\n")

    def logger_thread(self):
            while self.ok:
                with open(self.filepath + self.filename, 'a') as fd:
                    # 0. Log timestamp
                    fd.write(f"{time.time()},")

                    # 1. Log thermal data
                    self.thermals = []
                    for i in range(4):
                        path = f"/sys/devices/virtual/thermal/{self.thermal_zones[f'cpu{i}-silver-usr']}/temp"
                        temp = float(self.read(path)) / 1000
                        self.thermals.append(temp)
                        fd.write(f"{temp},")
                    for i in range(4):
                        path = f"/sys/devices/virtual/thermal/{self.thermal_zones[f'cpu{i}-gold-usr']}/temp"
                        temp = float(self.read(path)) / 1000
                        self.thermals.append(temp)
                        fd.write(f"{temp},")

                    # 2. Log cpu freq data
                    self.core_freq = []
                    for i in range(8):
                        path = f"/sys/devices/system/cpu/cpu{i}/cpufreq/cpuinfo_cur_freq"
                        freq = int(self.read(path))
                        self.core_freq.append(freq)
                        fd.write(f"{freq},")

                    # 3. Log ram data
                    self.ram = "{:.2f}".format(psutil.virtual_memory().used/(1024*1024)) # In MB
                    fd.write(f"{self.ram},")

                    self.gpu_percentage = 0
                    self.V = 0
                    self.I = 0
                    # 4. Log cpu percentage
                    self.cpu_percentage = psutil.cpu_percent(1) # Blocking for 1s
                    fd.write(f"{self.cpu_percentage},")

                    # 5. Log gpu percentage
                    path = "/sys/class/kgsl/kgsl-3d0/gpu_busy_percentage"
                    self.gpu_percentage = float(self.read(path).split(' ')[0])
                    fd.write(f"{self.gpu_percentage},")

                    # 6. Log battery terminal input voltage
                    path = "/sys/class/power_supply/battery/voltage_now"
                    self.V = "{:.2f}".format(float(self.read(path))/1000)
                    fd.write(f"{self.V},")

                    # 7. Log battery terminal input current
                    path = "/sys/class/power_supply/battery/current_now"
                    self.I = "{:.2f}".format(float(self.read(path))/1000)
                    fd.write(f"{self.I}")

                    fd.write("\n")

                # Display logs
                silver_core_avg_temp = "{:.2f}".format(np.array(self.thermals[:4]).sum()/4)
                gold_core_avg_temp = "{:.2f}".format(np.array(self.thermals[4:8]).sum()/4)
                silver_core_avg_freq = "{:.2f}".format(np.array(self.core_freq[:4]).sum()/4)
                gold_core_avg_freq = "{:.2f}".format(np.array(self.core_freq[4:8]).sum()/4)
                print(f"Ts: {silver_core_avg_temp}°C, Tg: {gold_core_avg_temp}°C, Fs: {silver_core_avg_freq}MHz, Fg: {gold_core_avg_freq}MHz, ram: {self.ram}MB, cpu%: {self.cpu_percentage}, gpu%: {self.gpu_percentage}, V: {self.V}mV, I: {self.I}mA")

    def read(self, path):
        with open(path, 'r') as fd:
            return fd.read().strip()

def main():
    perflogger = PerfLogger()
    perflogger.logger_thread()

if __name__ == "__main__":
    main()
