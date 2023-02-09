#!/usr/bin/env python3

import time

class Rate:
    def __init__(self, hz):
        self.__hz = hz
        self.__time = time.time()

    def sleep(self):
        sleep_duration = max(
            0,
            (1/self.__hz) - (time.time() - self.__time)
        )
        time.sleep(sleep_duration)
        self.__time = time.time()

