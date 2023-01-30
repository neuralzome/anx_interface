#!/usr/bin/env python3

from anx_interface import Anx

def device_state_cb(device_state):
    print(device_state)

def main():
    anx = Anx()

    anx.register_state_cb(device_state_cb)

    anx.wait()

if __name__ == "__main__":
    main()
