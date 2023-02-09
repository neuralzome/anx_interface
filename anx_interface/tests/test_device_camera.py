#!/usr/bin/env python3

import cv2
import numpy as np
from anx_interface import Anx

def camera_cb(img):
    cv2.imshow('ImageWindow', img)
    cv2.waitKey(1)

def main():
    anx = Anx()

    anx.start_device_camera(fps=30, width=480, height=640, pixel_format=0, cb=camera_cb)

    anx.wait()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
