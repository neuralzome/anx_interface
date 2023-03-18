#!/usr/bin/env python3

import cv2
import numpy as np
from anx_interface import Anx

def camera_cb(img):
    cv2.imshow('ImageWindow', img)
    cv2.waitKey(1)

def main():
    anx = Anx()

    anx.listen_device_camera(cb=camera_cb)

    anx.wait()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
