#! /usr/bin/env python3

import cv2

img = cv2.imread("frame.png")
while True:
    cv2.imshow("Image",img)
    cv2.waitKey(1)