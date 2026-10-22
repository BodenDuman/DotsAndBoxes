import cv2
import numpy as np

img = cv2.imread("/home/project26-group6/ros/src/DotsAndBoxes/frame0000.jpg")
points = []

def click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(img, (x, y), 8, (0, 255, 0), -1)
        cv2.imshow("Click 4 corners: top-left, top-right, bottom-right, bottom-left", img)
        print(f"Point {len(points)}: ({x}, {y})")
        if len(points) == 4:
            print("\nDone! Your 4 corners:", points)

cv2.imshow("Click 4 corners: top-left, top-right, bottom-right, bottom-left", img)
cv2.setMouseCallback("Click 4 corners: top-left, top-right, bottom-right, bottom-left", click)
cv2.waitKey(0)
