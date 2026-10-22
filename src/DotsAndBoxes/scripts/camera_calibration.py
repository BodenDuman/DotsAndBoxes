#!/usr/bin/env python3
"""
Run this script to calibrate the camera and save the result to config/.
Usage: python3 scripts/camera_calibration.py
"""

import os
import subprocess
import tarfile
import yaml
import numpy as np

CONFIG_DIR = "/home/project26-group6/ros/src/DotsAndBoxes/config"
os.makedirs(CONFIG_DIR, exist_ok=True)

print("Starting calibration window...")
print("Move the checkerboard until all 4 bars turn green, then click CALIBRATE -> SAVE.")
print()

subprocess.run([
    "rosrun", "camera_calibration", "cameracalibrator.py",
    "--size", "7x7",
    "--square", "0.025",
    "--no-service-check",
    "image:=/cam/color/image_raw",
    "camera:=/cam"
])

print()
print("Extracting calibration result from /tmp/calibrationdata.tar.gz ...")

with tarfile.open("/tmp/calibrationdata.tar.gz") as tar:
    tar.extractall("/tmp")

with open("/tmp/ost.yaml") as f:
    data = yaml.safe_load(f)

K    = np.array(data["camera_matrix"]["data"]).reshape(3, 3)
dist = np.array(data["distortion_coefficients"]["data"])

np.save(os.path.join(CONFIG_DIR, "camera_matrix.npy"), K)
np.save(os.path.join(CONFIG_DIR, "dist_coeffs.npy"), dist)

print("Saved to", CONFIG_DIR)
print("K =\n", K)
print("dist =", dist)
