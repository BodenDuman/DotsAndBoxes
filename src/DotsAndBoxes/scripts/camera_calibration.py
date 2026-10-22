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

calibration = {
    "camera_matrix": {
        "rows": 3,
        "cols": 3,
        "data": K.flatten().tolist()
    },
    "distortion_coefficients": {
        "rows": 1,
        "cols": 5,
        "data": dist.tolist()
    },
    "image_width":  1280,
    "image_height": 720,
}

output_path = os.path.join(CONFIG_DIR, "camera_calibration.yaml")
with open(output_path, "w") as f:
    yaml.dump(calibration, f, default_flow_style=False)

print("Saved to", output_path)
print("K =\n", K)
print("dist =", dist)
