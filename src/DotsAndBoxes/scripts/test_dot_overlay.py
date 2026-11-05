#!/usr/bin/env python3
"""
Run this to check if the dot positions in the vision node
match the actual dots on the board.

Red circles = where the code expects dots to be.
If they don't align with the real dots, adjust MARGIN and SPACING.

Usage: python3 scripts/test_dot_overlay.py
"""

import cv2
import numpy as np
import yaml
import glob
import os

CONFIG_DIR  = "/home/project26-group6/ros/src/DotsAndBoxes/config"
PROJECT_DIR = "/home/project26-group6/ros/src/DotsAndBoxes"

BOARD_PX = 400
MARGIN   = 50
SPACING  = 75    # (400 - 2*50) / 4 = 75px for 5x5 dots
ROWS     = 4   # number of boxes
COLS     = 4   # number of boxes

# Load homography
with open(os.path.join(CONFIG_DIR, "homography.yaml")) as f:
    data = yaml.safe_load(f)
H = np.array(data["homography_matrix"]["data"]).reshape(3, 3)

# Load latest captured frame
frames = sorted(glob.glob(os.path.join(PROJECT_DIR, "frame*.jpg")))
if not frames:
    print("No frame found. Run this first:")
    print("  rosrun image_view extract_images image:=/cam/color/image_raw _sec_per_frame:=2")
    exit()

img    = cv2.imread(frames[-1])
warped = cv2.warpPerspective(img, H, (BOARD_PX, BOARD_PX))

# Draw expected dot positions
for r in range(ROWS + 1):
    for c in range(COLS + 1):
        x = MARGIN + c * SPACING
        y = MARGIN + r * SPACING
        cv2.circle(warped, (x, y), 8, (0, 0, 255), -1)
        cv2.putText(warped, f"({r},{c})", (x + 5, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

output = "/tmp/dot_overlay.jpg"
cv2.imwrite(output, warped)
print(f"Saved to {output}")
print("Opening image...")
os.system(f"eog {output}")
