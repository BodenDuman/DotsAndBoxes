#!/usr/bin/env python3
"""
board_detector.py — Detects human-drawn lines on the Dots and Boxes board.

Loads camera calibration and homography, watches the camera feed, and
detects which new edge the human drew. Publishes it to /human_move.

Topics:
  Sub: /cam/color/image_raw   sensor_msgs/Image
  Pub: /human_move            std_msgs/String   JSON {"r1":0,"c1":0,"r2":0,"c2":1}

Params:
  ~config_dir   (str)   path to config folder
  ~threshold    (int)   darkness threshold for ink detection (default 80)
  ~debug        (bool)  show debug window (default False)
"""

import os
import json
import yaml
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# ── Board constants ──────────────────────────────────────────────────────────

ROWS, COLS   = 3, 3          # 3x3 boxes → 4x4 dots
BOARD_PX     = 400           # warped board image size in pixels
MARGIN       = 50            # pixels from edge to first dot
SPACING      = (BOARD_PX - 2 * MARGIN) // COLS   # pixels between dots

# ── Geometry helpers ─────────────────────────────────────────────────────────

def dot_px(row, col):
    """Pixel position of dot (row, col) in the warped image."""
    x = MARGIN + col * SPACING
    y = MARGIN + row * SPACING
    return (x, y)


def all_edges():
    edges = []
    for r in range(ROWS + 1):          # horizontal
        for c in range(COLS):
            edges.append((r, c, r, c + 1))
    for r in range(ROWS):              # vertical
        for c in range(COLS + 1):
            edges.append((r, c, r + 1, c))
    return edges

ALL_EDGES = all_edges()

# ── Config loader ─────────────────────────────────────────────────────────────

def load_calibration(config_dir):
    path = os.path.join(config_dir, "camera_calibration.yaml")
    with open(path) as f:
        data = yaml.safe_load(f)
    K    = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)
    return K, dist


def load_homography(config_dir):
    path = os.path.join(config_dir, "homography.yaml")
    with open(path) as f:
        data = yaml.safe_load(f)
    H = np.array(data["homography_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    return H

# ── Edge detection ────────────────────────────────────────────────────────────

def is_edge_drawn(gray, r1, c1, r2, c2, threshold, n_samples=12):
    """Sample pixels along the edge. Returns True if dark ink is detected."""
    p1 = dot_px(r1, c1)
    p2 = dot_px(r2, c2)
    xs = np.linspace(p1[0], p2[0], n_samples).astype(int)
    ys = np.linspace(p1[1], p2[1], n_samples).astype(int)
    # clip to image bounds
    xs = np.clip(xs, 0, gray.shape[1] - 1)
    ys = np.clip(ys, 0, gray.shape[0] - 1)
    values = gray[ys, xs]
    return float(np.mean(values)) < threshold


def detect_drawn_edges(warped_gray, threshold):
    """Return set of all edges that currently have a drawn line."""
    drawn = set()
    for edge in ALL_EDGES:
        if is_edge_drawn(warped_gray, *edge, threshold):
            drawn.add(edge)
    return drawn

# ── Debug visualisation ───────────────────────────────────────────────────────

def draw_debug(warped, drawn_edges, new_edge=None):
    vis = warped.copy()
    for r in range(ROWS + 1):
        for c in range(COLS + 1):
            cv2.circle(vis, dot_px(r, c), 5, (0, 255, 0), -1)
    for edge in drawn_edges:
        r1, c1, r2, c2 = edge
        color = (0, 0, 255) if edge == new_edge else (255, 255, 255)
        cv2.line(vis, dot_px(r1, c1), dot_px(r2, c2), color, 2)
    return vis

# ── ROS node ──────────────────────────────────────────────────────────────────

class BoardDetectorNode:
    def __init__(self):
        rospy.init_node("board_detector_node")

        config_dir    = rospy.get_param("~config_dir",
                        "/home/project26-group6/ros/src/DotsAndBoxes/config")
        self.threshold = rospy.get_param("~threshold", 80)
        self.debug     = rospy.get_param("~debug", False)

        self.H = load_homography(config_dir)
        self.bridge       = CvBridge()
        self.known_edges  = set()
        self.active       = True       # False while waiting for robot to finish

        self.pub = rospy.Publisher("/human_move", String, queue_size=1)
        rospy.Subscriber("/cam/color/image_raw", Image, self._on_image)
        rospy.Subscriber("/robot_done", String, self._on_robot_done)

        rospy.loginfo("Board detector ready. Watching for human moves.")

    def _warp(self, frame):
        # Homography was calibrated on raw (distorted) pixels — do not undistort.
        return cv2.warpPerspective(frame, self.H, (BOARD_PX, BOARD_PX))

    def _on_robot_done(self, msg):
        """Re-enable detection after robot finishes drawing."""
        self.active = True
        rospy.loginfo("Robot done — watching for human move.")

    def _on_image(self, msg):
        if not self.active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        warped      = self._warp(frame)
        gray        = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        current     = detect_drawn_edges(gray, self.threshold)
        new_edges   = current - self.known_edges

        if len(new_edges) == 1:
            edge = list(new_edges)[0]
            r1, c1, r2, c2 = edge
            msg_data = json.dumps({"r1": r1, "c1": c1, "r2": r2, "c2": c2})
            self.pub.publish(msg_data)
            rospy.loginfo("Human move detected: %s", msg_data)
            self.known_edges = current
            self.active = False        # pause until robot finishes

        if self.debug:
            new_edge = list(new_edges)[0] if len(new_edges) == 1 else None
            vis = draw_debug(warped, current, new_edge)
            cv2.imshow("Board Detector", vis)
            cv2.waitKey(1)

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    BoardDetectorNode().spin()
