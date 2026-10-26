#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera-to-Arm Coordinate Calibration & Transformation
Computes Homography matrix to transform 2D camera image coordinates (u, v)
into 3D Cartesian sorting plane coordinates (x, y, z) in the robot base frame.
"""

import cv2
import numpy as np


class CoordinateTransformer:
    """Transforms image pixels to robot workspace coordinates."""

    def __init__(self, homography_matrix=None):
        if homography_matrix is not None:
            self.H = np.array(homography_matrix, dtype=float)
        else:
            # Default calibration points for DOFBOT camera setup (640x480 resolution)
            # 4 corner pixels in image vs 4 real-world table coordinates (in mm)
            pts_src = np.array([
                [180, 120],  # Top-left pixel
                [460, 120],  # Top-right pixel
                [140, 380],  # Bottom-left pixel
                [500, 380]   # Bottom-right pixel
            ], dtype=float)

            pts_dst = np.array([
                [220.0, -80.0],  # Top-left mm (X forward, Y left)
                [220.0,  80.0],  # Top-right mm
                [130.0, -90.0],  # Bottom-left mm
                [130.0,  90.0]   # Bottom-right mm
            ], dtype=float)

            self.H, _ = cv2.findHomography(pts_src, pts_dst)

    def pixel_to_robot_plane(self, u, v, table_z=35.0):
        """
        Map image pixel (u, v) to robot Cartesian coordinate (x, y, z).
        :param u: Horizontal pixel coordinate (x)
        :param v: Vertical pixel coordinate (y)
        :param table_z: Target grasping height in mm above base
        :return: (x, y, z) in mm in the robot base reference frame
        """
        pt = np.array([[[float(u), float(v)]]], dtype=float)
        transformed = cv2.perspectiveTransform(pt, self.H)
        rx = float(transformed[0, 0, 0])
        ry = float(transformed[0, 0, 1])

        return round(rx, 1), round(ry, 1), float(table_z)
