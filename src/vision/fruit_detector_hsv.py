#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Color/HSV Fruit Detection Fallback Module
Uses HSV color filtering, morphological operations, and contour moments
to detect red/green apples and oranges when running on lightweight hardware.
"""

import cv2
import numpy as np


class FruitDetectorHSV:
    """HSV color segmentation fruit detector."""

    def __init__(self):
        # HSV Color Boundaries
        self.color_ranges = {
            "apple_red": [
                (np.array([0, 70, 50]), np.array([10, 255, 255])),
                (np.array([170, 70, 50]), np.array([180, 255, 255]))
            ],
            "orange": [
                (np.array([11, 100, 100]), np.array([25, 255, 255]))
            ],
            "apple_green": [
                (np.array([35, 50, 50]), np.array([85, 255, 255]))
            ]
        }

    def detect(self, frame, min_area=800):
        """
        Detect colored fruit objects in frame.
        :param frame: BGR input image
        :param min_area: Minimum contour pixel area
        :return: List of detections
        """
        if frame is None:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lower, upper) in ranges:
                sub_mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, sub_mask)

            # Morphological noise removal
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > min_area:
                    x, y, w, h = cv2.boundingRect(cnt)
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        mapped_name = "apple" if "apple" in color_name else "orange"
                        detections.append({
                            "class_name": mapped_name,
                            "confidence": min(1.0, area / 5000.0),
                            "box": [x, y, w, h],
                            "center": (cx, cy)
                        })

        return detections
