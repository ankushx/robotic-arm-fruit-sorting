#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO Real-Time Fruit Object Detection Module
Performs fruit classification (e.g. apple vs orange) and localization using OpenCV DNN.
"""

import os
import cv2
import numpy as np


class FruitDetectorYOLO:
    """YOLOv4 / YOLOv4-tiny fruit object detector."""

    def __init__(self, config_path=None, weights_path=None, names_path=None, conf_threshold=0.5, nms_threshold=0.4):
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        
        self.config_path = config_path or os.path.join(base_dir, "models", "yolov4-tiny-fruit.cfg")
        self.weights_path = weights_path or os.path.join(base_dir, "models", "yolov4-tiny-fruit.weights")
        self.names_path = names_path or os.path.join(base_dir, "models", "classes.names")

        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

        # Load class labels
        self.classes = []
        if os.path.exists(self.names_path):
            with open(self.names_path, "r") as f:
                self.classes = [line.strip() for line in f.readlines() if line.strip()]
        else:
            self.classes = ["apple", "orange"]

        # Initialize network
        self.net = None
        self.output_layers = []
        if os.path.exists(self.config_path) and os.path.exists(self.weights_path):
            try:
                self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weights_path)
                # Attempt to use OpenCL / GPU if available
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                
                layer_names = self.net.getLayerNames()
                out_indices = self.net.getUnconnectedOutLayers()
                if len(out_indices.shape) == 1:
                    self.output_layers = [layer_names[i - 1] for i in out_indices]
                else:
                    self.output_layers = [layer_names[i[0] - 1] for i in out_indices]
                print("[FruitDetector] YOLO model loaded successfully.")
            except Exception as e:
                print(f"[FruitDetector] Error initializing YOLO model: {e}")
        else:
            print("[FruitDetector] Note: Model weights not found on disk. Color-based detection available as fallback.")

    def detect(self, frame):
        """
        Detect fruits in a camera frame.
        :param frame: BGR input image
        :return: List of detection dictionaries:
                 [{'class_name': str, 'confidence': float, 'box': [x, y, w, h], 'center': (cx, cy)}]
        """
        if self.net is None or frame is None:
            return []

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        layer_outputs = self.net.forward(self.output_layers)

        boxes = []
        confidences = []
        class_ids = []

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])

                if confidence > self.conf_threshold:
                    center_x = int(detection[0] * w)
                    center_y = int(detection[1] * h)
                    box_w = int(detection[2] * w)
                    box_h = int(detection[3] * h)
                    box_x = int(center_x - box_w / 2)
                    box_y = int(center_y - box_h / 2)

                    boxes.append([box_x, box_y, box_w, box_h])
                    confidences.append(confidence)
                    class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        detections = []

        if len(indices) > 0:
            for i in indices.flatten():
                bx, by, bw, bh = boxes[i]
                cx = bx + bw // 2
                cy = by + bh // 2
                label = self.classes[class_ids[i]] if class_ids[i] < len(self.classes) else f"cls_{class_ids[i]}"

                detections.append({
                    "class_name": label,
                    "confidence": float(confidences[i]),
                    "box": [bx, by, bw, bh],
                    "center": (cx, cy)
                })

        return detections

    def draw_detections(self, frame, detections):
        """Annotate frame with bounding boxes and centroid coordinates."""
        annotated = frame.copy()
        for det in detections:
            x, y, w, h = det["box"]
            cx, cy = det["center"]
            label = f"{det['class_name']}: {det['confidence']:.2f}"

            color = (0, 0, 255) if "apple" in det["class_name"].lower() else (0, 165, 255)
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(annotated, label, (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(annotated, f"({cx}, {cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        return annotated
