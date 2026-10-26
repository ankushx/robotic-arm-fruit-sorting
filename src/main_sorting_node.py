#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main Application: Computer Vision-Based Real-Time Fruit Sorting System
Integrates Camera Stream, YOLO/HSV Detection, Inverse Kinematics, and 6-DOF Manipulator Control.
"""

import os
import sys
import time
import cv2

# Local modules
from hardware.Arm_Lib import Arm_Lib
from kinematics.dofbot_kinematics import DofbotKinematics
from vision.fruit_detector_yolo import FruitDetectorYOLO
from vision.fruit_detector_hsv import FruitDetectorHSV
from vision.coordinate_transformer import CoordinateTransformer
from motion_planning.moveit_controller import MoveItController


class FruitSortingSystem:
    """Main coordinator for automated fruit sorting."""

    def __init__(self, camera_index=0, use_yolo=True):
        print("=" * 60)
        print("  Computer Vision-Based Robotic Arm: Real-Time Fruit Sorting")
        print("=" * 60)

        self.arm = Arm_Lib()
        self.kinematics = DofbotKinematics()
        self.transformer = CoordinateTransformer()
        self.moveit = MoveItController()

        self.use_yolo = use_yolo
        if self.use_yolo:
            self.detector = FruitDetectorYOLO()
        else:
            self.detector = FruitDetectorHSV()

        # Destination sorting bins (Cartesian coords in mm: [x, y, z])
        # Apples -> Placed into Left Bin
        # Oranges -> Placed into Right Bin
        self.bin_locations = {
            "apple": {"x": 50.0, "y": 170.0, "z": 80.0},
            "orange": {"x": 50.0, "y": -170.0, "z": 80.0}
        }

        self.camera_index = camera_index
        self.cap = None

    def initialize_system(self):
        """Move arm to home pose and initialize camera."""
        print("[System] Homing robotic arm...")
        self.arm.home(time_ms=1500)
        time.sleep(1.0)

        print(f"[System] Opening camera stream (Index {self.camera_index})...")
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            print(f"[System] Warning: Could not open camera {self.camera_index}. Testing on sample frames or fallback.")
        else:
            print("[System] Camera successfully initialized.")

    def execute_pick_and_place(self, fruit_class, pick_x, pick_y, pick_z=35.0):
        """
        Execute pick-and-place sequence:
        Approach -> Grasp -> Lift -> Transit to Bin -> Release -> Return Home
        """
        print(f"\n[Action] >>> Initiating Pick & Place for [{fruit_class.upper()}] at ({pick_x}, {pick_y}, {pick_z}) mm")

        # 1. Calculate Pick Joint Angles
        approach_angles = self.kinematics.inverse_kinematics(pick_x, pick_y, pick_z + 40.0, pitch_deg=-40.0)
        grasp_angles = self.kinematics.inverse_kinematics(pick_x, pick_y, pick_z, pitch_deg=-40.0)

        dest = self.bin_locations.get(fruit_class, self.bin_locations["apple"])
        dest_transit = self.kinematics.inverse_kinematics(dest["x"], dest["y"], dest["z"] + 50.0, pitch_deg=-30.0)
        dest_drop = self.kinematics.inverse_kinematics(dest["x"], dest["y"], dest["z"], pitch_deg=-30.0)

        if not grasp_angles or not dest_drop:
            print("[Action] Error: Target location outside reachable workspace.")
            return False

        # Open gripper prior to approach
        self.arm.gripper_open(time_ms=400)
        time.sleep(0.4)

        # 2. Hover above fruit
        print("[Action] Hovering above target...")
        self.arm.Arm_serial_servo_write6(*approach_angles, 30, time_ms=1000)
        time.sleep(1.0)

        # 3. Descend to fruit level
        print("[Action] Lowering to grasp...")
        self.arm.Arm_serial_servo_write6(*grasp_angles, 30, time_ms=800)
        time.sleep(0.8)

        # 4. Grasp fruit
        print("[Action] Clamping gripper...")
        grip_pressure = 135 if fruit_class == "apple" else 125
        self.arm.gripper_close(angle=grip_pressure, time_ms=600)
        time.sleep(0.7)

        # 5. Lift fruit
        print("[Action] Lifting...")
        self.arm.Arm_serial_servo_write6(*approach_angles, grip_pressure, time_ms=800)
        time.sleep(0.8)

        # 6. Transit to destination bin
        print(f"[Action] Moving to {fruit_class} destination bin...")
        self.arm.Arm_serial_servo_write6(*dest_transit, grip_pressure, time_ms=1200)
        time.sleep(1.2)

        self.arm.Arm_serial_servo_write6(*dest_drop, grip_pressure, time_ms=800)
        time.sleep(0.8)

        # 7. Release fruit
        print("[Action] Releasing fruit...")
        self.arm.gripper_open(time_ms=500)
        time.sleep(0.6)

        # 8. Return to Home standby
        print("[Action] Returning to Home position.")
        self.arm.home(time_ms=1200)
        time.sleep(1.2)

        print(f"[Action] Finished sorting cycle for [{fruit_class.upper()}].\n")
        return True

    def run(self):
        """Continuous detection and sorting loop."""
        self.initialize_system()
        print("[System] Ready. Press 'q' in video preview to quit.")

        cooldown_until = 0.0

        try:
            while True:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if not ret:
                        time.sleep(0.1)
                        continue
                else:
                    # Simulation frame when no physical camera attached
                    import numpy as np
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(frame, "Waiting for fruit input...", (150, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                current_time = time.time()
                detections = self.detector.detect(frame)

                # Annotate preview
                if hasattr(self.detector, "draw_detections"):
                    display_frame = self.detector.draw_detections(frame, detections)
                else:
                    display_frame = frame.copy()

                cv2.imshow("Fruit Sorting Vision - DOFBOT", display_frame)

                # Process first valid fruit if not in cooldown
                if detections and current_time > cooldown_until:
                    target = detections[0]
                    fruit_class = target["class_name"]
                    u, v = target["center"]

                    rx, ry, rz = self.transformer.pixel_to_robot_plane(u, v)
                    print(f"[Detection] Identified {fruit_class} at pixel ({u}, {v}) -> Robot ({rx}, {ry}, {rz}) mm")

                    self.execute_pick_and_place(fruit_class, rx, ry, rz)
                    cooldown_until = time.time() + 2.0  # 2 second pause before next inspection

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("\n[System] Shutdown requested by user.")
        finally:
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            self.arm.home(time_ms=1000)
            print("[System] Robotic arm safely parked.")


if __name__ == "__main__":
    app = FruitSortingSystem(camera_index=0, use_yolo=True)
    app.run()
