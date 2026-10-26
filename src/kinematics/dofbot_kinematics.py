#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kinematics Analysis Module for 6-DOF DOFBOT Manipulator
Implements Forward Kinematics (FK) via Denavit-Hartenberg (D-H) matrices
and Geometric/Analytical Inverse Kinematics (IK) for Cartesian target positioning.
"""

import math
import numpy as np


class DofbotKinematics:
    """
    Kinematic model for 6-DOF DOFBOT manipulator.
    Link parameters (in mm):
      - d1: Base height to shoulder axis = 104.0 mm
      - a2: Upper arm link length = 82.85 mm
      - a3: Forearm link length = 82.85 mm
      - d5: Wrist to end-effector gripper tip = 128.0 mm
    """

    def __init__(self, d1=104.0, a2=82.85, a3=82.85, d5=128.0):
        self.d1 = float(d1)
        self.a2 = float(a2)
        self.a3 = float(a3)
        self.d5 = float(d5)

    def _dh_transform(self, theta_rad, d, a, alpha_rad):
        """Construct individual D-H homogeneous transformation matrix."""
        ct = math.cos(theta_rad)
        st = math.sin(theta_rad)
        ca = math.cos(alpha_rad)
        sa = math.sin(alpha_rad)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1]
        ], dtype=float)

    def forward_kinematics(self, joint_angles_deg):
        """
        Compute end-effector Cartesian pose (x, y, z) from joint angles in degrees.
        :param joint_angles_deg: List of joint angles [theta1, theta2, theta3, theta4, theta5]
        :return: (x, y, z) end-effector position in mm
        """
        th = [math.radians(a) for a in joint_angles_deg]

        # Standard DOFBOT D-H parameters:
        # Link 1: th1, d=d1, a=0, alpha=pi/2
        # Link 2: th2, d=0,  a=a2, alpha=0
        # Link 3: th3, d=0,  a=a3, alpha=0
        # Link 4: th4, d=0,  a=0,  alpha=pi/2
        # Link 5: th5, d=d5, a=0,  alpha=0
        T01 = self._dh_transform(th[0], self.d1, 0, math.pi / 2)
        T12 = self._dh_transform(th[1], 0, self.a2, 0)
        T23 = self._dh_transform(th[2], 0, self.a3, 0)
        T34 = self._dh_transform(th[3], 0, 0, math.pi / 2)
        T45 = self._dh_transform(th[4], self.d5, 0, 0)

        T05 = T01 @ T12 @ T23 @ T34 @ T45
        pos = T05[0:3, 3]
        return float(pos[0]), float(pos[1]), float(pos[2])

    def inverse_kinematics(self, x, y, z, pitch_deg=-45.0):
        """
        Analytical Inverse Kinematics solver.
        Computes joint angles (in degrees) to place the gripper at target Cartesian coordinate (x, y, z).
        :param x: Target X in mm (Forward)
        :param y: Target Y in mm (Left/Right)
        :param z: Target Z in mm (Height from base)
        :param pitch_deg: Gripper approach pitch angle (default: -45 deg for angled fruit grasping)
        :return: List of joint angles [th1, th2, th3, th4, th5] in degrees, or None if unreachable.
        """
        # Joint 1: Base azimuth
        th1 = math.degrees(math.atan2(y, x))
        r_total = math.hypot(x, y)

        # Wrist center position (subtract end-effector extension)
        pitch_rad = math.radians(pitch_deg)
        r_wrist = r_total - self.d5 * math.cos(pitch_rad)
        z_wrist = z - self.d1 - self.d5 * math.sin(pitch_rad)

        dist_sq = r_wrist**2 + z_wrist**2
        dist = math.sqrt(dist_sq)

        # Reachability check (triangle inequality)
        if dist > (self.a2 + self.a3) or dist < abs(self.a2 - self.a3):
            return None  # Target is outside workspace envelope

        # Joint 3: Elbow pitch using Law of Cosines
        cos_th3 = (dist_sq - self.a2**2 - self.a3**2) / (2.0 * self.a2 * self.a3)
        cos_th3 = max(-1.0, min(1.0, cos_th3))
        th3_rad = -math.acos(cos_th3)  # Elbow-up configuration

        # Joint 2: Shoulder pitch
        phi = math.atan2(z_wrist, r_wrist)
        psi = math.atan2(self.a3 * math.sin(th3_rad), self.a2 + self.a3 * math.cos(th3_rad))
        th2_rad = phi - psi

        # Joint 4: Wrist pitch to achieve target orientation
        th4_rad = pitch_rad - (th2_rad + th3_rad)

        # Convert to degrees and map to 0-180 degree servo frames
        th1_deg = 90.0 + th1
        th2_deg = 90.0 - math.degrees(th2_rad)
        th3_deg = 90.0 - math.degrees(th3_rad)
        th4_deg = 90.0 - math.degrees(th4_rad)
        th5_deg = 90.0  # Default neutral roll

        angles = [th1_deg, th2_deg, th3_deg, th4_deg, th5_deg]

        # Clamp check against physical servo boundaries
        for a in angles:
            if a < 0 or a > 180:
                return None

        return [round(a, 1) for a in angles]


if __name__ == "__main__":
    solver = DofbotKinematics()
    target = (180.0, 0.0, 50.0)
    print(f"Solving IK for target: {target} mm")
    sol = solver.inverse_kinematics(*target, pitch_deg=-30.0)
    print(f"Computed Joint Angles (deg): {sol}")
    if sol:
        fk_res = solver.forward_kinematics(sol)
        print(f"Verification via FK: {fk_res}")
