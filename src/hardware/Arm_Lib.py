#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yahboom DOFBOT 6-DOF Robotic Arm Hardware Interface Library
Handles I2C communication with the multi-servo driver board on Raspberry Pi.
"""

import time
import math

try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        smbus = None


class Arm_Lib(object):
    """Driver library for Yahboom DOFBOT serial bus servos and I2C board."""

    def __init__(self, bus_num=1, i2c_addr=0x15):
        self.i2c_addr = i2c_addr
        self.bus = None
        if smbus is not None:
            try:
                self.bus = smbus.SMBus(bus_num)
            except Exception as e:
                print(f"[Arm_Lib] Warning: Unable to open I2C bus {bus_num}: {e}")
        else:
            print("[Arm_Lib] Warning: smbus/smbus2 library not installed. Running in simulation mode.")

        # Joint angle limits (degrees)
        self.joint_limits = {
            1: (0, 180),    # Base rotation (yaw)
            2: (0, 180),    # Shoulder pitch
            3: (0, 180),    # Elbow pitch
            4: (0, 180),    # Wrist pitch
            5: (0, 180),    # Wrist roll
            6: (0, 180)     # Gripper clamp (0: open, 180: closed)
        }
        self.current_angles = [90, 90, 90, 90, 90, 30]

    def _write_i2c_block(self, reg, data):
        """Helper to send register data over I2C."""
        if self.bus is None:
            return
        try:
            self.bus.write_i2c_block_data(self.i2c_addr, reg, data)
        except Exception as e:
            print(f"[Arm_Lib] I2C Write Error: {e}")

    def Arm_serial_servo_write(self, servo_id, angle, time_ms=500):
        """
        Move a single servo to a target angle within a specific duration.
        :param servo_id: Servo ID 1 to 6
        :param angle: Target angle in degrees (0 - 180)
        :param time_ms: Duration of motion in milliseconds
        """
        if servo_id not in self.joint_limits:
            raise ValueError(f"Invalid servo ID: {servo_id}. Must be between 1 and 6.")

        min_ang, max_ang = self.joint_limits[servo_id]
        clamped_angle = int(max(min_ang, min(max_ang, angle)))
        time_ms = int(max(10, min(5000, time_ms)))

        # Format byte packet: [id, angle, time_high, time_low]
        time_h = (time_ms >> 8) & 0xFF
        time_l = time_ms & 0xFF
        self._write_i2c_block(0x11, [servo_id, clamped_angle, time_h, time_l])
        self.current_angles[servo_id - 1] = clamped_angle

    def Arm_serial_servo_write6(self, s1, s2, s3, s4, s5, s6, time_ms=1000):
        """
        Synchronously drive all 6 servos to specified angles.
        """
        angles = [s1, s2, s3, s4, s5, s6]
        clamped = []
        for i, a in enumerate(angles):
            sid = i + 1
            min_ang, max_ang = self.joint_limits[sid]
            clamped.append(int(max(min_ang, min(max_ang, a))))

        time_h = (time_ms >> 8) & 0xFF
        time_l = time_ms & 0xFF
        payload = clamped + [time_h, time_l]
        self._write_i2c_block(0x1E, payload)
        self.current_angles = clamped

    def Arm_serial_set_torque(self, enable=True):
        """Enable or disable servo torque (free movement / holding position)."""
        val = 1 if enable else 0
        self._write_i2c_block(0x1A, [val])

    def Arm_RGB_set(self, red, green, blue):
        """Control onboard RGB status LED on the expansion board."""
        r = max(0, min(255, int(red)))
        g = max(0, min(255, int(green)))
        b = max(0, min(255, int(blue)))
        self._write_i2c_block(0x02, [r, g, b])

    def gripper_open(self, time_ms=400):
        """Open the mechanical claw."""
        self.Arm_serial_servo_write(6, 30, time_ms)

    def gripper_close(self, angle=140, time_ms=500):
        """Close the mechanical claw to grip fruit (pressure adjustable)."""
        self.Arm_serial_servo_write(6, angle, time_ms)

    def home(self, time_ms=1200):
        """Move the robotic arm to its default home standby pose."""
        self.Arm_serial_servo_write6(90, 90, 90, 90, 90, 30, time_ms)
        time.sleep(time_ms / 1000.0)
