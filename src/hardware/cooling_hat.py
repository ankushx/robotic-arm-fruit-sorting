#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yahboom RGB Cooling HAT Driver
Monitors Raspberry Pi 4B CPU temperature and controls the cooling fan & status RGB LEDs.
"""

import time
import os

try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        smbus = None


class RGBCoolingHat:
    """Interface for Yahboom RGB Cooling HAT (I2C Address 0x0D)."""

    def __init__(self, bus_num=1, i2c_addr=0x0D):
        self.i2c_addr = i2c_addr
        self.bus = None
        if smbus is not None:
            try:
                self.bus = smbus.SMBus(bus_num)
            except Exception as e:
                print(f"[CoolingHat] Warning: Unable to open I2C bus {bus_num}: {e}")

    def get_cpu_temp(self):
        """Read Raspberry Pi SoC temperature in degrees Celsius."""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_raw = f.read().strip()
                return float(temp_raw) / 1000.0
        except Exception:
            return 45.0  # Default nominal fallback

    def set_fan(self, speed_level):
        """
        Set cooling fan speed.
        :param speed_level: 0 (Off), 1 (Low), 2 (Mid), 3 (Max) or PWM percentage (0-100)
        """
        if self.bus is None:
            return
        try:
            val = max(0, min(100, int(speed_level)))
            self.bus.write_byte_data(self.i2c_addr, 0x08, val)
        except Exception as e:
            pass

    def set_rgb_effect(self, mode=1):
        """
        Set RGB lighting mode:
        1: Water light, 2: Breathing, 3: Gradient, 0: Off
        """
        if self.bus is None:
            return
        try:
            self.bus.write_byte_data(self.i2c_addr, 0x04, mode)
        except Exception:
            pass

    def auto_thermal_loop(self, poll_interval=3.0):
        """Automatic temperature control loop."""
        print("[CoolingHat] Thermal management service started.")
        while True:
            temp = self.get_cpu_temp()
            if temp > 65.0:
                self.set_fan(100)
            elif temp > 55.0:
                self.set_fan(60)
            elif temp > 48.0:
                self.set_fan(30)
            else:
                self.set_fan(0)
            time.sleep(poll_interval)


if __name__ == "__main__":
    hat = RGBCoolingHat()
    print(f"Current Raspberry Pi CPU Temperature: {hat.get_cpu_temp():.1f} °C")
    hat.set_rgb_effect(1)
