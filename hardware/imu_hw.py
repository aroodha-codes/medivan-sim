"""
imu_hw.py -- Physical MPU6050 IMU Interface via I2C.

Uses smbus2 to read raw accelerometer and gyroscope data.
Calculates yaw, tilt, and vibration levels.
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Optional

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config import (
    IMU_I2C_ADDR, IMU_ALPHA, IMU_DT, GYRO_BIAS,
    VIB_SAFE_RMS, VIB_WARNING_RMS, VIB_DANGER_RMS,
)
from modules.imu_sim import IMUData, VibrationLevel

try:
    import smbus2
    _SMBUS_AVAILABLE = True
except ImportError:
    _SMBUS_AVAILABLE = False
    print("[WARNING] smbus2 not found. Running IMUHW in stub mode.")

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_ZOUT_H = 0x47


class IMUHW:
    """Reads physical MPU6050 via I2C on Raspberry Pi."""

    def __init__(self) -> None:
        self.yaw: float = 0.0
        self._last_time = time.time()
        self._vib_buffer: list[float] = []
        
        self.bus = None
        if _SMBUS_AVAILABLE:
            try:
                self.bus = smbus2.SMBus(1)  # standard I2C bus on Pi
                # Wake up the MPU6050
                self.bus.write_byte_data(IMU_I2C_ADDR, PWR_MGMT_1, 0)
                print("[Hardware] MPU6050 initialized via I2C.")
            except Exception as e:
                print(f"[Hardware] MPU6050 init failed: {e}")
                self.bus = None

    def _read_word_2c(self, addr: int) -> int:
        """Reads two bytes and converts to 2's complement."""
        if not self.bus:
            return 0
        high = self.bus.read_byte_data(IMU_I2C_ADDR, addr)
        low = self.bus.read_byte_data(IMU_I2C_ADDR, addr + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def get_latest(self) -> IMUData:
        """Reads physical sensors and calculates orientation."""
        now = time.time()
        dt = min(0.1, now - self._last_time)
        self._last_time = now

        if not self.bus:
            # Stub return if hardware is missing
            # FIX (MT3608 review): IMUData has no 'accel_rms' field — the
            # correct field name is 'vib_rms'. The old call raised an
            # uncaught TypeError on every read when the I2C bus was absent.
            return IMUData(
                yaw=self.yaw,
                tilt_fault=False,
                vib_level=VibrationLevel.SAFE,
                vib_rms=0.0,
            )

        try:
            # Read Accelerometer (X, Y, Z)
            acc_x = self._read_word_2c(ACCEL_XOUT_H) / 16384.0
            acc_y = self._read_word_2c(ACCEL_XOUT_H + 2) / 16384.0
            acc_z = self._read_word_2c(ACCEL_XOUT_H + 4) / 16384.0

            # Read Gyroscope Z
            gyro_z = self._read_word_2c(GYRO_ZOUT_H) / 131.0  # degrees/sec
            
            # Gyro integration for Yaw
            # We add bias correction and integrate
            d_yaw = math.radians(gyro_z - GYRO_BIAS) * dt
            self.yaw += d_yaw

            # Tilt calculation (Pitch and Roll)
            pitch = math.atan2(acc_x, math.sqrt(acc_y*acc_y + acc_z*acc_z))
            roll = math.atan2(acc_y, math.sqrt(acc_x*acc_x + acc_z*acc_z))
            tilt_fault = abs(pitch) > 0.4 or abs(roll) > 0.4

            # Vibration (RMS of XYZ minus gravity)
            acc_magnitude = math.sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z)
            vibration = abs(acc_magnitude - 1.0) * 9.81  # convert to m/s^2

            self._vib_buffer.append(vibration)
            if len(self._vib_buffer) > 10:
                self._vib_buffer.pop(0)

            rms = sum(self._vib_buffer) / max(1, len(self._vib_buffer))
            
            if rms > VIB_DANGER_RMS:
                lvl = VibrationLevel.DANGER
            elif rms > VIB_WARNING_RMS:
                lvl = VibrationLevel.WARNING
            else:
                lvl = VibrationLevel.SAFE

            # FIX (MT3608 review): 'accel_rms' -> 'vib_rms', and populate the
            # pitch/roll/accel/gyro fields the HUD and logger already expect.
            # Previously this line raised TypeError inside the try block, was
            # swallowed by the bare 'except Exception', and every reading
            # silently degraded to defaults — tilt and vibration safety looked
            # active but never fired.
            return IMUData(
                pitch=pitch,
                roll=roll,
                yaw=self.yaw,
                accel_x=acc_x * 9.81,
                accel_y=acc_y * 9.81,
                accel_z=acc_z * 9.81,
                gyro_z=gyro_z,
                vib_rms=rms,
                vib_level=lvl,
                tilt_fault=tilt_fault,
                slope_warning=abs(pitch) > 0.25,
            )
        except Exception as e:
            # Surface the fault instead of hiding it.
            print(f"[Hardware] MPU6050 read error: {e}")
            return IMUData(yaw=self.yaw)

    def snap_yaw(self, new_yaw: float) -> None:
        """Force the IMU yaw to a specific value (used for landmark correction)."""
        self.yaw = new_yaw

    def start(self) -> None:
        pass  # In HW, we just read synchronously or start a thread if needed
        
    def cleanup(self) -> None:
        if self.bus:
            self.bus.close()
