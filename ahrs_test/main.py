"""Fuse BNO055 accel/gyro/mag data into quaternions using imufusion."""

from __future__ import annotations
import time
import imufusion
import board
import busio
import numpy as np
from adafruit_bno055 import BNO055_I2C

def read_vector(vec: tuple[float | None, float | None, float | None] | None) -> np.ndarray | None:
    """Convert a 3-value sensor tuple to numpy array, or return None when invalid."""
    if vec is None or any(v is None for v in vec):
        return None
    return np.array(vec, dtype=float)

def main() -> None:
    # 1. Initialize Hardware
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO055_I2C(i2c)

    # 2. Initialize imufusion AHRS
    # The default sample rate is 100Hz, but we will update it dynamically in the loop
    ahrs = imufusion.Ahrs()
    
    # Configure settings (Gain, Acceleration Rejection, Magnetic Rejection)
    # 0.5 is a standard gain for general motion.
    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NED, # North-West-Up is common for robotics
        0.5,      # Gain
        10.0,     # Acceleration rejection (degrees)
        20.0,     # Magnetic rejection (degrees)
        5 * 100,   # Rejection timeout (5 seconds at 100Hz)
        recovery_trigger_period=10 * 100, # Recovery trigger period (10 seconds at 100Hz)
    )

    t_prev = time.perf_counter()

    print("Starting AHRS fusion. Press Ctrl+C to stop.")

    while True:
        # Read raw data
        acc_raw = read_vector(bno.acceleration) # m/s^2
        gyr_raw = read_vector(bno.gyro)         # rad/s (BNO055 default)
        mag_raw = read_vector(bno.magnetic)     # uT

        if acc_raw is None or gyr_raw is None or mag_raw is None:
            continue

        # Calculate Delta Time (dt)
        now = time.perf_counter()
        dt = now - t_prev
        t_prev = now

        # IMPORTANT: imufusion expects GYROSCOPE in DEGREES PER SECOND
        # BNO055 adafruit library provides RADIANS PER SECOND
        gyr_deg = np.degrees(gyr_raw)

        # Update AHRS
        # Note: We use .update() which handles 9-DOF (Gyro, Accel, Mag)
        # If you only wanted 6-DOF, you'd use .update_no_magnetometer()
        ahrs.update(gyr_deg, acc_raw, mag_raw, dt)

        # Get Quaternion
        q = ahrs.quaternion.to_matrix() # Returns [w, x, y, z]

        # Get Internal State (Optional: useful to see if magnetic rejection is kicking in)
        flags = ahrs.flags
        
        print(
            f"W:{q[0]:.3f} X:{q[1]:.3f} Y:{q[2]:.3f} Z:{q[3]:.3f} | "
            f"dt: {dt:.4f}s | MagRecovery: {flags.magnetic_recovery}"
        )

        # Small sleep to prevent CPU hogging, adjust based on desired sample rate
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")