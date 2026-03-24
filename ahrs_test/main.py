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

def get_hardcoded_calibration():
    # Replace these numbers with the ones printed during your one-time calibration
    return {
        "gyro_offset": np.array([-0.00127724, -0.00278231, -0.00129795]),
        "accel_offset": np.array([157.45,  158.155,   0.74 ]),
        "accel_scale": np.array([167.91,  168.865,  11.88]), # Notice this handles the m/s^2 to G conversion!
        "mag_offset": np.array([ 974.875,  1010.15625,  953.75   ]),
        "mag_scale": np.array( [1036.3125,  1031.15625, 1031.375  ])
    }

def apply_calibration(raw, offset, scale):
    """
    Applies zero-offset and scaling.
    Crucially, because raw accel is in m/s^2, this math forces the output 
    into exact +/- 1.0 (Earth Gs), which is what imufusion requires.
    """
    return (raw - offset) / scale

def main() -> None:
    # 1. Initialize Hardware
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO055_I2C(i2c)

    # 2. Initialize imufusion AHRS
    # The default sample rate is 100Hz, but we will update it dynamically in the loop
    ahrs = imufusion.Ahrs()
    sample_rate = 100  # Hz
    
    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NED,  # North-East-Down frame
        0.5,       # Gain
        2000.0,    # Gyroscope range in deg/s (BNO055 supports up to 2000 dps)
        10.0,      # Acceleration rejection (degrees)
        20.0,      # Magnetic rejection (degrees)
        10 * sample_rate   # Recovery trigger period (10 seconds at 100Hz)
    )

    t_prev = time.perf_counter()
    
    # The dynamic gyro calibrator catches temperature drift during runtime
    dynamic_offset = imufusion.Offset(sample_rate)
    cal = get_hardcoded_calibration()

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
        
        # A. Apply static calibration
        # Accel goes from m/s^2 -> Gs
        # Mag gets centered and spherized
        cal_accel = apply_calibration(acc_raw, cal["accel_offset"], cal["accel_scale"])
        cal_mag   = apply_calibration(mag_raw, cal["mag_offset"], cal["mag_scale"])
        
        # B. Handle Gyroscope
        # Adafruit library outputs Gyro in Radians/sec. 
        # imufusion REQUIRES Degrees/sec.
        if gyr_raw is None:
            continue
        cal_gyro_rad = gyr_raw - cal["gyro_offset"]
        cal_gyro_deg = np.degrees(cal_gyro_rad)

        # C. Apply dynamic run-time gyro offset 
        cal_gyro_deg = dynamic_offset.update(cal_gyro_deg)

        # D. Update Fusion algorithm
        ahrs.update(cal_gyro_deg, cal_accel, cal_mag, dt)

        # Get orientation as Euler angles [roll, pitch, yaw] in degrees.
        euler = ahrs.quaternion.to_euler()

        # Get Internal State (Optional: useful to see if magnetic rejection is kicking in)
        flags = ahrs.flags
        
        print(
            f"Roll:{euler[0]:6.2f} Pitch:{euler[1]:6.2f} Yaw:{euler[2]:6.2f} | "
            f"dt: {dt:.4f}s | MagRecovery: {flags.magnetic_recovery}"
        )

        # Small sleep to prevent CPU hogging, adjust based on desired sample rate
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")