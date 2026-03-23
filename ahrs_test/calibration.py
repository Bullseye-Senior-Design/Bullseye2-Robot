import time
import board
import busio
import adafruit_bno055
import numpy as np
import imufusion

# ======================================================================
# 1. HARDWARE SETUP & ABSTRACTION
# ======================================================================
def setup_bno055():
    """Initializes the I2C connection and the BNO055 sensor."""
    print("Initializing I2C and BNO055...")
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    # IMPORTANT: We must bypass the BNO055's internal fusion engine to get 
    # the fastest, rawest data possible for imufusion. 
    # MODE_AMG = Accelerometer, Magnetometer, and Gyroscope only.
    sensor.mode = adafruit_bno055.AMG_MODE
    time.sleep(0.1) # Give it a moment to change modes
    return sensor

def read_bno055_raw(sensor):
    """
    Safely reads raw data. The BNO055 over I2C (especially on Raspberry Pi) 
    can occasionally drop a frame or return None. This handles that gracefully.
    
    Returns:
        accel (m/s^2), gyro (rad/s), mag (micro-Tesla) as numpy arrays,
        or None, None, None if the read failed.
    """
    try:
        accel = sensor.acceleration
        gyro = sensor.gyro
        mag = sensor.magnetic
        
        # Check if any readings failed (returned None)
        if None in (accel, gyro, mag) or None in accel or None in gyro or None in mag:
            return None, None, None
            
        return np.array(accel), np.array(gyro), np.array(mag)
    except Exception as e:
        # Catch I2C errors (like clock stretching issues)
        return None, None, None

# ======================================================================
# 2. CALIBRATION ROUTINES
# ======================================================================
def interactive_calibration(sensor):
    """Guides the user physically to calibrate the IMU from scratch."""
    print("\n" + "="*40)
    print(" IMU CALIBRATION STARTING")
    print("="*40)
    
    cal_data = {
        "gyro_offset": np.zeros(3),
        "accel_offset": np.zeros(3),
        "accel_scale": np.ones(3),
        "mag_offset": np.zeros(3),
        "mag_scale": np.ones(3)
    }

    # --- A. Gyroscope Calibration ---
    print("\n1. GYROSCOPE CALIBRATION")
    print("Place the sensor on a solid surface and DO NOT TOUCH IT.")
    for i in range(3, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)
        
    print("Recording gyro for 3 seconds...")
    gyro_samples =[]
    end_time = time.time() + 3.0
    while time.time() < end_time:
        _, gyro, _ = read_bno055_raw(sensor)
        if gyro is not None:
            gyro_samples.append(gyro)
        time.sleep(0.01)
        
    cal_data["gyro_offset"] = np.mean(gyro_samples, axis=0)
    print(f"-> Gyro Offset: {cal_data['gyro_offset']}")

    # --- B. Accelerometer Calibration ---
    print("\n2. ACCELEROMETER CALIBRATION")
    print("Slowly tumble the sensor in ALL directions (like a sphere).")
    print("Ensure every side (X, Y, Z) points perfectly down and up.")
    input("Press Enter to begin recording for 15 seconds...")
    
    accel_samples =[]
    end_time = time.time() + 15.0
    while time.time() < end_time:
        accel, _, _ = read_bno055_raw(sensor)
        if accel is not None:
            accel_samples.append(accel)
        time.sleep(0.01)
        
    a_min = np.min(accel_samples, axis=0)
    a_max = np.max(accel_samples, axis=0)
    cal_data["accel_offset"] = (a_max + a_min) / 2.0
    cal_data["accel_scale"] = (a_max - a_min) / 2.0
    print(f"-> Accel Offset: {cal_data['accel_offset']}")
    print(f"-> Accel Scale:  {cal_data['accel_scale']}")

    # --- C. Magnetometer Calibration ---
    print("\n3. MAGNETOMETER CALIBRATION")
    print("Wave the sensor in a slow Figure-8 motion in the air.")
    print("Rotate it around all 3 axes while waving.")
    input("Press Enter to begin recording for 15 seconds...")
    
    mag_samples =[]
    end_time = time.time() + 15.0
    while time.time() < end_time:
        _, _, mag = read_bno055_raw(sensor)
        if mag is not None:
            mag_samples.append(mag)
        time.sleep(0.01)
        
    m_min = np.min(mag_samples, axis=0)
    m_max = np.max(mag_samples, axis=0)
    cal_data["mag_offset"] = (m_max + m_min) / 2.0
    cal_data["mag_scale"] = (m_max - m_min) / 2.0
    print(f"-> Mag Offset: {cal_data['mag_offset']}")
    print(f"-> Mag Scale:  {cal_data['mag_scale']}")

    print("\n--- CALIBRATION COMPLETE ---")
    return cal_data

# ======================================================================
# 3. HELPER MATH
# ======================================================================
def apply_calibration(raw, offset, scale):
    """
    Applies zero-offset and scaling.
    Crucially, because raw accel is in m/s^2, this math forces the output 
    into exact +/- 1.0 (Earth Gs), which is what imufusion requires.
    """
    return (raw - offset) / scale

# ======================================================================
# 4. MAIN FUSION LOOP
# ======================================================================
def run_ahrs():
    # 1. Hardware setup
    sensor = setup_bno055()

    # 2. Interactive calibration
    cal = interactive_calibration(sensor)

    # 3. Initialize IMUFusion AHRS
    ahrs = imufusion.Ahrs()
    sample_rate = 100 # Target Hz
    
    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NWU,  # North-West-Up
        0.5,    # Filter gain
        2000,   # Gyroscope range (deg/s)
        10,     # Acceleration rejection
        10,     # Magnetic rejection
        5 * sample_rate # Recovery trigger
    )

    # The dynamic gyro calibrator catches temperature drift during runtime
    dynamic_offset = imufusion.Offset(sample_rate)

    print("\nStarting AHRS Fusion loop... (Press Ctrl+C to stop)")
    prev_time = time.time()

    try:
        while True:
            current_time = time.time()
            dt = current_time - prev_time
            
            # Read sensor
            raw_accel, raw_gyro, raw_mag = read_bno055_raw(sensor)
            
            # Skip loop if I2C read failed
            if raw_accel is None:
                continue

            prev_time = current_time

            # A. Apply static calibration
            # Accel goes from m/s^2 -> Gs
            # Mag gets centered and spherized
            cal_accel = apply_calibration(raw_accel, cal["accel_offset"], cal["accel_scale"])
            cal_mag   = apply_calibration(raw_mag, cal["mag_offset"], cal["mag_scale"])
            
            # B. Handle Gyroscope
            # Adafruit library outputs Gyro in Radians/sec. 
            # imufusion REQUIRES Degrees/sec.
            if raw_gyro is None:
                continue
            cal_gyro_rad = raw_gyro - cal["gyro_offset"]
            cal_gyro_deg = np.degrees(cal_gyro_rad)

            # C. Apply dynamic run-time gyro offset 
            cal_gyro_deg = dynamic_offset.update(cal_gyro_deg)

            # D. Update Fusion algorithm
            ahrs.update(cal_gyro_deg, cal_accel, cal_mag, dt)

            # E. Output results
            euler = ahrs.quaternion.to_euler()
            print(f"Roll: {euler[0]:6.1f} | Pitch: {euler[1]:6.1f} | Yaw: {euler[2]:6.1f}", end='\r')
            
            # Brief sleep to yield CPU, adjusting to hit ~100Hz
            elapsed = time.time() - current_time
            time.sleep(max(0, (1.0 / sample_rate) - elapsed))

    except KeyboardInterrupt:
        print("\n\nFusion stopped by user.")

if __name__ == "__main__":
    run_ahrs()