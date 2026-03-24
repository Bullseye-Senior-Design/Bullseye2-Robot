import time, board, busio, numpy as np
from adafruit_bno055 import BNO055_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO055_I2C(i2c)

print("Spin the sensor slowly in 360 degrees in all directions for 15 seconds...")
time.sleep(2)

mag_data =[]
end_time = time.time() + 15.0
while time.time() < end_time:
    mag = bno.magnetic
    if mag is not None and None not in mag:
        mag_data.append(mag)
    time.sleep(0.02)

mag_data = np.array(mag_data)
mag_offset = (np.max(mag_data, axis=0) + np.min(mag_data, axis=0)) / 2.0

print("\nCopy this array into your main script:")
print(f"HARDCODED_MAG_OFFSET = np.array([{mag_offset[0]:.2f}, {mag_offset[1]:.2f}, {mag_offset[2]:.2f}])")