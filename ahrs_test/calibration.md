


As an experienced computer engineer who has spent a lot of time working with IMUs, embedded systems, and sensor fusion filters (like Kalman, Mahony, and Madgwick), I can give you a very direct answer: **The algorithm itself is only 20% of the equation; the other 80% is how you prepare the data and tune the parameters.** 

The `imufusion` library is excellent—it uses Madgwick's *revised* algorithm (which handles magnetic and linear acceleration anomalies much better than his original, more famous algorithm). However, if you just feed raw sensor data into it with default settings, you will get mediocre results.

Here is my honest, step-by-step engineering guide to getting the absolute best fused output from this library, along with my opinions on the pitfalls you will face.

---

### Step 1: Calibration is Mandatory (Garbage In = Garbage Out)
The documentation explicitly states: *"The library does not provide a method to determine these parameters."* This is the biggest hurdle in sensor fusion. 

**How to do it best:**
Before feeding data into the AHRS algorithm, you *must* pass your raw data through `FusionModelInertial` (for gyro/accel) and `FusionModelMagnetic` (for the magnetometer).
*   **Problem/Opinion:** Many developers skip magnetometer soft-iron and hard-iron calibration because it requires moving the sensor in a figure-8 pattern and doing ellipsoid fitting. If you skip this, your heading (yaw) will drift wildly or lock onto false magnetic Norths. Magnetic rejection won't save you from a poorly calibrated sensor; it only saves you from *temporary* external magnets (like walking past a speaker). 
*   **Actionable Advice:** Use an external tool (like MotionCal or Magneto) to calculate your $\mathbf{M}$, $\mathbf{s}$, $\mathbf{b}$, $\mathbf{S}$, and $\mathbf{h}$ matrices. Feed these into the library's sensor models.

### Step 2: Ensure Axis Alignment
Your Gyro, Accelerometer, and Magnetometer might be different silicon dies inside your IMU package, meaning their X, Y, and Z axes might not align naturally. 
*   **How to do it best:** Use the `FusionRemap` function to align all sensors to a standard body frame convention (like North-East-Down or East-North-Up) *before* fusion. If an acceleration on the X-axis doesn't match a rotation around the Y/Z axis perfectly, the math collapses.

### Step 3: Utilize the Bias Algorithm Properly
Gyroscopes drift based on temperature and time. The `imufusion` library includes a `FusionBias` algorithm that updates the gyro offset during stationary periods.
*   **Problem/Opinion:** If your device reboots, it loses this newly calculated offset. The 3-second default startup period isn't always enough to perfectly zero out a bad offset.
*   **Actionable Advice:** Use `FusionBiasGetOffset` to periodically save the calculated bias to your microcontroller's Non-Volatile Memory (EEPROM/Flash). On boot, read it back and inject it using `FusionBiasSetOffset`. This ensures your device boots with maximum accuracy immediately.

### Step 4: Tuning the AHRS Settings (The Black Magic)
You must configure `FusionAhrsSettings` based on your specific physical application. Defaults are rarely optimal.

*   **Gain (Default 0.5):** 
    *   *My Opinion:* A gain of 0.5 is often too high for highly dynamic systems (like racing drones) and too low for slow systems (like a ship's compass). 
    *   *Why:* Gain controls the "trust" ratio. If your device vibrates heavily, trust the gyroscope more (lower the gain, e.g., 0.1). If your device moves slowly but drifts over time, trust the accelerometer/magnetometer more (raise the gain, e.g., 0.8).
*   **Gyroscope Range:**
    *   *Actionable Advice:* Set this *exactly* to what you configured your hardware IMU to (e.g., 2000 deg/sec). The library uses this to trigger "Angular rate recovery." If you set this wrong, the algorithm will restart arbitrarily when it thinks it's maxed out, causing massive glitches in your output.
*   **Rejection Thresholds (Default 10°):**
    *   *My Opinion:* The 10-degree threshold for acceleration and magnetic rejection is a great starting point, but you should log `FusionAhrsGetInternalStates` during real-world testing. Look at the `accelerationError`. If normal operation of your device frequently exceeds 10° (e.g., a fast robotic arm), increase the threshold so the algorithm doesn't constantly ignore the accelerometer.

### Step 5: Ditch the "Fast Inverse Square Root" Hack
The library defaults to using Pizer's Fast Inverse Square Root (the famous algorithm from Quake III). 
*   **Problem/Opinion:** In 2026, if you are running this on an embedded system, you are likely using a 32-bit ARM Cortex-M4F, M33, or M7 processor, or an ESP32. These chips have Hardware Floating-Point Units (FPUs).
*   **Actionable Advice:** Add the `#define FUSION_USE_NORMAL_SQRT` to your compiler flags. Hardware square root instructions (`VSQRT.F32` on ARM) execute in just a few clock cycles. Pizer's hack relies on bit-level manipulation and Newton iterations that can actually be *slower* and *less precise* on modern hardware than just asking the FPU to do its job. Take the accuracy bump; you won't notice a performance hit on modern MCUs.

### Summary Checklist for "Best" Output:
1. Calibrate hard/soft iron and gyro offsets externally; feed them to `FusionModel*`.
2. Remap all sensors so X, Y, and Z point in the exact same physical directions.
3. Save dynamic gyro bias to EEPROM and load it on boot.
4. Set the `gyroscopeRange` to exactly match your IMU's hardware registers.
5. Profile your application's physical dynamics to tune `gain` and `rejection` thresholds.
6. Enable `FUSION_USE_NORMAL_SQRT` if your microcontroller has a hardware FPU.

As an engineer, I’ll give it to you straight: figuring out the parameters for `FusionModelInertial` ($\mathbf{M}$, $\mathbf{s}$, and $\mathbf{b}$) is the most tedious part of sensor fusion, but it is the absolute difference between a professional product and a hobbyist toy.

The `imufusion` library does the math, but **you have to calculate these matrices yourself** through physical calibration.

Here is exactly how you get each parameter for both your Accelerometer and your Gyroscope, ranging from the "quick and dirty" method to the "professional" method.

---

### Understanding the Parameters

You need three variables to feed the equation $\mathbf{i}_c = \mathbf{M} \mathbf{s} (\mathbf{i}_u - \mathbf{b})$:

1.  **$\mathbf{b}$ (Offset Vector):** The "Zero-bias" error. What does the sensor read when it is experiencing absolutely zero force/rotation?
2.  **$\mathbf{s}$ (Sensitivity Matrix):** The scale factor. A 3x3 diagonal matrix. If gravity is pulling at 1 _g_, does the sensor actually report exactly 1 _g_?
3.  **$\mathbf{M}$ (Misalignment Matrix):** A 3x3 matrix. The microscopic silicon MEMS structures inside the chip are never perfectly 90 degrees to each other. If you accelerate purely on the X-axis, some of that force "leaks" into the Y and Z axes. $\mathbf{M}$ corrects this non-orthogonality.

---

### Part 1: Calibrating the Accelerometer

#### The "Good Enough" Method (Datasheet + Offset)

If you are in a rush and don't need sub-degree precision:

- **$\mathbf{M}$:** Set this to a 3x3 Identity Matrix (1s on the diagonal, 0s everywhere else). Assume the axes are perfectly 90 degrees.
- **$\mathbf{s}$:** Read your IMU's datasheet. If you set your IMU to $\pm2g$ mode, the datasheet usually says the scale factor is `16384 LSB/g`. Divide your raw integer by 16384 to get _g_. Put `1.0` on the diagonal of the $\mathbf{s}$ matrix since you scaled it in code.
- **$\mathbf{b}$:** Lay the IMU flat on a table. Read the X and Y axes. They _should_ be 0. If X reads `0.03g`, your X offset is `0.03`. (Z is harder because it's measuring 1g from gravity, so you have to flip it upside down and average the difference).

#### The Professional Method: The 6-Point Tumble Test

To get the absolute best $\mathbf{M}$, $\mathbf{s}$, and $\mathbf{b}$ matrices, you must map the accelerometer to a perfect sphere of gravity (1 _g_ in all directions).

1.  **Data Collection:** Mount your IMU inside a perfectly square 3D-printed cube or a machined block.
2.  Rest the block on a flat, level table on **Face 1** (+Z pointing up). Keep it perfectly still for 3 seconds. Record the average raw X, Y, and Z data.
3.  Repeat this for all 6 faces (+X, -X, +Y, -Y, +Z, -Z).
4.  **The Math (Ellipsoid Fitting):** Plotting those 6 points in 3D space _should_ create a perfect sphere with a radius of 1. Because of sensor errors, it will actually be an off-center, tilted egg (an ellipsoid).
5.  **The Tools:** Do not write this math from scratch. Use an open-source tool to calculate the ellipsoid fit.
    - _Recommended Tool:_ **Magneto 1.2** (an old but gold C program/GUI) or Python's `scipy.optimize.least_squares`. You feed it your 6 static data points, tell it the expected norm is `1.0` (for 1 _g_), and it will spit out the exact $\mathbf{b}$ vector and a combined 3x3 matrix that represents $(\mathbf{M} \times \mathbf{s})$.

---

### Part 2: Calibrating the Gyroscope

Gyros measure rotation (degrees per second). Gravity does not affect them, which makes them easier to calibrate for offset, but harder to calibrate for sensitivity.

#### Getting the Offset ($\mathbf{b}$) - _Crucial_

You **must** do this. If your gyro has a zero-bias error, the fusion algorithm will integrate that error over time, and your heading will drift endlessly.

1.  Leave the IMU completely motionless on a table for 5 to 10 seconds.
2.  Average the raw data for X, Y, and Z.
3.  The average value is your $\mathbf{b}$ vector.
    _Note: As mentioned previously, `imufusion` has a built-in `FusionBias` algorithm that does exactly this dynamically at runtime when it detects the device is sitting still._

#### Getting Sensitivity ($\mathbf{s}$) and Misalignment ($\mathbf{M}$)

- **My Opinion:** For 95% of applications, **ignore Gyro $\mathbf{s}$ and $\mathbf{M}$.** Set $\mathbf{s}$ based purely on the datasheet scale factor (e.g., `16.4 LSB/dps` for a $\pm2000$ dps range) and set $\mathbf{M}$ to the Identity Matrix.
- **Why?** To actually calibrate gyro sensitivity and cross-axis misalignment, you need a highly precise motorized rate table (a machine that spins the IMU at exactly, say, 100.00 RPM on a single axis). Unless you are building aerospace avionics, you don't have a $10,000 rate table. Relying on the factory silicon calibration for the gyro's scale factor is universally accepted practice in embedded consumer electronics.

---

### Summary of What to Feed `FusionModelInertial`:

**For the Accelerometer:**

- $\mathbf{b}$: Derived from the 6-point tumble test (ellipsoid center offset).
- $\mathbf{s}$: The scale factor derived from the 6-point tumble test (ellipsoid radii).
- $\mathbf{M}$: The cross-axis matrix derived from the tumble test (ellipsoid tilt).

**For the Gyroscope:**

- $\mathbf{b}$: Measured dynamically at startup or via `FusionBias` (stationary average).
- $\mathbf{s}$: Derived directly from the manufacturer's datasheet LSB/dps conversion.
- $\mathbf{M}$: `[1, 0, 0; 0, 1, 0; 0, 0, 1]` (Identity matrix).

**Actionable Advice:** If you are using Python, look up a library called `imu-calibration` or write a quick script using `scipy` to calculate the accelerometer ellipsoid fit. If you use C/C++, search GitHub for "IMU ellipsoid fit" and just pass your raw 6-point data through it once, hardcode the resulting matrices into your firmware as constants, and you're done.

---

## BNO055 Magnetometer Capture for Magneto

Use the helper script below to log raw magnetometer samples from the BNO055 over I2C.

### Command

```bash
python ahrs_test/magnetometer_calibration.py --duration 120 --rate 30
```

### Output

- The script writes a CSV file in `ahrs_test/` named like `magnetometer_samples_YYYYMMDD_HHMMSS.csv`.
- Default columns are `timestamp,mx,my,mz`.
- Add `--no-timestamp` if you only want `mx,my,mz`.
- Add `--include-cal-status` to include BNO055 calibration level columns.

### Capture Tips

- Slowly rotate and tumble the sensor through as many orientations as possible.
- Avoid holding it still for long periods during capture.
- Collect at least 60 to 120 seconds of data for a stable ellipsoid fit.

### Example Variants

```bash
# Magneto-only columns (mx,my,mz)
python ahrs_test/magnetometer_calibration.py --duration 90 --rate 25 --no-timestamp

# Save to a specific path
python ahrs_test/magnetometer_calibration.py --output example/mag_for_magneto.csv
```
