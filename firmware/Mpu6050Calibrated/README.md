# Calibrated MPU6050 check

This is a sensor commissioning step for the requested flight controller. It cannot fly or spin motors. Keep the ESC LiPo disconnected and power the ESP32-C3 by USB. Existing motor pins 2, 3, 5 and 7 are held low. MPU wiring: VCC=3V3, GND=common ground, SDA=10, SCL=1, AD0=GND.

Select ESP32C3 Dev Module, USB CDC On Boot Enabled, and open the monitor at 115200 baud. Keep the complete sensor/frame still for at least three seconds after startup. Calibration restarts if motion or excessive variation is detected. It estimates gyro bias, preserves gravity, then displays sensor-axis roll/pitch and corrected gyro rates. It does not make the current surface into a zero-angle reference. Acceleration scale and offsets are not calibrated by this step. Yaw heading is not measured or held.

After CALIBRATED, leave it still and check that all three rates are near zero. Tilt one axis at a time to determine which sensor axis matches frame roll and pitch. Pitch is singular near +/-90 degrees; this is a tilt check for near-level flight, not an aerobatic controller. I2C failures, missing fresh data, excessive gaps and sensor saturation latch a fault until reset.

Before writing the motor mixer and pilot input, confirm the GPIO and rotation direction at each physical corner viewed from above, the sensor mounting orientation, and the control method (phone Wi-Fi, RC receiver, etc.). PID gains must then be tuned on the actual frame. The existing ESC command frame is 50 Hz; faster sensor readings do not establish faster ESC response. Loaded power stability and actual ESC response remain unverified.

References: [MPU register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), [PX4 controller tuning principles](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter).
