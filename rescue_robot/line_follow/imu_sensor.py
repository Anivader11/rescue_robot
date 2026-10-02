"""
imu_sensor.py
=============
Driver wrapper for the Adafruit BNO085 9-DOF IMU (Adafruit #4754).

Datasheet: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-9-dof-orientation-imu-fusion-breakout-bno085.pdf

Wiring (I2C, shares the bus with the motors and ToF sensor):
    BNO085 VIN -> Pi 3.3V
    BNO085 GND -> Pi GND
    BNO085 SDA -> Pi GPIO2 (SDA) / same bus as control/motors.py
    BNO085 SCL -> Pi GPIO3 (SCL)

    Default I2C address is 0x4A (solder the ADR jumper to switch to
    0x4B if you ever need two on the same bus). Worth noting: an
    earlier `i2cdetect -y 1` run on this robot already showed an
    unidentified device at 0x4A -- that's almost certainly this IMU,
    already wired up and enumerating correctly.

For now this just reports the RAW sensor values, same as the readings
shown in the datasheet/Adafruit learn guide: raw acceleration (m/s^2),
raw gyroscope (rad/s), and raw magnetometer (uT) -- no ramp logic, no
fused orientation, just what the sensor is measuring right now.

Install (on the Pi):
    pip install adafruit-circuitpython-bno08x adafruit-blinka

Usage:
    from imu_sensor import BNO085IMU
    imu = BNO085IMU()
    ax, ay, az = imu.read_acceleration()   # m/s^2
    gx, gy, gz = imu.read_gyro()           # rad/s
    mx, my, mz = imu.read_magnetometer()   # uT
"""

DEFAULT_ADDRESS = 0x4A  # BNO085 default I2C address


class BNO085IMU:
    """Thin wrapper around adafruit_bno08x giving raw sensor readings."""

    def __init__(self, address=DEFAULT_ADDRESS):
        import board
        import busio
        from adafruit_bno08x.i2c import BNO08X_I2C
        from adafruit_bno08x import (
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_MAGNETOMETER,
        )

        self._i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self._bno = BNO08X_I2C(self._i2c, address=address)

        # These three reports give the raw (calibrated-but-not-fused)
        # sensor values -- same numbers the datasheet shows, not the
        # rotation-vector/quaternion output.
        self._bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self._bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self._bno.enable_feature(BNO_REPORT_MAGNETOMETER)

    def read_acceleration(self):
        """Returns (x, y, z) linear acceleration in m/s^2 (includes gravity)."""
        return self._bno.acceleration

    def read_gyro(self):
        """Returns (x, y, z) angular velocity in radians/second."""
        return self._bno.gyro

    def read_magnetometer(self):
        """Returns (x, y, z) magnetic field in microtesla (uT)."""
        return self._bno.magnetic


if __name__ == "__main__":
    imu = BNO085IMU()
    print("Reading BNO085 raw values. Ctrl-C to stop.")
    try:
        while True:
            ax, ay, az = imu.read_acceleration()
            gx, gy, gz = imu.read_gyro()
            mx, my, mz = imu.read_magnetometer()

            print(
                f"Accel (m/s^2): X={ax:+6.2f} Y={ay:+6.2f} Z={az:+6.2f}  |  "
                f"Gyro (rad/s): X={gx:+6.2f} Y={gy:+6.2f} Z={gz:+6.2f}  |  "
                f"Mag (uT): X={mx:+6.2f} Y={my:+6.2f} Z={mz:+6.2f}"
            )
    except KeyboardInterrupt:
        print("\nStopped.")
