"""
tof_sensor.py
==============
Python port of the SteelBar ToF distance sensor's .ino example driver,
for use on the Raspberry Pi over I2C (smbus2) instead of Arduino Wire.

Protocol (from the vendor's example): write register 0x10, then read
back 5 bytes -- a 1-byte sequence number followed by a 4-byte
little-endian distance in millimetres. The sequence number changes each
time a new measurement is ready, which is how nextMeasurement() knows to
keep polling until a genuinely new reading shows up instead of returning
a stale one.

Shares the same I2C bus as the BLDC motor drivers (control/motors.py,
addresses 0x20 and 0x19) -- the ToF sensor's address (0x51 on this
board, confirmed with i2cdetect -y 1) doesn't collide with either.

Usage:
    from tof_sensor import SteelBarToF
    sensor = SteelBarToF()             # bus 1, address 0x51 by default
    distance_mm = sensor.next_measurement()      # blocks until a NEW reading
    distance_mm = sensor.current_measurement()   # returns immediately, may repeat
"""

import struct
import time

import smbus2

DEFAULT_ADDRESS = 0x51   # confirmed via i2cdetect -y 1 on this board
DEFAULT_BUS = 1   # I2C1 -- same bus as control/motors.py


class SteelBarToF:
    def __init__(self, address=DEFAULT_ADDRESS, bus=DEFAULT_BUS):
        self._address = address
        self._bus = smbus2.SMBus(bus)
        self.last_sequence = 0

    def _read(self, wait):
        while True:
            # Write register 0x10, then read 5 bytes back (matches the
            # .ino: beginTransmission -> write(0x10) -> endTransmission ->
            # requestFrom(address, 5)).
            msg_write = smbus2.i2c_msg.write(self._address, [0x10])
            msg_read = smbus2.i2c_msg.read(self._address, 5)
            self._bus.i2c_rdwr(msg_write, msg_read)

            data = list(msg_read)
            sequence = data[0]
            distance = struct.unpack("<i", bytes(data[1:5]))[0]   # signed, little-endian

            changed = sequence != self.last_sequence
            self.last_sequence = sequence

            if not wait or changed:
                return distance

            time.sleep(0.00001)   # 10 microseconds, matches delayMicroseconds(10) in the .ino

    def next_measurement(self):
        """Waits for the next NEW sensor measurement. Returns distance in mm."""
        return self._read(wait=True)

    def current_measurement(self):
        """Reads the current distance measurement, which may or may not
        have updated since last read. Returns distance in mm."""
        return self._read(wait=False)


if __name__ == "__main__":
    sensor = SteelBarToF()
    print("Reading ToF sensor. Ctrl-C to stop.")
    try:
        while True:
            distance = sensor.next_measurement()
            print(f"Distance: {distance} mm")
    except KeyboardInterrupt:
        print("\nStopped.")
