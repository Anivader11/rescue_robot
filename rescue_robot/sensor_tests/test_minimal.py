import os
import time
import lgpio

os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")


class h:
    """Simple GPIO-backed sensor reader for two infrared sensors."""

    def __init__(self, chip=0, sensor_pins=None, en_pin=None, poll_hz=20):
        self.chip = chip
        self.sensor_pins = sensor_pins or [24, 23]
        self.en_pin = en_pin
        self.poll_hz = poll_hz
        self.poll_sleep = 1.0 / self.poll_hz
        self.handle = lgpio.gpiochip_open(self.chip)
        self._configure_pins()

    def _configure_pins(self):
        if self.handle < 0:
            raise RuntimeError(f"Unable to open GPIO chip {self.chip}: {self.handle}")

        for pin in self.sensor_pins:
            lgpio.gpio_claim_input(self.handle, pin)

        if self.en_pin is not None:
            lgpio.gpio_claim_output(self.handle, self.en_pin, 0)

    def read_sensors(self):
        return [lgpio.gpio_read(self.handle, pin) for pin in self.sensor_pins]

    def poll(self, iterations=None):
        count = 0
        while iterations is None or count < iterations:
            states = self.read_sensors()
            print(" ".join(str(state) for state in states))
            count += 1
            if iterations is not None and count >= iterations:
                break
            time.sleep(self.poll_sleep)

    def close(self):
        if self.handle is not None:
            lgpio.gpiochip_close(self.handle)
            self.handle = None


if __name__ == "__main__":
    try:
        sensor_reader = h(poll_hz=20)
        sensor_reader.poll(iterations=50)
    finally:
        if 'sensor_reader' in locals():
            sensor_reader.close()



