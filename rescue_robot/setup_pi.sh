#!/bin/bash
# setup_pi.sh
# ===========
# Run once on the Pi after flashing to install all dependencies.
# Usage: bash setup_pi.sh
#
# Safe to re-run — all steps are idempotent.

set -e
echo ""
echo "=========================================="
echo " Rescue Robot Pi Setup"
echo "=========================================="
echo ""

# 1. System packages
echo "[1/5] Installing system packages..."
sudo apt update -y
sudo apt install -y python3-opencv python3-pip python3-picamera2 git i2c-tools

# 2. Enable I2C
echo "[2/5] Enabling I2C..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_camera 0

# 3. Set I2C speed to 1MHz for BLDC driver
echo "[3/5] Setting I2C speed to 1MHz..."
CONFIG=/boot/firmware/config.txt
LINE="dtoverlay=i2c1,pins_2_3,baudrate=1000000"
if grep -qF "$LINE" "$CONFIG"; then
    echo "  Already set — skipping"
else
    echo "$LINE" | sudo tee -a "$CONFIG"
    echo "  Added to $CONFIG"
fi

# 4. CircuitPython and BLDC library
echo "[4/5] Installing CircuitPython and BLDC motor library..."

# CircuitPython base
pip install adafruit-blinka --break-system-packages 2>/dev/null || true

# Activate env if it exists, otherwise install globally
if [ -d "$HOME/env" ]; then
    echo "  Virtual env found at ~/env — installing there"
    source "$HOME/env/bin/activate"
    pip install adafruit-blinka
    pip install git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver.git
else
    echo "  Installing with --break-system-packages"
    pip install adafruit-blinka --break-system-packages
    pip install git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver.git --break-system-packages
fi

# 5. numpy
echo "[5/5] Installing numpy..."
pip install numpy --break-system-packages 2>/dev/null || true

echo ""
echo "=========================================="
echo " Setup complete — rebooting in 5 seconds"
echo " (Ctrl-C to cancel reboot)"
echo "=========================================="
sleep 5
sudo reboot
