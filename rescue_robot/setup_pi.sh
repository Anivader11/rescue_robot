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
echo "[1/6] Installing system packages..."
sudo apt update -y
sudo apt install -y python3-opencv python3-pip python3-picamera2 git i2c-tools libcamera-apps v4l-utils

# 2. Enable I2C
echo "[2/6] Enabling I2C..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_camera 0

# 3. Set I2C speed to 1MHz for BLDC driver
echo "[3/6] Setting I2C speed to 1MHz..."
CONFIG=/boot/firmware/config.txt
LINE="dtoverlay=i2c1,pins_2_3,baudrate=1000000"
if grep -qF "$LINE" "$CONFIG"; then
    echo "  Already set — skipping"
else
    echo "$LINE" | sudo tee -a "$CONFIG"
    echo "  Added to $CONFIG"
fi

# 4. CircuitPython and BLDC library
echo "[4/6] Installing CircuitPython and BLDC motor library..."

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
echo "[5/6] Installing numpy..."
pip install numpy --break-system-packages 2>/dev/null || true

# 6. Create package __init__.py files and self-test all imports
# Catches broken installs NOW instead of mid-testing later.
# Note: the pip package is "steelbar-circuitpython-powerful-bldc-driver"
# but the actual importable module name is "steelbar_powerful_bldc_driver" — different.
echo "[6/6] Copying project files into package layout and verifying..."
# The imports use control.* / vision.* packages — the files MUST be laid out
# as packages or main.py dies with ModuleNotFoundError. This copies whatever
# directory this script is run from into ~/rescue_robot with the right layout.
SRC="$(cd "$(dirname "$0")" && pwd)"
mkdir -p ~/rescue_robot/vision ~/rescue_robot/control ~/rescue_robot/tests
touch ~/rescue_robot/vision/__init__.py ~/rescue_robot/control/__init__.py ~/rescue_robot/tests/__init__.py
cp "$SRC/line_detector.py" "$SRC/vision_thread.py"           ~/rescue_robot/vision/  2>/dev/null || true
cp "$SRC/motors.py" "$SRC/state_machine.py" "$SRC/shared_vision.py" ~/rescue_robot/control/ 2>/dev/null || true
cp "$SRC/test_line_detector.py"                              ~/rescue_robot/tests/   2>/dev/null || true
cp "$SRC/main.py" "$SRC/calibrate.py" "$SRC/FILE_MAP.md"     ~/rescue_robot/         2>/dev/null || true
# If the source is already in package layout (vision/, control/ dirs), copy those too
[ -d "$SRC/vision" ]  && cp "$SRC"/vision/*.py  ~/rescue_robot/vision/  || true
[ -d "$SRC/control" ] && cp "$SRC"/control/*.py ~/rescue_robot/control/ || true
[ -d "$SRC/tests" ]   && cp "$SRC"/tests/*.py   ~/rescue_robot/tests/   || true
echo "  Files copied to ~/rescue_robot — layout:"
find ~/rescue_robot -name "*.py" | sort

python3 -c "
import sys
failed = []
try:
    import cv2; print(f'  cv2 {cv2.__version__} OK')
except ImportError as e: failed.append(f'cv2: {e}')
try:
    import numpy; print(f'  numpy {numpy.__version__} OK')
except ImportError as e: failed.append(f'numpy: {e}')
try:
    from picamera2 import Picamera2; print('  picamera2 OK')
except ImportError as e: failed.append(f'picamera2: {e}')
try:
    from steelbar_powerful_bldc_driver import PowerfulBLDCDriver; print('  steelbar_powerful_bldc_driver OK')
except ImportError as e: failed.append(f'steelbar_powerful_bldc_driver: {e}')
if failed:
    print('FAILED IMPORTS:')
    for f in failed: print('  -', f)
    sys.exit(1)
print('All imports verified OK')
"

echo ""
echo "=========================================="
echo " Setup complete — rebooting in 5 seconds"
echo " (Ctrl-C to cancel reboot)"
echo "=========================================="
sleep 5
sudo reboot
