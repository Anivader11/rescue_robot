#!/bin/bash
# push_test_forwardv2.sh
# =======================
# Copies test_forwardv2.py from the local Windows project folder to the Pi.
# Run this from Git Bash / WSL on your laptop (plain PowerShell doesn't run
# bash scripts).
#
# Usage:
#   bash push_test_forwardv2.sh

set -e

LOCAL_PATH="/c/Users/RAM0038/OneDrive - Melbourne High School/Desktop/rescue_robot/test_forwardv2.py"
REMOTE_HOST="nigesh@rescue.local"
REMOTE_PATH="~/rescue_robot/test_forwardv2.py"

echo "Copying test_forwardv2.py to the Pi..."
scp "$LOCAL_PATH" "$REMOTE_HOST:$REMOTE_PATH"
echo "Done — pushed to $REMOTE_HOST:$REMOTE_PATH"
