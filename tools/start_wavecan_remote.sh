#!/bin/sh
set -eu

cd /home/pi/WaveCan
pkill -f "python main.py" >/dev/null 2>&1 || true
rm -f /tmp/wavecan.log
setsid -f python main.py >/tmp/wavecan.log 2>&1
sleep 1
pgrep -af "python main.py" || true
