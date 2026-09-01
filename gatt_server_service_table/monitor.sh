#!/bin/bash
# Capture 15 s of serial output to serial.log (run from a REAL WSL terminal)
# During the capture you can press the doorbell / volume buttons to test.
cd "$(dirname "$0")"
stty -F /dev/ttyUSB0 115200 raw -echo 2>/dev/null
timeout 15 cat /dev/ttyUSB0 | sed 's/\x1b\[[0-9;]*m//g' > serial.log
echo "=== serial.log saved: $(wc -c < serial.log) bytes ==="
