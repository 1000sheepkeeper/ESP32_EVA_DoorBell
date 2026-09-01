#!/bin/bash
# Flash the doorbell-slave firmware (run this from a REAL WSL terminal, not the agent sandbox)
# Usage: cd ~/espprj/gatt_server_service_table/gatt_server_service_table && ./flash.sh
set -e
cd "$(dirname "$0")"
source /home/sk/.espressif/v6.0.2/esp-idf/export.sh >/dev/null 2>&1
IDF_COMPONENT_MANAGER=0 idf.py -p /dev/ttyUSB0 -b 115200 flash 2>&1 | tee flash.log
echo "=== FLASH EXIT: ${PIPESTATUS[0]} ==="
