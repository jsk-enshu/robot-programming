#!/bin/bash
# Start micro_ros_agent on the UNO R4 WiFi's USB serial port.
#
#   scripts/run_agent.sh                 # /dev/ttyACM0
#   PORT=/dev/ttyACM1 scripts/run_agent.sh
#
# The board's USB-serial bridge occasionally stops forwarding board->PC data
# after the port has been closed and reopened, so the bridge is reset first
# (see recover_usb.sh). Set NO_USB_RESET=1 to skip that.
set -eu

PORT="${PORT:-/dev/ttyACM0}"
BAUD="${BAUD:-115200}"

if [ "${NO_USB_RESET:-0}" != "1" ]; then
  "$(dirname "$0")/../../scripts/recover_usb.sh"
fi
exec ros2 run micro_ros_agent micro_ros_agent serial --dev "${PORT}" -b "${BAUD}" "$@"
