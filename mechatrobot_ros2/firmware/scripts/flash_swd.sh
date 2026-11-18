#!/bin/bash
# Write a built env to the UNO R4 WiFi through its on-board CMSIS-DAP (SWD),
# without going through the USB-serial bootloader path.
#
#   scripts/flash_swd.sh uno_r4_microros mechatrobot
#   scripts/flash_swd.sh uno_r4_serial   mechatrobot_serial
#
# Use this when `pio run -t upload` fails with "No device found on ttyACM0" or
# "Device unsupported". Only the application area (0x4000-) is erased and
# written; the Arduino bootloader at 0x0000-0x3FFF is left as it is, so normal
# uploads keep working afterwards. Slower than bossac (about 40 s for 120 KB).
set -eu

USAGE="usage: $0 <project dir under firmware/> <pio env name>"
PROJECT="${1:?${USAGE}}"
ENV_NAME="${2:?${USAGE}}"
PROJECT_DIR="$(cd "$(dirname "$0")/../${PROJECT}" && pwd)"
FIRMWARE="${PROJECT_DIR}/.pio/build/${ENV_NAME}/firmware.bin"
TARGET="r7fa4m1ab"
APP_BASE="0x4000"

if [ ! -f "${FIRMWARE}" ]; then
  echo "${FIRMWARE} がありません。先に pio run -e ${ENV_NAME} でビルドしてください" >&2
  exit 1
fi

PYOCD=(uv run --no-project --with pyocd pyocd)
# The Renesas device pack (flash algorithm for RA4M1) is installed once.
if ! "${PYOCD[@]}" list --targets 2>/dev/null | grep -q "^ *${TARGET} "; then
  "${PYOCD[@]}" pack install "${TARGET}"
fi
"${PYOCD[@]}" flash -t "${TARGET}" -e sector -a "${APP_BASE}" "${FIRMWARE}"
"${PYOCD[@]}" reset -t "${TARGET}"
# After the RA4M1 is reset over SWD, the USB-serial bridge (ESP32-S3) has stopped
# forwarding board->PC data every time we tried; a USB reset brings it back.
"$(dirname "$0")/recover_usb.sh"
