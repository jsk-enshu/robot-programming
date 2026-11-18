#!/bin/bash
# Bring back the UNO R4 WiFi's USB-serial bridge (ESP32-S3) without unplugging it.
#
# Symptom: the sketch is running (the LED blinks slowly while it waits for the
# agent) but micro_ros_agent never reports "session established", or
# `pio run -t upload` fails with "No device found" / "Device unsupported".
# A USB bus reset of the board is equivalent to replugging the cable for the
# bridge, and does not need sudo when /dev/bus/usb is world-writable.
set -eu

USB_ID="2341:1002"  # Arduino UNO R4 WiFi (CMSIS-DAP + serial bridge)

if ! command -v usbreset >/dev/null 2>&1; then
  echo "usbreset がありません: sudo apt install usbutils" >&2
  exit 1
fi
usbreset "${USB_ID}"
# The port drops and re-enumerates; give it a moment, then wait for it to come back.
sleep 1
for _ in $(seq 1 50); do
  if ls /dev/ttyACM* >/dev/null 2>&1; then
    exit 0
  fi
  sleep 0.1
done
echo "USB リセット後に /dev/ttyACM* が現れません" >&2
exit 1
