#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd)"
NETWORK_DEVICE="${1:-enx00909e9d235f}"
IMU_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
MD_LEFT_PORT="/dev/serial/by-id/usb-FTDI_USB_to_RS-422_485_Adapter_A76BHBER-if00-port0"
MD_RIGHT_PORT="/dev/serial/by-id/usb-FTDI_USB_to_RS-422_485_Adapter_A751CYHM-if00-port0"
ARM_RS485_PORT="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FBMQ-if00-port0"
ARM_TTL_PORT="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT78LQH1-if00-port0"
XBOX_STATUS=0
export PYTHONNOUSERSITE=1

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

printf '%s\n' '[devices]'
find /dev/serial/by-id -maxdepth 1 -type l -printf '%f -> %l\n' 2>/dev/null | sort || true
find /dev/input -maxdepth 1 -name 'js*' -printf '%p\n' 2>/dev/null | sort || true
for serial_port in "${IMU_PORT}" "${MD_LEFT_PORT}" "${MD_RIGHT_PORT}"; do
  if [[ -e "${serial_port}" ]]; then
    printf 'OK      %s -> %s\n' "${serial_port}" "$(readlink -f "${serial_port}")"
  else
    printf 'MISSING %s\n' "${serial_port}"
  fi
done

printf '%s\n' '[optional arm devices]'
for arm_port in "${ARM_RS485_PORT}" "${ARM_TTL_PORT}"; do
  if [[ -e "${arm_port}" ]]; then
    printf 'OK      %s -> %s\n' "${arm_port}" "$(readlink -f "${arm_port}")"
  else
    printf 'NOT CONNECTED %s\n' "${arm_port}"
  fi
done
if command -v rs-enumerate-devices >/dev/null 2>&1 && \
   rs-enumerate-devices -s 2>/dev/null | grep -q D435; then
  printf '%s\n' 'OK      Intel RealSense D435'
else
  printf '%s\n' 'NOT CONNECTED Intel RealSense D435'
fi

printf '%s\n' '[roka xbox]'
if ! python3 -m roka_md_driver.xbox_identity; then
  XBOX_STATUS=1
fi

printf '%s\n' '[velodyne network]'
ip -br address show dev "${NETWORK_DEVICE}" 2>/dev/null || printf 'missing: %s\n' "${NETWORK_DEVICE}"

printf '%s\n' '[nodes]'
ros2 node list 2>/dev/null | sort || true

printf '%s\n' '[required topics]'
for topic in \
  /velodyne_points /scan_stabilized /imu/data /wheel/odom /odom \
  /terrain/local_map /terrain/state /md/base_connected /cmd_vel_joy /cmd_vel; do
  topic_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ -n "${topic_type}" ]]; then
    printf 'OK      %-28s %s\n' "${topic}" "${topic_type}"
  else
    printf 'MISSING %s\n' "${topic}"
  fi
done

printf '%s\n' '[short rate checks]'
for topic in /velodyne_points /imu/data /odom /terrain/state; do
  if ros2 topic type "${topic}" >/dev/null 2>&1; then
    timeout 4 ros2 topic hz "${topic}" 2>/dev/null | tail -n 1 || true
  fi
done

printf '%s\n' '[tf]'
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | sed -n '1,8p' || true
timeout 3 ros2 run tf2_ros tf2_echo base_link velodyne 2>/dev/null | sed -n '1,8p' || true

exit "${XBOX_STATUS}"
