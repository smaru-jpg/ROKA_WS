#!/usr/bin/env bash

# User-local build hints for the vendored Velodyne ROS 2 driver.
# Needed because this machine has libpcap runtime installed but not the
# system-wide libpcap development package.

roka_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PCAP_ROOT="$roka_ws_root/third_party/libpcap-dev/usr"
export PCAP_LIBRARY="/usr/lib/x86_64-linux-gnu/libpcap.so.0.8"
