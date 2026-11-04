#!/usr/bin/env bash
set -euo pipefail

echo "=== platform ==="
hostnamectl 2>/dev/null || true
uname -a
echo "=== cameras ==="
find -L /dev/v4l/by-id /dev/v4l/by-path -maxdepth 1 -type l -o -type c 2>/dev/null | sort || true
v4l2-ctl --list-devices 2>/dev/null || true
echo "=== serial ==="
find -L /dev/serial/by-id -maxdepth 1 -type l -o -type c 2>/dev/null | sort || true
echo "=== audio ==="
aplay -l 2>/dev/null || true
echo "=== CAN ==="
for iface in can10 can11; do
    ip -details -statistics link show "${iface}" 2>/dev/null || true
done
echo "=== thermal and power ==="
vcgencmd measure_temp 2>/dev/null || true
vcgencmd get_throttled 2>/dev/null || true
