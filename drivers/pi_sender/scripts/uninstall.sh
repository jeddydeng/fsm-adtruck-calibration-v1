#!/usr/bin/env bash
set -euo pipefail

purge=0
[[ "${1:-}" == "--purge" ]] && purge=1
if [[ ${EUID} -ne 0 ]]; then
    echo "Run through sudo. Add --purge only if application/config removal is intended." >&2
    exit 1
fi
metadata=/etc/fsm-pi-001/install.conf
install_dir=""
if [[ -r "${metadata}" ]]; then
    source "${metadata}"
    install_dir="${INSTALL_DIR:-}"
fi
units=(fsm-pi-camera-gateway fsm-pi-telephoto-sender fsm-pi-vehicle-io
       fsm-pi-linear-fan fsm-pi-can-interfaces)
systemctl disable --now "${units[@]/%/.service}" 2>/dev/null || true
for unit in "${units[@]}"; do
    rm -f "/etc/systemd/system/${unit}.service"
done
rm -f /etc/systemd/system/fsm-pi.slice \
  /etc/systemd/system.conf.d/90-fsm-pi-watchdog.conf \
  /etc/udev/rules.d/80-fsm-pi-can.rules \
  /etc/udev/rules.d/81-fsm-pi-rs485.rules \
  /usr/local/libexec/fsm-pi-can-setup
systemctl daemon-reload
udevadm control --reload-rules
if [[ ${purge} -eq 1 ]]; then
    if [[ "${install_dir}" =~ ^/[A-Za-z0-9._/-]+$ && "${install_dir}" != "/" ]]; then
        rm -rf -- "${install_dir}"
    else
        echo "Refusing purge: invalid or unknown installation directory." >&2
        exit 2
    fi
    rm -rf /etc/fsm-pi-001
else
    echo "Programs and configuration preserved at ${install_dir:-unknown}."
fi
echo "The CAN overlay line and file were preserved to avoid changing boot hardware unexpectedly."
