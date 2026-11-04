#!/usr/bin/env bash
set -uo pipefail

metadata=/etc/fsm-pi-001/install.conf
if [[ ! -r "${metadata}" ]]; then
    echo "FAIL: ${metadata} is missing; run install.sh first." >&2
    exit 1
fi
# This file is root-owned and written by install.sh.
source "${metadata}"
failures=0
warnings=0
pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*" >&2; failures=$((failures + 1)); }
warn() { echo "WARN: $*" >&2; warnings=$((warnings + 1)); }

required_units=(fsm-pi-can-interfaces fsm-pi-camera-gateway fsm-pi-linear-fan
                fsm-pi-telephoto-sender fsm-pi-vehicle-io)
for unit in "${required_units[@]}"; do
    if systemctl is-enabled --quiet "${unit}.service"; then
        pass "${unit} enabled"
    else
        fail "${unit} is not enabled"
    fi
    if systemctl is-active --quiet "${unit}.service"; then
        pass "${unit} active"
    else
        fail "${unit} is not active"
    fi
done

for iface in can10 can11; do
    if ip -details link show "${iface}" > /tmp/fsm-pi-verify-can 2>/dev/null; then
        if grep -q 'LISTEN-ONLY' /tmp/fsm-pi-verify-can; then
            pass "${iface} is kernel listen-only"
        else
            fail "${iface} is not listen-only; do not connect it to the vehicle"
        fi
    else
        fail "${iface} is absent"
    fi
done
rm -f /tmp/fsm-pi-verify-can

for map in front rear left right; do
    [[ -s "${INSTALL_DIR}/app/bev/calibration/fisheye_bev_20260903_physical_center/${map}.yaml" ]] \
      && pass "${map} calibration YAML present" || fail "${map} calibration missing"
    [[ -s "${INSTALL_DIR}/app/bev/calibration/fisheye_bev_20260903_physical_center/${map}_runtime_maps.xml.gz" ]] \
      && pass "${map} 512 remap present" || fail "${map} 512 remap missing"
done

while IFS='=' read -r key value; do
    case "${key}" in CAMERA_FRONT|CAMERA_REAR|CAMERA_LEFT|CAMERA_RIGHT)
        [[ -e "${value}" ]] && pass "${key} exists" || fail "${key} missing: ${value}" ;;
    esac
done < "${INSTALL_DIR}/config/gateway.env"

telephoto_device="$(sed -n 's/^CAMERA_DEVICE=//p' "${INSTALL_DIR}/config/telephoto.env")"
[[ -e "${telephoto_device}" ]] && pass "traffic-light camera exists" \
  || warn "traffic-light camera missing: ${telephoto_device}"

serial_device="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["serial"]["device"])' "${INSTALL_DIR}/state/vehicle-io.json")"
[[ -e "${serial_device}" ]] && pass "RS485 serial device exists" \
  || warn "RS485 device missing: ${serial_device}"

if curl --silent --fail --max-time 2 http://127.0.0.1:8089/api/status >/dev/null; then
    pass "vehicle I/O API responds on port 8089"
else
    fail "vehicle I/O API does not respond"
fi
if command -v vcgencmd >/dev/null 2>&1; then
    throttled="$(vcgencmd get_throttled 2>/dev/null || true)"
    [[ "${throttled}" == "throttled=0x0" ]] && pass "no current/historic undervoltage or throttling" \
      || warn "firmware reports ${throttled}"
fi

echo "Verification complete: failures=${failures} warnings=${warnings}"
[[ ${failures} -eq 0 ]]
