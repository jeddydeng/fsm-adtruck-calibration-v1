#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
install_dir="/opt/fsm-pi-001"
run_user="${SUDO_USER:-fsm-pi-001}"
skip_packages=0
force_config=0
start_now=0

usage() {
    printf '%s\n' \
      "Usage: sudo ./scripts/install.sh [options]" \
      "  --user USER           service account (default: sudo caller or fsm-pi-001)" \
      "  --install-dir PATH    installation root (default: /opt/fsm-pi-001)" \
      "  --skip-packages       do not run apt-get" \
      "  --force-config        replace installed configuration with vehicle-186 defaults" \
      "  --start-now           start services now (normally reboot first)"
}

while (($#)); do
    case "$1" in
        --user) run_user="$2"; shift 2 ;;
        --install-dir) install_dir="$2"; shift 2 ;;
        --skip-packages) skip_packages=1; shift ;;
        --force-config) force_config=1; shift ;;
        --start-now) start_now=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ ${EUID} -ne 0 ]]; then
    echo "Run this installer through sudo." >&2
    exit 1
fi
if [[ ! "${install_dir}" =~ ^/[A-Za-z0-9._/-]+$ || "${install_dir}" == "/" ]]; then
    echo "Unsafe installation path: ${install_dir}" >&2
    exit 2
fi
if ! id "${run_user}" >/dev/null 2>&1; then
    echo "Service user does not exist: ${run_user}" >&2
    exit 2
fi
run_group="$(id -gn "${run_user}")"

if [[ ${skip_packages} -eq 0 ]]; then
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      build-essential cmake pkg-config libopencv-dev \
      can-utils iproute2 python3 ffmpeg alsa-utils v4l-utils \
      network-manager curl rsync
fi

build_dir="$(mktemp -d /tmp/fsm-pi-build.XXXXXX)"
cleanup() { rm -rf -- "${build_dir}"; }
trap cleanup EXIT
cmake -S "${repo_root}/app/bev" -B "${build_dir}/bev" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "${build_dir}/bev" --target w102_camera_gateway --parallel 2
g++ -O3 -DNDEBUG -std=c++17 -Wall -Wextra -Wpedantic \
  "${repo_root}/app/telephoto-camera/src/telephoto_udp_sender.cpp" \
  -o "${build_dir}/telephoto_udp_sender"

install -d -m 0755 "${install_dir}" "${install_dir}/app" \
  "${install_dir}/bin" "${install_dir}/config" "${install_dir}/state/audio-cache"
rsync -a --delete "${repo_root}/app/" "${install_dir}/app/"
install -m 0755 "${build_dir}/bev/w102_camera_gateway" \
  "${install_dir}/bin/w102_camera_gateway"
install -m 0755 "${build_dir}/telephoto_udp_sender" \
  "${install_dir}/bin/telephoto_udp_sender"
install -m 0755 "${install_dir}/app/vehicle-io/pi/vehicle_io_daemon.py" \
  "${install_dir}/app/vehicle-io/pi/vehicle_io_daemon.py"
install -m 0755 "${install_dir}/app/fan-control/linear_fan_control.py" \
  "${install_dir}/app/fan-control/linear_fan_control.py"

render_config() {
    local source_file="$1" destination="$2" mode="$3"
    if [[ ! -e "${destination}" || ${force_config} -eq 1 ]]; then
        sed "s|@INSTALL_DIR@|${install_dir}|g" "${source_file}" > "${destination}"
        chmod "${mode}" "${destination}"
    fi
}
render_config "${repo_root}/config/gateway.env.example" \
  "${install_dir}/config/gateway.env" 0644
render_config "${repo_root}/config/telephoto.env.example" \
  "${install_dir}/config/telephoto.env" 0644
if [[ ! -e "${install_dir}/state/vehicle-io.json" || ${force_config} -eq 1 ]]; then
    install -m 0644 "${repo_root}/config/vehicle-io.json.example" \
      "${install_dir}/state/vehicle-io.json"
fi
chown -R root:root "${install_dir}/app" "${install_dir}/bin"
chown -R "${run_user}:${run_group}" "${install_dir}/state"
chown "${run_user}:${run_group}" "${install_dir}/state/vehicle-io.json"

install -d -m 0755 /usr/local/libexec /etc/fsm-pi-001
install -m 0755 "${repo_root}/scripts/fsm-pi-can-setup" \
  /usr/local/libexec/fsm-pi-can-setup
install -m 0644 "${repo_root}/config/80-fsm-pi-can.rules" \
  /etc/udev/rules.d/80-fsm-pi-can.rules
install -m 0644 "${repo_root}/config/81-fsm-pi-rs485.rules" \
  /etc/udev/rules.d/81-fsm-pi-rs485.rules
install -m 0644 "${repo_root}/config/90-fsm-pi-watchdog.conf" \
  /etc/systemd/system.conf.d/90-fsm-pi-watchdog.conf

boot_config=/boot/firmware/config.txt
overlay_dir=/boot/firmware/overlays
if [[ ! -f "${boot_config}" || ! -d "${overlay_dir}" ]]; then
    echo "Raspberry Pi boot firmware directory was not found." >&2
    exit 1
fi
install -m 0644 "${repo_root}/drivers/waveshare-can-fd-hat-mode-b.dtbo" \
  "${overlay_dir}/waveshare-can-fd-hat-mode-b.dtbo"
if ! grep -q '^dtoverlay=waveshare-can-fd-hat-mode-b$' "${boot_config}"; then
    printf '\n# BEGIN FSM-PI-001 CAN FD HAT\ndtparam=spi=on\ndtoverlay=waveshare-can-fd-hat-mode-b\n# END FSM-PI-001 CAN FD HAT\n' \
      >> "${boot_config}"
fi

render_unit() {
    sed -e "s|@INSTALL_DIR@|${install_dir}|g" \
        -e "s|@RUN_USER@|${run_user}|g" \
        -e "s|@RUN_GROUP@|${run_group}|g" "$1" > "$2"
    chmod 0644 "$2"
}
for unit in fsm-pi-camera-gateway fsm-pi-can-interfaces \
            fsm-pi-linear-fan fsm-pi-telephoto-sender fsm-pi-vehicle-io; do
    render_unit "${repo_root}/systemd/${unit}.service.in" \
      "/etc/systemd/system/${unit}.service"
done
install -m 0644 "${repo_root}/systemd/fsm-pi.slice" \
  /etc/systemd/system/fsm-pi.slice
printf 'INSTALL_DIR=%s\nRUN_USER=%s\nRUN_GROUP=%s\n' \
  "${install_dir}" "${run_user}" "${run_group}" > /etc/fsm-pi-001/install.conf
chmod 0644 /etc/fsm-pi-001/install.conf

udevadm control --reload-rules
systemctl daemon-reload
systemctl enable fsm-pi-can-interfaces.service fsm-pi-camera-gateway.service \
  fsm-pi-linear-fan.service fsm-pi-telephoto-sender.service \
  fsm-pi-vehicle-io.service
if [[ ${start_now} -eq 1 ]]; then
    systemctl restart fsm-pi-can-interfaces.service
    systemctl restart fsm-pi-linear-fan.service fsm-pi-camera-gateway.service \
      fsm-pi-vehicle-io.service fsm-pi-telephoto-sender.service
else
    echo "Installation complete. Reboot before verification so the CAN overlay is loaded."
fi
