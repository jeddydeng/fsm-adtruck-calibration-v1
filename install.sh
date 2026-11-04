#!/usr/bin/env bash
# Idempotent native installation. No vehicle services or hardware are started.
set -euo pipefail
export PYTHONNOUSERSITE=1
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:---full}"
case "${mode}" in
  --full|--python-only|--check) ;;
  *) echo "Usage: $0 [--full|--python-only|--check]" >&2; exit 2 ;;
esac
if [[ "${mode}" == --check ]]; then
  exec bash "${root}/scripts/verify.sh"
fi
if [[ "${mode}" == --full ]]; then
  [[ -r /etc/os-release ]] || { echo "Full installation requires Ubuntu." >&2; exit 2; }
  source /etc/os-release
  [[ "${ID}" == ubuntu ]] || { echo "Ubuntu required." >&2; exit 2; }
  case "${VERSION_ID}" in
    24.04) distro=jazzy ;;
    22.04) distro=humble ;;
    *) echo "Supported Ubuntu releases: 24.04 / 22.04." >&2; exit 2 ;;
  esac
  sudo -v
  sudo apt-get update
  sudo apt-get install -y curl ca-certificates software-properties-common
  sudo add-apt-repository -y universe
  if [[ ! -r "/opt/ros/${distro}/setup.bash" ]]; then
    task_tmp="$(mktemp -d)"
    # Official ROS apt-source package; HTTPS failures abort installation.
    ros_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.${VERSION_CODENAME}_all.deb" -o "${task_tmp}/ros2-apt-source.deb"
    sudo dpkg -i "${task_tmp}/ros2-apt-source.deb"
  fi
  sudo apt-get update
  mapfile -t packages < <(sed '/^[[:space:]]*#/d; /^[[:space:]]*$/d' "${root}/environment/apt.txt")
  sudo apt-get install -y "${packages[@]}" "ros-${distro}-ros-base" \
    "ros-${distro}-rmw-cyclonedds-cpp" "ros-${distro}-robot-state-publisher" \
    "ros-${distro}-xacro" "ros-${distro}-cv-bridge" "ros-${distro}-rosbag2-storage-mcap"
fi
python_cmd="${PYTHON_BIN:-python3}"
"${python_cmd}" -c 'import sys; assert (3,10) <= sys.version_info[:2] <= (3,12), "Use Python 3.10–3.12"'
if [[ ! -x "${root}/.venv/bin/python" ]]; then
  "${python_cmd}" -m venv --system-site-packages "${root}/.venv"
fi
"${root}/.venv/bin/python" -m pip install -r "${root}/requirement.txt"
if [[ "${mode}" == --full ]]; then
  export ROS_DISTRO="${distro}"
  source "${root}/scripts/env.sh"
  calibration_source_ros
  if [[ ! -r /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update --rosdistro "${distro}"
  rosdep install --from-paths "${root}/ros_ws/src" --ignore-src --rosdistro "${distro}" -r -y
  bash "${root}/scripts/build.sh"
  sudo usermod -aG video,dialout "$(id -un)"
  echo "Log out/in once for video/dialout group membership. Device/IP mappings: docs/OPERATIONS_EN.md."
fi
bash "${root}/scripts/verify.sh"
