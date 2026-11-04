#!/usr/bin/env python3
"""Safely update FSM-Pi runtime settings without editing service files."""

from __future__ import annotations

import argparse
import ipaddress
import json
from pathlib import Path
import re
import subprocess


def read_install_dir(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit)
    metadata = Path("/etc/fsm-pi-001/install.conf")
    for line in metadata.read_text(encoding="utf-8").splitlines():
        if line.startswith("INSTALL_DIR="):
            return Path(line.split("=", 1)[1])
    raise RuntimeError("INSTALL_DIR is absent from install.conf")


def replace_env(path: Path, updates: dict[str, str]) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    seen: set[str] = set()
    output: list[str] = []
    for line in lines:
        key = line.split("=", 1)[0] if "=" in line and not line.startswith("#") else ""
        if key in updates:
            output.append(f"{key}={updates[key]}")
            seen.add(key)
        else:
            output.append(line)
    output.extend(f"{key}={value}" for key, value in updates.items() if key not in seen)
    path.write_text("\n".join(output) + "\n", encoding="utf-8")


def device(value: str) -> str:
    if not re.fullmatch(r"/dev/[A-Za-z0-9_./:+-]+", value):
        raise argparse.ArgumentTypeError("expected an absolute /dev path")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install-dir")
    parser.add_argument("--peer-ip", type=ipaddress.ip_address)
    parser.add_argument("--camera-front", type=device)
    parser.add_argument("--camera-rear", type=device)
    parser.add_argument("--camera-left", type=device)
    parser.add_argument("--camera-right", type=device)
    parser.add_argument("--telephoto-device", type=device)
    parser.add_argument("--serial-device", type=device)
    parser.add_argument("--audio-output")
    parser.add_argument("--volume", type=int)
    parser.add_argument("--max-volume", type=int)
    parser.add_argument("--mute", choices=("on", "off"))
    parser.add_argument("--no-restart", action="store_true")
    args = parser.parse_args()
    root = read_install_dir(args.install_dir)
    gateway_updates: dict[str, str] = {}
    telephoto_updates: dict[str, str] = {}
    if args.peer_ip:
        gateway_updates["PEER_IP"] = str(args.peer_ip)
        telephoto_updates["PEER_IP"] = str(args.peer_ip)
    for option, key in (("camera_front", "CAMERA_FRONT"),
                        ("camera_rear", "CAMERA_REAR"),
                        ("camera_left", "CAMERA_LEFT"),
                        ("camera_right", "CAMERA_RIGHT")):
        value = getattr(args, option)
        if value:
            gateway_updates[key] = value
    if args.telephoto_device:
        telephoto_updates["CAMERA_DEVICE"] = args.telephoto_device
    if gateway_updates:
        replace_env(root / "config/gateway.env", gateway_updates)
    if telephoto_updates:
        replace_env(root / "config/telephoto.env", telephoto_updates)

    vehicle_path = root / "state/vehicle-io.json"
    vehicle = json.loads(vehicle_path.read_text(encoding="utf-8"))
    changed_vehicle = False
    if args.serial_device:
        vehicle["serial"]["device"] = args.serial_device
        changed_vehicle = True
    if args.audio_output:
        if not re.fullmatch(r"[A-Za-z0-9_:,=.+-]+", args.audio_output):
            parser.error("invalid ALSA output")
        vehicle["audio"]["output"] = args.audio_output
        changed_vehicle = True
    for option, key in (("volume", "volume_percent"),
                        ("max_volume", "max_volume_percent")):
        value = getattr(args, option)
        if value is not None:
            if not 0 <= value <= 100:
                parser.error(f"--{option.replace('_', '-')} must be 0-100")
            vehicle["audio"][key] = value
            changed_vehicle = True
    if args.mute:
        vehicle["audio"]["muted"] = args.mute == "on"
        changed_vehicle = True
    if changed_vehicle:
        vehicle_path.write_text(json.dumps(vehicle, ensure_ascii=False, indent=2) + "\n",
                                encoding="utf-8")

    if not args.no_restart:
        units: list[str] = []
        if gateway_updates:
            units.append("fsm-pi-camera-gateway.service")
        if telephoto_updates:
            units.append("fsm-pi-telephoto-sender.service")
        if changed_vehicle:
            units.append("fsm-pi-vehicle-io.service")
        if units:
            subprocess.run(["systemctl", "restart", *units], check=True)
    print("Configuration updated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
