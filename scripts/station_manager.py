#!/usr/bin/env python3
"""Own only this checkout's capture processes; preserve pre-existing services."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen
import webbrowser

ROOT = Path(__file__).resolve().parents[1]


def process_identity(pid: int) -> str | None:
    """Linux start ticks prevent a stale PID file from terminating another process."""
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return fields[19]
    except (OSError, IndexError):
        return None


def http_ready(url: str) -> bool:
    try:
        with urlopen(url, timeout=0.6) as response:
            return response.status == 200
    except (URLError, TimeoutError, OSError):
        return False


class StationManager:
    """Serialize starts/stops and track PID identity, ownership, readiness and logs."""

    def __init__(self, root: Path = ROOT) -> None:
        self.root = root
        self.runtime = root / ".runtime"
        self.runtime.mkdir(exist_ok=True)
        self.record = self.runtime / "processes.json"
        self.entries = json.loads(self.record.read_text()) if self.record.exists() else []
        self.python = os.environ.get("CALIBRATION_PYTHON", sys.executable)
        self.port = int(os.environ.get("CALIBRATION_PORT", "8765"))
        self.url = f"http://127.0.0.1:{self.port}"
        self.logs = root / "run_logs"
        self.logs.mkdir(exist_ok=True)

    @contextmanager
    def lock(self):
        with (self.runtime / "lock").open("w") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            # Reload after taking the lock: another invocation may have changed it.
            self.entries = json.loads(self.record.read_text()) if self.record.exists() else []
            yield

    def save(self) -> None:
        temporary = self.record.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.entries, indent=2))
        temporary.replace(self.record)

    def launch(self, name: str, args: list[str]) -> None:
        log = self.logs / f"{datetime.now():%Y%m%d_%H%M%S}_{name}.log"
        with log.open("ab", buffering=0) as output:
            child = subprocess.Popen(
                args,
                cwd=self.root,
                stdout=output,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        identity = process_identity(child.pid)
        if identity is None:
            raise RuntimeError(f"{name} exited immediately; inspect {log}")
        self.entries.append({"name": name, "pid": child.pid, "identity": identity, "log": str(log)})
        self.save()
        print(f"Started {name}: PID {child.pid}; log {log}", flush=True)

    def wait_ready(self, url: str, seconds: float = 20.0) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if http_ready(url):
                return
            if any(process_identity(e["pid"]) != e["identity"] for e in self.entries):
                raise RuntimeError("A managed process exited; inspect run_logs/.")
            time.sleep(0.25)
        raise TimeoutError(f"Service did not become ready: {url}")

    def start(self, traffic_only: bool = False) -> None:
        if any(process_identity(e["pid"]) == e["identity"] for e in self.entries):
            print(f"This checkout is already running. Stop it before changing mode: {self.url}")
            return
        self.entries = []
        if http_ready(self.url + "/api/status"):
            raise RuntimeError(f"Port {self.port} belongs to another station; choose another port.")
        gateway = self.root / "drivers/pi_gateway"
        pi_ip = os.environ.get("PI_GATEWAY_IP", "192.168.1.186")
        try:
            stream_url = os.environ.get("PI_STREAM_BASE_URL", "http://127.0.0.1:8086")
            if not traffic_only and os.environ.get("CALIBRATION_AUTO_GATEWAY", "1") == "1":
                if not http_ready(stream_url + "/api/stream/status"):
                    if stream_url.rstrip("/") != "http://127.0.0.1:8086":
                        raise RuntimeError(
                            "Custom Pi stream URL is unavailable; start its gateway."
                        )
                    self.launch(
                        "pi_gateway",
                        [
                            self.python,
                            str(gateway / "app/w102_monitor.py"),
                            "--gateway-ip",
                            pi_ip,
                            "--headless",
                            "--web",
                            "--data-port",
                            "5610",
                            "--control-port",
                            "5611",
                            "--web-bind",
                            "127.0.0.1",
                            "--web-port",
                            "8086",
                            "--capture-program",
                            str(gateway / "capture/capture_cameras.py"),
                            "--capture-output",
                            str(self.root / "output/pi_capture"),
                            "--voice-cache",
                            str(self.runtime / "voice-cache"),
                        ],
                    )
                    self.wait_ready(stream_url + "/api/stream/status")
            if os.environ.get("CALIBRATION_AUTO_TRAFFIC", "0") == "1":
                if not http_ready("http://127.0.0.1:8090/"):
                    self.launch(
                        "traffic_receiver",
                        [
                            self.python,
                            str(gateway / "traffic_camera/telephoto_ros2_receiver.py"),
                            "--source-ip",
                            pi_ip,
                        ],
                    )
                    self.wait_ready("http://127.0.0.1:8090/")
            command = [
                self.python,
                str(self.root / "process/joint_extrinsic_station/joint_capture_station.py"),
            ]
            if traffic_only:
                command.append("--traffic-only")
            self.launch("station", command)
            self.wait_ready(self.url + "/api/status")
        except (Exception, KeyboardInterrupt):
            self.stop()
            raise
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
            webbrowser.open(self.url)
        print(f"Capture station ready: {self.url}\nStop with: {self.root}/stop.sh")

    def stop(self) -> None:
        # Station closes SDK/restores Pi mode before its gateway is stopped.
        remaining = []
        for entry in reversed(self.entries):
            pid = entry["pid"]
            if process_identity(pid) != entry["identity"]:
                continue
            try:
                os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
            deadline = time.monotonic() + 20
            while process_identity(pid) == entry["identity"] and time.monotonic() < deadline:
                time.sleep(0.2)
            if process_identity(pid) == entry["identity"]:
                remaining.append(entry)
                print(
                    f"WARNING: {entry['name']} still exiting; retained PID record.", file=sys.stderr
                )
        self.entries = list(reversed(remaining))
        self.save()
        if remaining:
            raise RuntimeError("Some managed processes have not stopped; inspect logs and retry.")
        print("Capture processes stopped. Existing external services were preserved.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop"))
    parser.add_argument("--traffic-only", action="store_true")
    args = parser.parse_args()
    if not sys.platform.startswith("linux"):
        parser.error("Hardware capture requires Linux; offline solvers also support macOS.")
    manager = StationManager()
    with manager.lock():
        if args.action == "start":
            manager.start(args.traffic_only)
        else:
            manager.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
