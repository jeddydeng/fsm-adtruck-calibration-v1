"""Run the validated four-camera capture client for the local web dashboard."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import threading
from typing import Any


class CameraCaptureClient:
    """Serialize dashboard capture requests and validate the saved image set."""

    def __init__(self, program: str, output_dir: str, gateway_ip: str,
                 control_port: int, timeout_s: float = 2.0,
                 retries: int = 5) -> None:
        self.program = Path(program).resolve()
        self.output_dir = Path(output_dir).resolve()
        self.gateway_ip = gateway_ip
        self.control_port = control_port
        self.timeout_s = timeout_s
        self.retries = retries
        self._lock = threading.Lock()

    def capture(self) -> dict[str, Any]:
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("已有一组四路摄像头抓拍正在进行，请稍候")
        try:
            if not self.program.is_file():
                raise RuntimeError(f"抓拍程序不存在：{self.program}")
            self.output_dir.mkdir(parents=True, exist_ok=True)
            command = [
                sys.executable,
                str(self.program),
                "--gateway-ip", self.gateway_ip,
                "--control-port", str(self.control_port),
                "--timeout", str(self.timeout_s),
                "--retries", str(self.retries),
                "--output-dir", str(self.output_dir),
            ]
            try:
                completed = subprocess.run(
                    command,
                    cwd=str(self.program.parent),
                    capture_output=True,
                    text=True,
                    timeout=max(10.0, self.timeout_s * self.retries + 5.0),
                    check=False,
                )
            except subprocess.TimeoutExpired as error:
                raise RuntimeError("四路摄像头抓拍超时") from error
            if completed.returncode != 0:
                detail = (completed.stderr or completed.stdout).strip()
                raise RuntimeError(detail or "四路摄像头抓拍失败")

            paths = []
            for line in completed.stdout.splitlines():
                candidate = Path(line.strip())
                if candidate.suffix.lower() != ".jpg" or not candidate.is_absolute():
                    continue
                try:
                    candidate.relative_to(self.output_dir)
                except ValueError:
                    continue
                if candidate.is_file():
                    paths.append(candidate)
            if len(paths) != 4:
                raise RuntimeError("抓拍程序未返回完整的四张图片")
            names = [path.name for path in paths]
            roles = {name.rsplit("_", 1)[-1].removesuffix(".jpg").lower()
                     for name in names}
            if roles != {"front", "rear", "left", "right"}:
                raise RuntimeError("抓拍图片的方向命名不完整")
            return {
                "ok": True,
                "directory": str(self.output_dir),
                "files": names,
            }
        finally:
            self._lock.release()
