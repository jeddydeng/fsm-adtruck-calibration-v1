"""Offline Mandarin neural TTS backed by a persistent Piper worker."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
from typing import Any


ENGINE_CACHE_VERSION = "piper-1.4.2-huayan-medium-v1"
PROJECT_ROOT = Path(os.environ.get(
    "FSM_PI_MASTER_ROOT", Path(__file__).resolve().parents[1]))
DEFAULT_PYTHON = os.environ.get(
    "PIPER_PYTHON", str(PROJECT_ROOT / "voice-env/bin/python"))
DEFAULT_WORKER = os.environ.get(
    "PIPER_WORKER", str(PROJECT_ROOT / "app/piper_worker.py"))
DEFAULT_MODEL = os.environ.get(
    "PIPER_MODEL", str(PROJECT_ROOT / "voice/models/zh_CN-huayan-medium.onnx"))
DEFAULT_FFMPEG = "/usr/bin/ffmpeg"


VOICE_OPTIONS = [
    {"id": "voice_f1", "label": "01 清亮女声", "length_scale": 0.94, "noise_scale": 0.62, "noise_w": 0.72, "pitch": 2.0},
    {"id": "voice_f2", "label": "02 温柔女声", "length_scale": 1.08, "noise_scale": 0.55, "noise_w": 0.68, "pitch": 1.0},
    {"id": "voice_f3", "label": "03 标准女声（推荐）", "length_scale": 1.00, "noise_scale": 0.62, "noise_w": 0.72, "pitch": 0.0},
    {"id": "voice_f4", "label": "04 沉稳女声", "length_scale": 1.12, "noise_scale": 0.52, "noise_w": 0.65, "pitch": -1.0},
    {"id": "voice_f5", "label": "05 明快女声", "length_scale": 0.86, "noise_scale": 0.68, "noise_w": 0.76, "pitch": 2.5},
    {"id": "voice_m1", "label": "06 青年男声", "length_scale": 0.94, "noise_scale": 0.60, "noise_w": 0.70, "pitch": -2.0},
    {"id": "voice_m2", "label": "07 标准男声", "length_scale": 1.00, "noise_scale": 0.56, "noise_w": 0.68, "pitch": -3.0},
    {"id": "voice_m3", "label": "08 稳重男声", "length_scale": 1.10, "noise_scale": 0.50, "noise_w": 0.62, "pitch": -4.0},
    {"id": "voice_m4", "label": "09 浑厚男声", "length_scale": 1.14, "noise_scale": 0.48, "noise_w": 0.60, "pitch": -5.0},
    {"id": "voice_m5", "label": "10 明朗男声", "length_scale": 0.90, "noise_scale": 0.64, "noise_w": 0.74, "pitch": -1.5},
    {"id": "voice_m6", "label": "11 快速男声", "length_scale": 0.78, "noise_scale": 0.58, "noise_w": 0.68, "pitch": -2.5},
    {"id": "voice_m7", "label": "12 低沉男声", "length_scale": 1.16, "noise_scale": 0.46, "noise_w": 0.58, "pitch": -6.0},
    {"id": "voice_child_f", "label": "13 女童声", "length_scale": 0.88, "noise_scale": 0.70, "noise_w": 0.78, "pitch": 5.0},
    {"id": "voice_child_m", "label": "14 男童声", "length_scale": 0.90, "noise_scale": 0.66, "noise_w": 0.75, "pitch": 3.5},
    {"id": "voice_grandma", "label": "15 年长女声", "length_scale": 1.22, "noise_scale": 0.48, "noise_w": 0.60, "pitch": -1.0},
    {"id": "voice_grandpa", "label": "16 年长男声", "length_scale": 1.25, "noise_scale": 0.44, "noise_w": 0.56, "pitch": -5.0},
    {"id": "voice_announcer", "label": "17 广播员", "length_scale": 0.92, "noise_scale": 0.50, "noise_w": 0.58, "pitch": -2.0},
    {"id": "voice_whisper", "label": "18 轻声提示", "length_scale": 1.10, "noise_scale": 0.42, "noise_w": 0.55, "pitch": 1.0},
    {"id": "voice_robot", "label": "19 机器人", "length_scale": 0.96, "noise_scale": 0.25, "noise_w": 0.35, "pitch": -3.0},
    {"id": "voice_warning", "label": "20 警示播报", "length_scale": 0.82, "noise_scale": 0.55, "noise_w": 0.62, "pitch": -1.0},
]
_BY_ID = {item["id"]: item for item in VOICE_OPTIONS}


class OfflineTts:
    """Synthesize Mandarin with one long-lived Piper model process."""

    def __init__(self, python: str = DEFAULT_PYTHON,
                 worker: str = DEFAULT_WORKER,
                 model: str = DEFAULT_MODEL,
                 ffmpeg: str = DEFAULT_FFMPEG) -> None:
        self.python = Path(python)
        self.worker = Path(worker)
        self.model = Path(model)
        self.ffmpeg = Path(ffmpeg)
        for path, label in ((self.python, "Piper Python"),
                            (self.worker, "Piper 工作进程"),
                            (self.model, "中文语音模型"),
                            (Path(str(self.model) + ".json"), "中文模型配置"),
                            (self.ffmpeg, "FFmpeg")):
            if not path.is_file():
                raise RuntimeError(f"{label}不存在：{path}")
        self.lock = threading.Lock()
        self.process: subprocess.Popen[str] | None = None

    def _start_worker(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        self.process = subprocess.Popen(
            [str(self.python), str(self.worker), "--model", str(self.model)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1)
        assert self.process.stdout is not None
        ready_line = self.process.stdout.readline()
        try:
            ready = json.loads(ready_line)
        except json.JSONDecodeError as error:
            stderr = self.process.stderr.read() if self.process.stderr else ""
            raise RuntimeError(f"中文语音模型启动失败：{stderr.strip()}") from error
        if not ready.get("ready"):
            raise RuntimeError(str(ready.get("error", "中文语音模型启动失败")))

    def _raw_wav(self, text: str, role: dict[str, Any],
                 rate_percent: int, output: Path) -> None:
        self._start_worker()
        assert self.process is not None
        assert self.process.stdin is not None and self.process.stdout is not None
        speed_factor = max(0.5, min(2.0, (100 + rate_percent) / 100.0))
        request = {
            "text": text,
            "output": str(output),
            "length_scale": max(0.55, min(1.9,
                float(role["length_scale"]) / speed_factor)),
            "noise_scale": float(role["noise_scale"]),
            "noise_w_scale": float(role["noise_w"]),
        }
        self.process.stdin.write(json.dumps(request, ensure_ascii=False) + "\n")
        self.process.stdin.flush()
        response_line = self.process.stdout.readline()
        try:
            response = json.loads(response_line)
        except json.JSONDecodeError as error:
            stderr = self.process.stderr.read() if self.process.stderr else ""
            raise RuntimeError(f"中文语音生成进程异常退出：{stderr.strip()}") from error
        if not response.get("ok"):
            raise RuntimeError(str(response.get("error", "中文语音生成失败")))

    def _postprocess(self, source: Path, pitch_semitones: float) -> bytes:
        pitch = math.pow(2.0, pitch_semitones / 12.0)
        filters = []
        if abs(pitch_semitones) >= 0.05:
            filters.append(f"rubberband=pitch={pitch:.7f}:tempo=1.0")
        filters.append("loudnorm=I=-20:TP=-2:LRA=7")
        command = [
            str(self.ffmpeg), "-nostdin", "-hide_banner", "-loglevel", "error",
            "-i", str(source), "-filter:a", ",".join(filters),
            "-ac", "1", "-ar", "22050", "-c:a", "pcm_s16le", "-f", "wav", "pipe:1",
        ]
        result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, timeout=30, check=False)
        if result.returncode != 0 or len(result.stdout) < 1024:
            message = result.stderr.decode("utf-8", "replace").strip()
            raise RuntimeError(f"神经语音后处理失败：{message}")
        return result.stdout

    def synthesize(self, text: str, role_id: str,
                   rate_percent: int = 0) -> bytes:
        if role_id not in _BY_ID:
            raise ValueError("未知播音角色")
        text = str(text).strip()
        if not text or len(text) > 120:
            raise ValueError("播报文字必须为 1-120 个字符")
        rate_percent = max(-50, min(100, int(rate_percent)))
        role = _BY_ID[role_id]
        with self.lock, tempfile.TemporaryDirectory(prefix="w102-piper-") as directory:
            raw = Path(directory) / "raw.wav"
            self._raw_wav(text, role, rate_percent, raw)
            return self._postprocess(raw, float(role["pitch"]))


def cache_key(text: str, role_id: str, rate_percent: int,
              model_key: str = "") -> str:
    source = (f"{ENGINE_CACHE_VERSION}\0{model_key}\0{role_id}\0"
              f"{rate_percent}\0{text}")
    return hashlib.sha256(source.encode("utf-8")).hexdigest()


class CachedOfflineTts:
    def __init__(self, cache_dir: Path) -> None:
        self.cache_dir = cache_dir
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.engine = OfflineTts()

    def synthesize(self, text: str, role_id: str, rate_percent: int) -> bytes:
        path = self.cache_dir / f"{cache_key(text, role_id, rate_percent, self.engine.model.name)}.wav"
        if path.is_file():
            return path.read_bytes()
        payload = self.engine.synthesize(text, role_id, rate_percent)
        temporary = path.with_suffix(".tmp")
        temporary.write_bytes(payload)
        temporary.replace(path)
        return payload
