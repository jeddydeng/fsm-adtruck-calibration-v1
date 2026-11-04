"""IPC proxy for the Pi vehicle-I/O configuration and offline TTS."""

from __future__ import annotations

import base64
import json
import os
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from offline_tts import CachedOfflineTts, VOICE_OPTIONS


EVENTS = ("reverse", "left", "right", "automatic", "remote")
AUDIO_OUTPUTS = [
    {"id": "hw:CARD=Audio,DEV=0", "label": "USB 音频线（当前推荐）"},
    {"id": "hw:CARD=vc4hdmi0,DEV=0", "label": "HDMI 0 音频"},
    {"id": "hw:CARD=vc4hdmi1,DEV=0", "label": "HDMI 1 音频"},
]
AUDIO_SOURCES = [
    {"id": "original", "label": "原系统录音（树莓派本地，推荐）"},
    {"id": "neural", "label": "可编辑中文神经语音"},
]


PROJECT_ROOT = Path(os.environ.get(
    "FSM_PI_MASTER_ROOT", Path(__file__).resolve().parents[1]))


class VehicleIoClient:
    def __init__(self, base_url: str = "http://192.168.1.186:8089",
                 cache_dir: str = str(PROJECT_ROOT / "runtime/voice-cache"),
                 speaker_test_path: str = str(
                     PROJECT_ROOT / "voice/assets/speaker-test-zh-cn.mp3")) -> None:
        self.base_url = base_url.rstrip("/")
        self.tts = CachedOfflineTts(Path(cache_dir))
        self.speaker_test_path = Path(speaker_test_path)

    def _request(self, path: str, value: Any = None, timeout: float = 4.0) -> Any:
        data = None
        headers = {"Accept": "application/json"}
        if value is not None:
            data = json.dumps(value, ensure_ascii=False,
                              separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json; charset=utf-8"
        request = Request(self.base_url + path, data=data, headers=headers,
                          method="POST" if data is not None else "GET")
        try:
            with urlopen(request, timeout=timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            try:
                message = json.loads(error.read().decode("utf-8")).get("error")
            except Exception:
                message = str(error)
            raise RuntimeError(message) from error
        except (URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"树莓派语音服务不可达：{error}") from error

    def bundle(self) -> dict[str, Any]:
        try:
            return {"online": True, "config": self._request("/api/config", timeout=1.2),
                    "status": self._request("/api/status", timeout=1.2),
                    "voices": VOICE_OPTIONS, "outputs": AUDIO_OUTPUTS,
                    "sources": AUDIO_SOURCES}
        except RuntimeError as error:
            return {"online": False, "error": str(error), "voices": VOICE_OPTIONS,
                    "outputs": AUDIO_OUTPUTS, "sources": AUDIO_SOURCES}

    def _assets(self, config: dict[str, Any]) -> dict[str, str]:
        audio = config.get("audio")
        if not isinstance(audio, dict):
            raise ValueError("缺少音频配置")
        if audio.get("source", "original") == "original":
            return {}
        role = str(audio.get("voice", ""))
        rate = int(audio.get("rate_percent", 0))
        events = audio.get("events")
        if not isinstance(events, dict):
            raise ValueError("缺少播报项")
        result = {}
        for event in EVENTS:
            item = events.get(event)
            if not isinstance(item, dict):
                raise ValueError(f"缺少播报项 {event}")
            if bool(item.get("enabled", True)):
                wav = self.tts.synthesize(str(item.get("text", "")), role, rate)
                result[event] = base64.b64encode(wav).decode("ascii")
        return result

    def apply(self, config: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(config, dict):
            raise ValueError("配置格式无效")
        return self._request("/api/config", {"config": config,
                                             "assets": self._assets(config)}, timeout=20.0)

    def set_muted(self, muted: bool) -> dict[str, Any]:
        return self._request("/api/mute", {"muted": bool(muted)}, timeout=4.0)

    def preview(self, config: dict[str, Any], event: str) -> dict[str, Any]:
        if event not in EVENTS:
            raise ValueError("未知试听项目")
        if not isinstance(config, dict):
            raise ValueError("配置格式无效")
        audio = config.get("audio", {})
        controls = self._playback_controls(audio)
        if audio.get("source", "original") == "original":
            return self._request("/api/original-test", {
                **controls,
                "event": event,
            }, timeout=5.0)
        item = audio.get("events", {}).get(event, {})
        wav = self.tts.synthesize(str(item.get("text", "")),
                                  str(audio.get("voice", "")),
                                  int(audio.get("rate_percent", 0)))
        return self._request("/api/test", {
            "audio_base64": base64.b64encode(wav).decode("ascii"),
            **controls,
        }, timeout=5.0)

    @staticmethod
    def _playback_controls(audio: dict[str, Any]) -> dict[str, Any]:
        output = str(audio.get("output", "hw:CARD=Audio,DEV=0"))
        if output not in {item["id"] for item in AUDIO_OUTPUTS}:
            raise ValueError("未知音频输出源")
        if bool(audio.get("muted", False)):
            raise ValueError("当前已一键静音，请先解除静音")
        return {
            "output": output,
            "volume_percent": max(
                0, min(100, int(audio.get("volume_percent", 80)))),
            "max_volume_percent": max(
                0, min(100, int(audio.get("max_volume_percent", 80)))),
            "playback_rate": max(
                0.5, min(2.0, float(audio.get("playback_rate", 1.0)))),
            "muted": False,
        }

    def speaker_test(self, config: dict[str, Any]) -> dict[str, Any]:
        """Play the known-good Chinese MP3 under the configured safety ceiling."""
        if not isinstance(config, dict):
            raise ValueError("配置格式无效")
        audio = config.get("audio")
        if not isinstance(audio, dict):
            raise ValueError("缺少音频配置")
        controls = self._playback_controls(audio)
        try:
            payload = self.speaker_test_path.read_bytes()
        except OSError as error:
            raise RuntimeError("固定中文测试音不存在，请联系维护人员") from error
        if not payload.startswith((b"ID3", b"\xff\xf3", b"\xff\xfb")):
            raise RuntimeError("固定中文测试音文件格式错误")
        if not 1024 <= len(payload) <= 2_000_000:
            raise RuntimeError("固定中文测试音文件大小异常")
        return self._request("/api/test", {
            "audio_base64": base64.b64encode(payload).decode("ascii"),
            **controls,
        }, timeout=5.0)
