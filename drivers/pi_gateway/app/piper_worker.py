#!/usr/bin/env python3
"""Persistent Piper worker using one loaded Mandarin ONNX model."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import wave

import onnxruntime
from piper import PiperVoice, SynthesisConfig
from piper.config import PiperConfig
from piper.phonemize_espeak import ESPEAK_DATA_DIR


def load_voice(model_path: str) -> PiperVoice:
    """Load Piper with bounded threads for the monitor service CPU quota."""
    config_path = Path(model_path + ".json")
    with config_path.open("r", encoding="utf-8") as stream:
        config = PiperConfig.from_dict(json.load(stream))
    session_options = onnxruntime.SessionOptions()
    session_options.intra_op_num_threads = 2
    session_options.inter_op_num_threads = 1
    session_options.execution_mode = onnxruntime.ExecutionMode.ORT_SEQUENTIAL
    session = onnxruntime.InferenceSession(
        model_path, sess_options=session_options,
        providers=["CPUExecutionProvider"])
    return PiperVoice(config=config, session=session,
                      espeak_data_dir=ESPEAK_DATA_DIR,
                      download_dir=Path.cwd())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    args = parser.parse_args()
    try:
        voice = load_voice(args.model)
    except Exception as error:
        print(json.dumps({"ready": False, "error": str(error)},
                         ensure_ascii=False), flush=True)
        return 2
    print(json.dumps({"ready": True}, ensure_ascii=False), flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            text = str(request["text"]).strip()
            output = Path(str(request["output"]))
            if not text or len(text) > 120:
                raise ValueError("播报文字必须为 1-120 个字符")
            config = SynthesisConfig(
                length_scale=float(request["length_scale"]),
                noise_scale=float(request["noise_scale"]),
                noise_w_scale=float(request["noise_w_scale"]),
            )
            with wave.open(str(output), "wb") as wav_file:
                voice.synthesize_wav(text, wav_file, syn_config=config)
            if not output.is_file() or output.stat().st_size < 1024:
                raise RuntimeError("中文语音模型未产生有效音频")
            print(json.dumps({"ok": True}), flush=True)
        except Exception as error:
            print(json.dumps({"ok": False, "error": str(error)},
                             ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
