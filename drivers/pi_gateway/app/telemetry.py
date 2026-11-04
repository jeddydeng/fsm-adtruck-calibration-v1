"""Compatibility helpers for W102 health JSON revisions."""

from __future__ import annotations

from typing import Any, Optional


def finite_number(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if number != number or number in (float("inf"), float("-inf")):
        return None
    return number


def cpu_metrics(health: dict[str, Any]) -> tuple[Optional[float], Optional[float]]:
    """Return (system CPU, gateway/process CPU), preserving v1 compatibility."""
    system = finite_number(health.get("system_cpu_percent"))
    process = finite_number(health.get("process_cpu_percent"))
    if process is None:
        process = finite_number(health.get("gateway_cpu_percent"))
    if process is None:
        process = finite_number(health.get("gateway_cpu_percent_total_capacity"))
    if system is None:
        # The original gateway only emitted cpu_total_percent. Its sampling was
        # process CPU divided by core count, but it remains the best available
        # compatibility value and is labelled as legacy in the dashboard.
        system = finite_number(health.get("cpu_total_percent"))
    return system, process


def memory_percent(health: dict[str, Any]) -> Optional[float]:
    value = finite_number(health.get("memory_used_percent"))
    return value if value is not None else finite_number(health.get("memory_percent"))


def image_rate_metrics(
        health: dict[str, Any], derived_generation_fps: Any
) -> tuple[Optional[float], Optional[float]]:
    """Return (Pi send FPS, real processing FPS) with old-gateway fallbacks."""
    send_fps = finite_number(health.get("output_send_fps"))
    if send_fps is None:
        send_fps = finite_number(health.get("bev_send_fps"))
    processing_fps = finite_number(health.get("image_processing_fps"))
    if processing_fps is None:
        processing_fps = finite_number(health.get("bev_fusion_fps"))
    if processing_fps is None:
        processing_fps = finite_number(derived_generation_fps)
    return send_fps, processing_fps
