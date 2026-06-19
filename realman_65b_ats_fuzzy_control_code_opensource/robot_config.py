"""Runtime configuration for the robot connection.

The public repository must not contain real robot IP addresses. Put local
values in config.local.json or pass them with environment variables.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Any


CONFIG_FILE = Path(__file__).with_name("config.local.json")
IP_ENV = "CUSTOM_ROBOT_IP"
PORT_ENV = "CUSTOM_ROBOT_PORT"

_PLACEHOLDER_VALUES = {
    "",
    "YOUR_ROBOT_IP",
    "CUSTOM_ROBOT_IP",
    "ROBOT_IP",
    "xxx.xxx.xxx.xxx",
}


def _load_config() -> dict[str, Any]:
    if not CONFIG_FILE.exists():
        return {}

    with CONFIG_FILE.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("config.local.json must contain a JSON object.")
    return data


def get_robot_ip() -> str:
    """Return the configured robot IP, refusing placeholders."""
    config = _load_config()
    ip = os.environ.get(IP_ENV) or str(config.get("robot_ip", "")).strip()

    if ip in _PLACEHOLDER_VALUES or "x" in ip.lower():
        raise RuntimeError(
            "Robot IP is not configured. Copy config.example.json to "
            "config.local.json and set robot_ip, or set CUSTOM_ROBOT_IP."
        )

    if not re.fullmatch(r"\d{1,3}(?:\.\d{1,3}){3}", ip):
        raise ValueError(f"Invalid robot IP address: {ip!r}")

    parts = [int(part) for part in ip.split(".")]
    if any(part > 255 for part in parts):
        raise ValueError(f"Invalid robot IP address: {ip!r}")

    return ip


def get_robot_port(default: int = 8080) -> int:
    """Return the configured robot TCP port."""
    config = _load_config()
    raw_port = os.environ.get(PORT_ENV) or config.get("robot_port", default)

    try:
        port = int(raw_port)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid robot port: {raw_port!r}") from exc

    if not (1 <= port <= 65535):
        raise ValueError(f"Robot port out of range: {port}")

    return port


def get_robot_connection() -> tuple[str, int]:
    """Return (ip, port) for RobotArmController constructors."""
    return get_robot_ip(), get_robot_port()
