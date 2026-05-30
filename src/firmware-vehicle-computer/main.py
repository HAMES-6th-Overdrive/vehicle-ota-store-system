#!/usr/bin/env python3
"""
Threaded vehicle-computer runtime skeleton.

Run:
    python main.py
"""

import os
import ast
import base64
import binascii
import json
import logging
import re
import socket
import subprocess
import threading
import time
import queue
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Literal

from ethernet import build_someip_packet, parse_someip_packet, send_ethernet_message
from ota import OtaManager
from vehicle_control import FEATURE_IDS, GEAR_D, GEAR_P, VehicleControl


HOST = os.getenv("VEHICLE_HOST", "192.168.10.1")
PORT = int(os.getenv("VEHICLE_PORT", "8000"))
BASE_DIR = Path(__file__).resolve().parent
FRONTEND_DIR = BASE_DIR / "frontend"
FEATURE_STATE_FILE = BASE_DIR / "feature_state.json"
LEGACY_PURCHASES_FILE = BASE_DIR / "purchases.json"
DOWNLOADED_FEATURES_DIR = BASE_DIR / "features"
FIRMWARE_DIR = BASE_DIR / "firmware"

LOG_LEVEL = os.getenv("VEHICLE_LOG_LEVEL", "INFO").upper()
RESET_SETTINGS_CLEAR_DOWNLOADS = os.getenv("VEHICLE_RESET_CLEAR_DOWNLOADS", "1") != "0"

DEFAULT_VEHICLE_TX_HOST = "192.168.10.2"
DEFAULT_VEHICLE_TX_PORT = 30500
DEFAULT_DRIVE_SERVICE_ID = 0x0001
DEFAULT_DRIVE_METHOD_ID = 0x1001
DEFAULT_DRIVE_CLIENT_ID = 0x0001
DEFAULT_SENSOR_SERVICE_ID = 0x0002
DEFAULT_TOF_VALUE_UPDATED_EVENT_ID = 0x2002
DEFAULT_SPEED_UPDATED_EVENT_ID = 0x2003
DEFAULT_VEHICLE_EVENT_PORT = 30500
DEFAULT_AEB_SERVICE_ID = 0x0006
DEFAULT_AEB_STATUS_METHOD_ID = 0x1001
DEFAULT_AEB_CONTROL_METHOD_ID = 0x1002
DEFAULT_AEB_TRIGGER_EVENT_ID = 0x2001
DEFAULT_INFO_SERVICE_ID = 0x0007
DEFAULT_GET_SENSOR_ECU_VERSION_METHOD_ID = 0x1001
DEFAULT_GET_DRIVE_ECU_VERSION_METHOD_ID = 0x1002
DEFAULT_GET_FRONT_ECU_VERSION_METHOD_ID = 0x1003
DEFAULT_GET_AEB_VERSION_METHOD_ID = 0x1004
DEFAULT_OTA_RESET_SERVICE_ID = 0x0008
DEFAULT_OTA_FRONT_ZCU_TRIGGER_EVENT_ID = 0x2001
DEFAULT_OTA_DRIVE_ECU_TRIGGER_EVENT_ID = 0x2002
DEFAULT_OTA_SENSOR_ECU_TRIGGER_EVENT_ID = 0x2003

VEHICLE_TX_ENABLED = os.getenv("VEHICLE_TX_ENABLED", "1") != "0"
VEHICLE_TX_PROTOCOL = os.getenv("VEHICLE_TX_PROTOCOL", "udp").lower()
VEHICLE_TX_HOST = os.getenv("VEHICLE_TX_HOST", DEFAULT_VEHICLE_TX_HOST)
VEHICLE_TX_PORT = int(os.getenv("VEHICLE_TX_PORT", str(DEFAULT_VEHICLE_TX_PORT)))
VEHICLE_TX_TIMEOUT_SECONDS = float(os.getenv("VEHICLE_TX_TIMEOUT_SECONDS", "0.05"))
OTA_RESET_TRIGGER_ENABLED = os.getenv("VEHICLE_OTA_RESET_TRIGGER_ENABLED", "1") != "0"
OTA_RESET_TRIGGER_PROTOCOL = os.getenv("VEHICLE_OTA_RESET_TRIGGER_PROTOCOL", "udp").lower()
OTA_RESET_TRIGGER_HOST = os.getenv("VEHICLE_OTA_RESET_TRIGGER_HOST", VEHICLE_TX_HOST)
OTA_RESET_TRIGGER_PORT = int(os.getenv("VEHICLE_OTA_RESET_TRIGGER_PORT", str(VEHICLE_TX_PORT)))
OTA_RESET_TRIGGER_TIMEOUT_SECONDS = float(os.getenv("VEHICLE_OTA_RESET_TRIGGER_TIMEOUT_SECONDS", "0.1"))
VEHICLE_LINK_PING_ENABLED = os.getenv("VEHICLE_LINK_PING_ENABLED", "1") != "0"
VEHICLE_LINK_PING_HOST = os.getenv("VEHICLE_LINK_PING_HOST", VEHICLE_TX_HOST)
VEHICLE_LINK_PING_TIMEOUT_SECONDS = float(os.getenv("VEHICLE_LINK_PING_TIMEOUT_SECONDS", "0.75"))
VEHICLE_LINK_PING_INTERVAL_SECONDS = float(os.getenv("VEHICLE_LINK_PING_INTERVAL_SECONDS", "2"))
FRONT_ZCU_PING_HOST = os.getenv("FRONT_ZCU_PING_HOST", VEHICLE_TX_HOST)
VEHICLE_EVENT_HOST = os.getenv("VEHICLE_EVENT_HOST", "0.0.0.0")
VEHICLE_EVENT_PORT = int(os.getenv("VEHICLE_EVENT_PORT", str(DEFAULT_VEHICLE_EVENT_PORT)))
OTA_POLL_INTERVAL_SECONDS = float(os.getenv("VEHICLE_OTA_POLL_INTERVAL_SECONDS", "300"))
OTA_DOWNLOAD_TIMEOUT_SECONDS = float(os.getenv("VEHICLE_OTA_TIMEOUT_SECONDS", "30"))
INTERNET_CHECK_HOST = os.getenv("VEHICLE_INTERNET_CHECK_HOST", "8.8.8.8")
INTERNET_CHECK_PORT = int(os.getenv("VEHICLE_INTERNET_CHECK_PORT", "53"))
INTERNET_CHECK_TIMEOUT_SECONDS = float(os.getenv("VEHICLE_INTERNET_CHECK_TIMEOUT_SECONDS", "0.75"))
INTERNET_CHECK_INTERVAL_SECONDS = float(os.getenv("VEHICLE_INTERNET_CHECK_INTERVAL_SECONDS", "2"))
WEATHER_LATITUDE = float(os.getenv("VEHICLE_WEATHER_LATITUDE", "37.5665"))
WEATHER_LONGITUDE = float(os.getenv("VEHICLE_WEATHER_LONGITUDE", "126.9780"))
WEATHER_TIMEOUT_SECONDS = float(os.getenv("VEHICLE_WEATHER_TIMEOUT_SECONDS", "3"))
WEATHER_CACHE_SECONDS = float(os.getenv("VEHICLE_WEATHER_CACHE_SECONDS", "600"))
VEHICLE_COMM_STALE_SECONDS = float(os.getenv("VEHICLE_COMM_STALE_SECONDS", "7.5"))
FIRMWARE_VERSION_POLL_INTERVAL_SECONDS = float(
    os.getenv("VEHICLE_FIRMWARE_VERSION_POLL_INTERVAL_SECONDS", "5")
)
FIRMWARE_VERSION_QUERY_ENABLED = os.getenv("VEHICLE_FIRMWARE_VERSION_QUERY_ENABLED", "1") != "0"
FIRMWARE_VERSION_PROTOCOL = os.getenv("VEHICLE_FIRMWARE_VERSION_PROTOCOL", "udp").lower()
FIRMWARE_VERSION_PORT = int(os.getenv("VEHICLE_FIRMWARE_VERSION_PORT", str(DEFAULT_VEHICLE_TX_PORT)))
FIRMWARE_VERSION_TIMEOUT_SECONDS = float(os.getenv("VEHICLE_FIRMWARE_VERSION_TIMEOUT_SECONDS", "0.2"))
FIRMWARE_VERSION_SERVICE_ID = int(
    os.getenv("FIRMWARE_VERSION_SERVICE_ID", str(DEFAULT_INFO_SERVICE_ID)), 0
)
VEHICLE_COMPUTER_VERSION_TARGET = "VehicleComputer"
VEHICLE_COMPUTER_FIRMWARE_VERSION = os.getenv("VEHICLE_COMPUTER_FIRMWARE_VERSION", "1.0.0")
FEATURE_VERSION_TARGET = "Feature"

# SOME/IP IDs for Drive Service / Drive method.
DRIVE_SERVICE_ID = DEFAULT_DRIVE_SERVICE_ID
DRIVE_METHOD_ID = DEFAULT_DRIVE_METHOD_ID
DRIVE_CLIENT_ID = DEFAULT_DRIVE_CLIENT_ID
OTA_RESET_TRIGGER_CLIENT_ID = int(os.getenv("OTA_RESET_TRIGGER_CLIENT_ID", str(DRIVE_CLIENT_ID)), 0)
OTA_RESET_SERVICE_ID = int(os.getenv("OTA_RESET_SERVICE_ID", str(DEFAULT_OTA_RESET_SERVICE_ID)), 0)
OTA_FRONT_ZCU_TRIGGER_EVENT_ID = int(
    os.getenv("OTA_FRONT_ZCU_TRIGGER_EVENT_ID", str(DEFAULT_OTA_FRONT_ZCU_TRIGGER_EVENT_ID)), 0
)
OTA_DRIVE_ECU_TRIGGER_EVENT_ID = int(
    os.getenv("OTA_DRIVE_ECU_TRIGGER_EVENT_ID", str(DEFAULT_OTA_DRIVE_ECU_TRIGGER_EVENT_ID)), 0
)
OTA_SENSOR_ECU_TRIGGER_EVENT_ID = int(
    os.getenv("OTA_SENSOR_ECU_TRIGGER_EVENT_ID", str(DEFAULT_OTA_SENSOR_ECU_TRIGGER_EVENT_ID)), 0
)
SENSOR_SERVICE_ID = int(os.getenv("SENSOR_SERVICE_ID", str(DEFAULT_SENSOR_SERVICE_ID)), 0)
TOF_VALUE_UPDATED_EVENT_ID = int(
    os.getenv("TOF_VALUE_UPDATED_EVENT_ID", str(DEFAULT_TOF_VALUE_UPDATED_EVENT_ID)), 0
)
SPEED_UPDATED_EVENT_ID = int(
    os.getenv("SPEED_UPDATED_EVENT_ID", os.getenv("SPEED_EVENT_METHOD_ID", str(DEFAULT_SPEED_UPDATED_EVENT_ID))), 0
)
SPEED_EVENT_SERVICE_ID = SENSOR_SERVICE_ID
SPEED_EVENT_METHOD_ID = SPEED_UPDATED_EVENT_ID
AEB_SERVICE_ID = int(os.getenv("AEB_SERVICE_ID", str(DEFAULT_AEB_SERVICE_ID)), 0)
AEB_STATUS_METHOD_ID = int(os.getenv("AEB_STATUS_METHOD_ID", str(DEFAULT_AEB_STATUS_METHOD_ID)), 0)
AEB_CONTROL_METHOD_ID = int(
    os.getenv("AEB_CONTROL_METHOD_ID", os.getenv("AEB_ENABLE_METHOD_ID", str(DEFAULT_AEB_CONTROL_METHOD_ID))), 0
)
AEB_TRIGGER_EVENT_ID = int(
    os.getenv("AEB_TRIGGER_EVENT_ID", os.getenv("AEB_TRIGGER_METHOD_ID", str(DEFAULT_AEB_TRIGGER_EVENT_ID))), 0
)

MANUAL_FLASHER_PROGRESS_ID = "MANUAL_FLASHER"
FLASHER_BOARD_CONFIGS: dict[str, dict[str, Any]] = {
    "zcu": {
        "id": "zcu",
        "name": "ZCU",
        "transport": "doip",
        "implemented": True,
        "ecu_ip": os.getenv("FRONT_ZCU_OTA_IP", "192.168.10.2"),
        "doip_port": int(os.getenv("FRONT_ZCU_OTA_PORT", "13400")),
        "tester_address": int(os.getenv("FRONT_ZCU_TESTER_ADDRESS", "0x0E00"), 0),
        "ecu_address": int(os.getenv("FRONT_ZCU_ECU_ADDRESS", "0x0001"), 0),
        "bank_start": int(os.getenv("FRONT_ZCU_BANK_START", "0x80300000"), 0),
        "timeout_seconds": float(os.getenv("FRONT_ZCU_UDS_TIMEOUT_SECONDS", "60")),
        "p2_timeout_seconds": float(os.getenv("FRONT_ZCU_UDS_P2_TIMEOUT_SECONDS", os.getenv("FRONT_ZCU_UDS_TIMEOUT_SECONDS", "60"))),
        "p2_star_timeout_seconds": float(os.getenv("FRONT_ZCU_UDS_P2_STAR_TIMEOUT_SECONDS", os.getenv("FRONT_ZCU_UDS_TIMEOUT_SECONDS", "60"))),
        "use_server_timing": os.getenv("FRONT_ZCU_UDS_USE_SERVER_TIMING", "0") == "1",
    },
    "sensor-ecu": {
        "id": "sensor-ecu",
        "name": "Sensor ECU",
        "transport": "doip_sensor_can_ota",
        "implemented": True,
        "note": "Sensor CAN OTA via ZCU DoIP gateway",
        "ecu_ip": os.getenv("AEB_SENSOR_ECU_OTA_ZCU_IP", "192.168.10.2"),
        "doip_port": int(os.getenv("AEB_SENSOR_ECU_OTA_DOIP_PORT", "13401")),
        "tester_address": int(os.getenv("AEB_SENSOR_ECU_OTA_TESTER_ADDRESS", "0x0E00"), 0),
        "ecu_address": int(os.getenv("AEB_SENSOR_ECU_OTA_ZCU_ADDRESS", "0x0001"), 0),
        "app_addr": int(os.getenv("AEB_SENSOR_ECU_OTA_APP_ADDR", "0x80020000"), 0),
        "block_size": int(os.getenv("AEB_SENSOR_ECU_OTA_BLOCK_SIZE", "32")),
        "timeout_seconds": float(os.getenv("AEB_SENSOR_ECU_OTA_TIMEOUT_SECONDS", "60")),
        "block_delay_seconds": float(os.getenv("MANUAL_SENSOR_ECU_OTA_BLOCK_DELAY_SECONDS", "0")),
        "activate_after_transfer": False,
    },
}

BOARD_VERSION_CONFIGS: dict[str, dict[str, Any]] = {
    "ZCU": {
        "board_id": "front-zcu",
        "host": os.getenv("FRONT_ZCU_VERSION_HOST", "192.168.10.2"),
        "port": int(os.getenv("FRONT_ZCU_VERSION_PORT", str(FIRMWARE_VERSION_PORT))),
        "method_id": int(
            os.getenv("GET_FRONT_ECU_VERSION_METHOD_ID", str(DEFAULT_GET_FRONT_ECU_VERSION_METHOD_ID)), 0
        ),
        "method_name": "GetFrontEcuVersion",
    },
    "MotorECU": {
        "board_id": "drive-ecu",
        "host": os.getenv("FRONT_ZCU_VERSION_HOST", "192.168.10.2"),
        "port": int(os.getenv("FRONT_ZCU_VERSION_PORT", str(FIRMWARE_VERSION_PORT))),
        "method_id": int(
            os.getenv("GET_DRIVE_ECU_VERSION_METHOD_ID", str(DEFAULT_GET_DRIVE_ECU_VERSION_METHOD_ID)), 0
        ),
        "method_name": "GetDriveEcuVersion",
    },
    "SensorECU": {
        "board_id": "sensor-ecu",
        "host": os.getenv("FRONT_ZCU_VERSION_HOST", "192.168.10.2"),
        "port": int(os.getenv("FRONT_ZCU_VERSION_PORT", str(FIRMWARE_VERSION_PORT))),
        "method_id": int(
            os.getenv("GET_SENSOR_ECU_VERSION_METHOD_ID", str(DEFAULT_GET_SENSOR_ECU_VERSION_METHOD_ID)), 0
        ),
        "method_name": "GetSensorEcuVersion",
    },
    "AEB": {
        "board_id": "aeb",
        "host": os.getenv("FRONT_ZCU_VERSION_HOST", "192.168.10.2"),
        "port": int(os.getenv("FRONT_ZCU_VERSION_PORT", str(FIRMWARE_VERSION_PORT))),
        "method_id": int(
            os.getenv("GET_AEB_VERSION_METHOD_ID", str(DEFAULT_GET_AEB_VERSION_METHOD_ID)), 0
        ),
        "method_name": "GetAebVersion",
    },
}


logger = logging.getLogger("vehicle-computer")
ZCU_NETWORK_LOCK = threading.RLock()


STORE_CATALOG = [
    {
        "id": "AEB",
        "name": "AEB",
        "full_name": "자동 긴급 제동",
        "description": "전방 위험 상황을 감지하면 긴급 제동을 보조하는 기능입니다.",
        "kind": "feature",
        "latest_version": "1.0.0",
        "downloadable": True,
        "package_required": False,
        "download_file": "AEB.py",
        "runtime_class": "AEBFeature",
        "release_repo": "HAMES-6th-Overdrive/firmware-front-zcu",
        "ota_actions": [
            {
                "id": "sensor_ecu_firmware",
                "type": "doip_sensor_can_ota",
                "target": "sensor-ecu",
                "release_repo": os.getenv("AEB_SENSOR_ECU_OTA_REPO", "HAMES-6th-Overdrive/sensor-ecu"),
                "target_dir": "firmware",
                "asset_name": os.getenv("AEB_SENSOR_ECU_OTA_ASSET", "sensor_ecu__slow_toggle.bin"),
                "ecu_ip": os.getenv("AEB_SENSOR_ECU_OTA_ZCU_IP", "192.168.10.2"),
                "doip_port": int(os.getenv("AEB_SENSOR_ECU_OTA_DOIP_PORT", "13401")),
                "tester_address": int(os.getenv("AEB_SENSOR_ECU_OTA_TESTER_ADDRESS", "0x0E00"), 0),
                "ecu_address": int(os.getenv("AEB_SENSOR_ECU_OTA_ZCU_ADDRESS", "0x0001"), 0),
                "app_addr": int(os.getenv("AEB_SENSOR_ECU_OTA_APP_ADDR", "0x80020000"), 0),
                "block_size": int(os.getenv("AEB_SENSOR_ECU_OTA_BLOCK_SIZE", "32")),
                "timeout_seconds": float(os.getenv("AEB_SENSOR_ECU_OTA_TIMEOUT_SECONDS", "60")),
                "block_delay_seconds": float(os.getenv("AEB_SENSOR_ECU_OTA_BLOCK_DELAY_SECONDS", "0.005")),
                "activate_after_transfer": False,
                "release_patch_filter": int(os.getenv("AEB_SENSOR_ECU_OTA_RELEASE_PATCH_FILTER", "1")),
            },
            {
                "id": "zcu_firmware",
                "type": "doip_uds_flash",
                "target": "zcu",
                "release_repo": "HAMES-6th-Overdrive/firmware-front-zcu",
                "target_dir": "firmware",
                "ecu_ip": "192.168.10.2",
                "doip_port": 13400,
                "tester_address": 3584,
                "ecu_address": 1,
                "bank_start": 0x80300000,
                "timeout_seconds": 60,
                "p2_timeout_seconds": 60,
                "p2_star_timeout_seconds": 60,
                "use_server_timing": False,
                "release_patch_filter": 1,
            },
        ],
    },
    {
        "id": "FVSA",
        "name": "FVSA",
        "full_name": "앞차 출발 알림",
        "description": "정차 중 앞차가 출발하면 운전자에게 알려주는 기능입니다.",
        "kind": "feature",
        "latest_version": "1.0.0",
        "downloadable": True,
        "download_file": "FVSA.py",
        "runtime_class": "FVSAFeature",
        "release_repo": "HAMES-6th-Overdrive/FVSA",
        "ota_actions": [
            {
                "id": "python_package",
                "type": "github_release_file",
                "target": "rpi",
                "release_repo": "HAMES-6th-Overdrive/FVSA",
                "download_file": "FVSA.py",
                "target_dir": "features",
            }
        ],
    },
    {
        "id": "LKAS",
        "name": "LKAS",
        "full_name": "차선 유지 보조",
        "description": "차선 중앙 주행을 돕는 보조 기능입니다.",
        "kind": "feature",
        "latest_version": "1.3.0",
        "downloadable": True,
        "download_file": "LKAS.py",
        "runtime_class": "LKASFeature",
        "release_repo": "HAMES-6th-Overdrive/LKAS",
        "ota_actions": [
            {
                "id": "python_package",
                "type": "github_release_file",
                "target": "rpi",
                "release_repo": "HAMES-6th-Overdrive/LKAS",
                "download_file": "LKAS.py",
                "target_dir": "features",
            }
        ],
    },
    {
        "id": "LUFFY_THEME",
        "name": "테마(루피)",
        "full_name": "루피 테마",
        "description": "대시보드에 적용할 수 있는 루피 스타일 테마입니다.",
        "kind": "theme",
        "latest_version": "1.0.0",
        "downloadable": False,
    },
]

STORE_ITEM_IDS = {item["id"] for item in STORE_CATALOG}


HeartbeatCallback = Callable[[], None]
ChildTarget = Callable[[threading.Event, HeartbeatCallback], None]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def seconds_since(timestamp: str | None) -> float | None:
    if not timestamp:
        return None
    try:
        parsed = datetime.fromisoformat(timestamp)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return max(0.0, (datetime.now(timezone.utc) - parsed.astimezone(timezone.utc)).total_seconds())


def safe_filename(name: str) -> str:
    clean = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name.strip())
    clean = clean.strip("._")
    return clean or "firmware.bin"


def firmware_version(version: str | None) -> str | None:
    if not version:
        return None
    value = str(version).strip()
    if value.lower().startswith("v"):
        value = value[1:]
    match = re.search(r"(?<!\d)(\d+)[._-](\d+)[._-](\d+)(?!\d)", value)
    if match:
        major, minor, patch = (int(part) for part in match.groups())
        return f"{major}.{minor}.{patch}"

    match = re.search(r"(?<!\d)(\d+)[._-](\d+)(?!\d)", value)
    if not match:
        return None
    return f"{int(match.group(1))}.{int(match.group(2))}.0"


def firmware_version_tuple(version: str | None) -> tuple[int, int, int]:
    normalized = firmware_version(version)
    if not normalized:
        return (0, 0, 0)
    major, minor, patch = normalized.split(".", 2)
    return int(major), int(minor), int(patch)


def firmware_version_gt(candidate: str | None, current: str | None) -> bool:
    return firmware_version_tuple(candidate) > firmware_version_tuple(current)


def ota_target_display_name(target: Any) -> str:
    return {
        "zcu": "ZCU",
        "front-zcu": "Front ZCU",
        "sensor-ecu": "Sensor ECU",
        "drive-ecu": "Drive ECU",
    }.get(str(target or "zcu"), str(target or "zcu"))


def firmware_target_versions(record: dict[str, Any]) -> dict[str, str]:
    zcu_ota = record.get("zcu_ota", {})
    raw_versions = zcu_ota.get("versions", {}) if isinstance(zcu_ota, dict) else {}
    versions: dict[str, str] = {}
    if isinstance(raw_versions, dict):
        for target, version in raw_versions.items():
            normalized = firmware_version(str(version)) or str(version or "")
            if normalized:
                versions[str(target)] = normalized

    if not versions and isinstance(zcu_ota, dict) and zcu_ota.get("version"):
        target = ota_target_display_name(zcu_ota.get("target", "zcu"))
        version = firmware_version(str(zcu_ota["version"])) or str(zcu_ota["version"])
        versions[target] = version

    return versions


def python_module_version(path: Path | None) -> str | None:
    if path is None or not path.exists() or path.suffix != ".py":
        return None
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, SyntaxError, UnicodeDecodeError):
        return None

    constants: dict[str, str] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        try:
            value = ast.literal_eval(node.value)
        except (ValueError, SyntaxError):
            continue
        if not isinstance(value, str):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ("VERSION", "__version__"):
                constants[target.id] = value

    return constants.get("VERSION") or constants.get("__version__")


def configure_logging() -> None:
    logging.basicConfig(
        level=getattr(logging, LOG_LEVEL, logging.INFO),
        format="%(asctime)s %(levelname)-8s [%(threadName)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def ensure_api_port_available(host: str, port: int) -> None:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind((host, port))
    except OSError as exc:
        raise RuntimeError(
            f"API port is already in use: {host}:{port}. "
            f"Stop the existing server or set VEHICLE_PORT to another port."
        ) from exc


@dataclass
class ChildService:
    name: str
    target: ChildTarget
    thread: threading.Thread | None = None
    started_at: str | None = None
    last_heartbeat: str | None = None
    error: str | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def mark_started(self) -> None:
        with self._lock:
            now = utc_now()
            self.started_at = now
            self.last_heartbeat = now
            self.error = None

    def mark_heartbeat(self) -> None:
        with self._lock:
            self.last_heartbeat = utc_now()

    def mark_error(self, exc: BaseException) -> None:
        with self._lock:
            self.error = f"{type(exc).__name__}: {exc}"

    def snapshot(self) -> dict:
        with self._lock:
            running = self.thread.is_alive() if self.thread else False
            return {
                "name": self.name,
                "running": running,
                "started_at": self.started_at,
                "last_heartbeat": self.last_heartbeat,
                "error": self.error,
            }


class Supervisor:
    def __init__(self) -> None:
        self.started_at = utc_now()
        self.stop_event = threading.Event()
        self.children: dict[str, ChildService] = {}
        self._lock = threading.Lock()

    def register(self, child: ChildService) -> None:
        with self._lock:
            if child.name in self.children:
                raise ValueError(f"duplicate child service: {child.name}")
            self.children[child.name] = child

    def start_all(self) -> None:
        with self._lock:
            children = list(self.children.values())

        for child in children:
            if child.thread and child.thread.is_alive():
                continue

            child.mark_started()
            child.thread = threading.Thread(
                name=child.name,
                target=self._run_child,
                args=(child,),
                daemon=False,
            )
            child.thread.start()
            logger.debug("started child service: %s", child.name)

    def _run_child(self, child: ChildService) -> None:
        try:
            child.target(self.stop_event, child.mark_heartbeat)
        except BaseException as exc:
            child.mark_error(exc)
            logger.exception("child service stopped with error: %s", child.name)
        finally:
            child.mark_heartbeat()
            logger.debug("child service exited: %s", child.name)

    def request_stop(self) -> None:
        if not self.stop_event.is_set():
            logger.debug("stop requested")
        self.stop_event.set()

    def join_all(self, timeout: float = 5.0) -> None:
        with self._lock:
            children = list(self.children.values())

        for child in children:
            if child.thread and child.thread.is_alive():
                child.thread.join(timeout=timeout)
                if child.thread.is_alive():
                    logger.error("child service did not exit within %.1fs: %s", timeout, child.name)

    def children_snapshot(self) -> list[dict]:
        with self._lock:
            children = list(self.children.values())
        return [child.snapshot() for child in children]

    def health_snapshot(self) -> dict:
        children = self.children_snapshot()
        now = time.time()
        started = datetime.fromisoformat(self.started_at).timestamp()
        return {
            "status": "stopping" if self.stop_event.is_set() else "running",
            "started_at": self.started_at,
            "uptime_seconds": round(now - started, 3),
            "children": children,
        }


class FeatureStateStore:
    def __init__(
        self,
        path: Path,
        *,
        ota_manager: OtaManager,
        legacy_path: Path | None = None,
    ) -> None:
        self.path = path
        self.legacy_path = legacy_path
        self.ota_manager = ota_manager
        self._lock = threading.Lock()

    def _empty(self) -> dict:
        return {
            "schema_version": 2,
            "purchased": [],
            "purchase_history": [],
            "pending_ota": {"to_install": [], "to_update": [], "checked_at": None},
            "items": {item["id"]: self._default_record(item["id"]) for item in STORE_CATALOG},
        }

    def _catalog_item(self, feature_id: str) -> dict:
        for item in STORE_CATALOG:
            if item["id"] == feature_id:
                return item
        raise ValueError(f"unknown store item: {feature_id}")

    def _default_record(self, feature_id: str) -> dict:
        item = self._catalog_item(feature_id)
        package_required = bool(item.get("package_required", True))
        download_file = item.get("download_file") if package_required else None
        package_action = None
        if item.get("downloadable") and package_required:
            try:
                package_action = self.ota_manager.python_package_action(item)
            except ValueError:
                package_action = None
        zcu_action = self.ota_manager.zcu_flash_action(item) if any(
            action.get("type") == "doip_uds_flash"
            for action in self.ota_manager.actions_for(item)
        ) else None
        return {
            "purchased": False,
            "enabled": False,
            "version": item.get("latest_version", "1.0.0"),
            "package": {
                "downloadable": bool(item.get("downloadable", False)),
                "downloaded": False,
                "applied": False,
                "path": str(Path("features") / download_file) if download_file else None,
                "source": (
                    "github-release" if package_action and package_action.get("type") == "github_release_file"
                    else "local-file" if package_action and package_action.get("type") == "local_file"
                    else "firmware-only" if item.get("downloadable") and not package_required
                    else None
                ),
                "action_id": package_action.get("id") if package_action else None,
                "action_type": package_action.get("type") if package_action else None,
                "target": package_action.get("target") if package_action else None,
                "repo": package_action.get("release_repo") if package_action else None,
                "release_tag": None,
                "release_url": None,
                "asset_name": None,
                "versions": {},
                "downloaded_at": None,
                "applied_at": None,
                "checked_at": None,
                "error": None,
            },
            "zcu_ota": {
                "required": zcu_action is not None,
                "downloaded": False,
                "applied": False,
                "path": None,
                "source": "github-release" if zcu_action else None,
                "action_id": zcu_action.get("id") if zcu_action else None,
                "action_type": zcu_action.get("type") if zcu_action else None,
                "target": zcu_action.get("target") if zcu_action else None,
                "repo": zcu_action.get("release_repo") if zcu_action else None,
                "version": None,
                "release_tag": None,
                "release_url": None,
                "asset_name": None,
                "downloaded_at": None,
                "applied_at": None,
                "checked_at": None,
                "error": None,
            },
        }

    def load(self) -> dict:
        with self._lock:
            return self._load_unlocked()

    def _load_unlocked(self) -> dict:
        source_path = self.path
        if not source_path.exists() and self.legacy_path and self.legacy_path.exists():
            source_path = self.legacy_path

        if not source_path.exists():
            data = self._empty()
            try:
                self._save_unlocked(data)
            except OSError as exc:
                logger.warning("feature state could not be recreated: %s", exc)
            return data

        try:
            with source_path.open("r", encoding="utf-8") as file:
                data = json.load(file)
        except (OSError, json.JSONDecodeError):
            logger.warning("feature state is unreadable, using empty state: %s", source_path)
            data = self._empty()
            try:
                self._save_unlocked(data)
            except OSError as exc:
                logger.warning("feature state could not be repaired: %s", exc)
            return data

        return self._normalize_unlocked(data)

    def _normalize_unlocked(self, data: dict) -> dict:
        purchased = data.get("purchased", [])
        purchase_history = data.get("purchase_history", data.get("purchases", []))
        if not isinstance(purchased, list):
            purchased = []
        if not isinstance(purchase_history, list):
            purchase_history = []

        items = data.get("items", {})
        if not isinstance(items, dict):
            items = {}

        normalized = {
            "schema_version": 2,
            "purchased": [item for item in purchased if isinstance(item, str)],
            "purchase_history": [item for item in purchase_history if isinstance(item, dict)],
            "pending_ota": {"to_install": [], "to_update": [], "checked_at": None},
            "items": {},
        }
        raw_pending = data.get("pending_ota", {})
        if isinstance(raw_pending, dict):
            normalized["pending_ota"] = {
                "to_install": [
                    item for item in raw_pending.get("to_install", []) if isinstance(item, dict)
                ],
                "to_update": [
                    item for item in raw_pending.get("to_update", []) if isinstance(item, dict)
                ],
                "checked_at": raw_pending.get("checked_at"),
            }

        for catalog_item in STORE_CATALOG:
            feature_id = catalog_item["id"]
            record = self._default_record(feature_id)
            raw_record = items.get(feature_id, {})
            if not isinstance(raw_record, dict):
                raw_record = {}

            record["purchased"] = bool(raw_record.get("purchased", feature_id in normalized["purchased"]))
            record["enabled"] = bool(raw_record.get("enabled", False))
            record["version"] = str(raw_record.get("version", catalog_item.get("latest_version", "1.0.0")))

            raw_package = raw_record.get("package", {})
            if isinstance(raw_package, dict):
                record["package"].update(
                    {
                        "downloaded": bool(raw_package.get("downloaded", False)),
                        "applied": bool(raw_package.get("applied", False)),
                        "action_id": raw_package.get("action_id", record["package"]["action_id"]),
                        "action_type": raw_package.get("action_type", record["package"]["action_type"]),
                        "target": raw_package.get("target", record["package"]["target"]),
                        "release_tag": raw_package.get("release_tag"),
                        "release_url": raw_package.get("release_url"),
                        "asset_name": raw_package.get("asset_name"),
                        "downloaded_at": raw_package.get("downloaded_at"),
                        "applied_at": raw_package.get("applied_at"),
                        "checked_at": raw_package.get("checked_at"),
                        "error": raw_package.get("error"),
                    }
                )

            raw_ota = raw_record.get("zcu_ota", {})
            if isinstance(raw_ota, dict):
                record["zcu_ota"].update(
                    {
                        "required": bool(raw_ota.get("required", False)),
                        "downloaded": bool(raw_ota.get("downloaded", False)),
                        "applied": bool(raw_ota.get("applied", False)),
                        "path": raw_ota.get("path"),
                        "action_id": raw_ota.get("action_id", record["zcu_ota"]["action_id"]),
                        "action_type": raw_ota.get("action_type", record["zcu_ota"]["action_type"]),
                        "target": raw_ota.get("target", record["zcu_ota"]["target"]),
                        "repo": raw_ota.get("repo", record["zcu_ota"]["repo"]),
                        "version": raw_ota.get("version"),
                        "release_tag": raw_ota.get("release_tag"),
                        "release_url": raw_ota.get("release_url"),
                        "asset_name": raw_ota.get("asset_name"),
                        "versions": raw_ota.get("versions") if isinstance(raw_ota.get("versions"), dict) else {},
                        "downloaded_at": raw_ota.get("downloaded_at"),
                        "applied_at": raw_ota.get("applied_at"),
                        "checked_at": raw_ota.get("checked_at"),
                        "error": raw_ota.get("error"),
                    }
                )

            if catalog_item.get("downloadable") and catalog_item.get("package_required", True):
                download_path = self.ota_manager.downloaded_features_dir / str(catalog_item["download_file"])
                record["package"]["downloaded"] = download_path.exists()
                record["package"]["path"] = (
                    str(download_path.relative_to(self.ota_manager.base_dir))
                    if download_path.is_relative_to(self.ota_manager.base_dir)
                    else str(download_path)
                )
                if not record["package"]["downloaded"]:
                    record["package"]["applied"] = False
                module_version = python_module_version(download_path)
                if module_version:
                    record["version"] = module_version

            normalized["items"][feature_id] = record

        normalized["purchased"] = [
            feature_id
            for feature_id, record in normalized["items"].items()
            if record["purchased"]
        ]
        return normalized

    def _save_unlocked(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        with temp_path.open("w", encoding="utf-8") as file:
            json.dump(data, file, ensure_ascii=False, indent=2)
            file.write("\n")
        temp_path.replace(self.path)

    def reset(self, *, clear_downloads: bool = True) -> dict:
        with self._lock:
            removed_downloads = (
                self.ota_manager.clear_downloaded_feature_packages()
                + self.ota_manager.clear_downloaded_firmware_packages()
                if clear_downloads
                else []
            )
            self.ota_manager.clear_progress()
            legacy_removed = False
            if self.legacy_path and self.legacy_path.exists():
                self.legacy_path.unlink()
                legacy_removed = True
            data = self._empty()
            self._save_unlocked(data)
            return {
                "success": True,
                "state": data,
                "removed_downloads": removed_downloads,
                "legacy_removed": legacy_removed,
            }

    def _zcu_actions(self, item: dict) -> list[dict]:
        return [
            action
            for action in self.ota_manager.actions_for(item)
            if action.get("type") in ("doip_uds_flash", "doip_sensor_can_ota")
        ]

    def _is_record_installed(self, item: dict, record: dict) -> bool:
        if item.get("kind") == "theme":
            return bool(record["purchased"])
        if not item.get("downloadable"):
            return bool(record["purchased"])
        package_required = bool(item.get("package_required", True))
        if package_required and not (record["package"].get("downloaded") and record["package"].get("applied")):
            return False
        if self._zcu_actions(item):
            return bool(record["zcu_ota"].get("applied"))
        return True

    def _pending_item(
        self,
        item: dict,
        action: dict,
        record: dict,
        *,
        ota_kind: Literal["initial_install", "version_update"],
        update_scope: Literal["install", "feature", "firmware"],
        current_version: str | None,
        latest_version: str | None,
        latest_versions: dict[str, str | None],
    ) -> dict:
        feature_id = item["id"]
        target = action.get("target", "zcu")
        target_name = ota_target_display_name(target)
        if ota_kind == "initial_install":
            full_name = f"{item.get('name', feature_id)} {target_name} initial install"
        elif update_scope == "firmware":
            full_name = f"{item.get('name', feature_id)} {target_name} firmware update"
        else:
            full_name = f"{item.get('name', feature_id)} feature update"
        return {
            "feature_id": feature_id,
            "name": item.get("name", feature_id),
            "full_name": full_name,
            "icon": item.get("icon", "FW"),
            "target": target,
            "ecu_target": target,
            "current_version": current_version,
            "latest_version": latest_version,
            "latest_versions": latest_versions,
            "repo": action.get("release_repo") or item.get("release_repo"),
            "action_id": action.get("id", "zcu_flash"),
            "ota_kind": ota_kind,
            "update_scope": update_scope,
            "install_required": ota_kind == "initial_install",
        }

    def _upsert_pending_unlocked(self, pending: dict, bucket: str, pending_item: dict) -> None:
        items = [item for item in pending.get(bucket, []) if isinstance(item, dict)]
        key = (
            pending_item.get("feature_id"),
            pending_item.get("action_id"),
            pending_item.get("ota_kind"),
            pending_item.get("update_scope"),
        )
        items = [
            item
            for item in items
            if (
                item.get("feature_id"),
                item.get("action_id"),
                item.get("ota_kind"),
                item.get("update_scope"),
            ) != key
        ]
        items.append(pending_item)
        pending[bucket] = items

    def _queue_initial_install_unlocked(self, data: dict, item: dict, record: dict) -> None:
        if self._is_record_installed(item, record):
            return
        actions = self._zcu_actions(item)
        if not actions:
            return
        pending = data.setdefault("pending_ota", {"to_install": [], "to_update": [], "checked_at": None})
        firmware_versions = firmware_target_versions(record)
        for action in actions:
            target_name = ota_target_display_name(action.get("target", "zcu"))
            latest_version = (
                firmware_versions.get(target_name)
                or record.get("version")
                or item.get("latest_version")
            )
            pending_item = self._pending_item(
                item,
                action,
                record,
                ota_kind="initial_install",
                update_scope="install",
                current_version=firmware_version(firmware_versions.get(target_name)),
                latest_version=latest_version,
                latest_versions=firmware_versions or {target_name: latest_version},
            )
            self._upsert_pending_unlocked(pending, "to_install", pending_item)
        pending["checked_at"] = utc_now()

    def purchase(self, feature_id: str, *, run_flash: bool = True) -> dict:
        item = self._catalog_item(feature_id)

        with self._lock:
            data = self._load_unlocked()
            record = data["items"][feature_id]
            already_purchased = record["purchased"]

            if not already_purchased:
                record["purchased"] = True
                data["purchase_history"].append(
                    {
                        "feature_id": feature_id,
                        "purchased_at": utc_now(),
                    }
                )

            if item.get("downloadable"):
                self._download_feature_unlocked(data, feature_id, force=True, run_flash=run_flash)
                self._queue_initial_install_unlocked(data, item, data["items"][feature_id])
                self._queue_reset_trigger_unlocked(data, item, data["items"][feature_id])

            data["purchased"] = [
                item_id for item_id, item_record in data["items"].items() if item_record["purchased"]
            ]
            self._save_unlocked(data)

            return {
                "success": True,
                "already_purchased": already_purchased,
                "downloaded": data["items"][feature_id]["package"]["downloaded"],
                "item": data["items"][feature_id],
                "purchased": data["purchased"],
            }

    def download_feature(self, feature_id: str, *, run_flash: bool = True) -> dict:
        self._catalog_item(feature_id)
        with self._lock:
            data = self._load_unlocked()
            if not data["items"][feature_id]["purchased"]:
                raise ValueError(f"store item is not purchased: {feature_id}")
            item = self._catalog_item(feature_id)
            self._download_feature_unlocked(data, feature_id, force=True, run_flash=run_flash)
            self._queue_initial_install_unlocked(data, item, data["items"][feature_id])
            self._queue_reset_trigger_unlocked(data, item, data["items"][feature_id])
            self._save_unlocked(data)
            return {"success": True, "item": data["items"][feature_id]}

    def update_purchased_downloads(self) -> list[dict]:
        updates: list[dict] = []
        with self._lock:
            data = self._load_unlocked()
            changed = False
            for feature_id, record in data["items"].items():
                if not record["purchased"] or not record["package"]["downloadable"]:
                    continue

                before_version = record.get("version")
                before_downloaded = bool(record["package"]["downloaded"])
                try:
                    updated = self._download_feature_unlocked(data, feature_id, force=False)
                except Exception as exc:
                    record["package"]["checked_at"] = utc_now()
                    record["package"]["error"] = f"{type(exc).__name__}: {exc}"
                    changed = True
                    logger.warning("OTA update check failed for %s: %s", feature_id, exc)
                    continue

                changed = True
                after_record = data["items"][feature_id]
                if updated or not before_downloaded:
                    updates.append(
                        {
                            "feature_id": feature_id,
                            "from_version": before_version,
                            "to_version": after_record["version"],
                            "item": after_record,
                        }
                    )

            if changed:
                self._save_unlocked(data)
        return updates

    def check_pending_zcu_updates(self) -> dict:
        with self._lock:
            data = self._load_unlocked()
            existing_pending = data.get("pending_ota", {})
            next_pending = {
                "to_install": [
                    pending_item
                    for pending_item in existing_pending.get("to_install", [])
                    if isinstance(pending_item, dict)
                    and pending_item.get("install_required")
                    and pending_item.get("feature_id") in data["items"]
                    and data["items"][pending_item["feature_id"]].get("purchased")
                    and (
                        bool(pending_item.get("reset_required"))
                        or not self._is_record_installed(
                            self._catalog_item(pending_item["feature_id"]),
                            data["items"][pending_item["feature_id"]],
                        )
                    )
                ],
                "to_update": [],
                "checked_at": utc_now(),
            }

            for item in STORE_CATALOG:
                feature_id = item["id"]
                record = data["items"][feature_id]
                if not record["purchased"]:
                    continue

                if not self._is_record_installed(item, record):
                    self._queue_initial_install_unlocked(data, item, record)
                    for pending_item in data.get("pending_ota", {}).get("to_install", []):
                        if isinstance(pending_item, dict) and pending_item.get("feature_id") == feature_id:
                            self._upsert_pending_unlocked(next_pending, "to_install", pending_item)
                    continue

                if item.get("downloadable"):
                    try:
                        package_action = self.ota_manager.python_package_action(item)
                    except ValueError:
                        package_action = None
                    if package_action:
                        latest_version = None
                        release = None
                        if package_action.get("type") == "github_release_file":
                            try:
                                release = self.ota_manager.fetch_latest_release(item, package_action)
                                latest_version = firmware_version(str(release.get("tag_name") or ""))
                            except Exception as exc:
                                record["package"]["checked_at"] = utc_now()
                                record["package"]["error"] = f"{type(exc).__name__}: {exc}"
                        else:
                            source_version = None
                            if package_action.get("type") == "local_file":
                                try:
                                    source_version = python_module_version(
                                        self.ota_manager.resolve_source_path(package_action)
                                    )
                                except Exception as exc:
                                    record["package"]["checked_at"] = utc_now()
                                    record["package"]["error"] = f"{type(exc).__name__}: {exc}"
                            latest_version = firmware_version(
                                str(source_version or package_action.get("version") or item.get("latest_version") or "")
                            )
                        if latest_version and firmware_version_gt(latest_version, record.get("version")):
                            update_item = self._pending_item(
                                item,
                                package_action,
                                record,
                                ota_kind="version_update",
                                update_scope="feature",
                                current_version=firmware_version(record.get("version")),
                                latest_version=latest_version,
                                latest_versions={FEATURE_VERSION_TARGET: latest_version},
                            )
                            if release:
                                update_item["release_tag"] = release.get("tag_name")
                                update_item["release_url"] = release.get("html_url")
                            self._upsert_pending_unlocked(next_pending, "to_update", update_item)

                zcu = record["zcu_ota"]
                firmware_versions = firmware_target_versions(record)
                for action in self._zcu_actions(item):
                    if not zcu.get("applied"):
                        continue
                    target_name = ota_target_display_name(action.get("target", "zcu"))
                    try:
                        release = self.ota_manager.fetch_latest_release(item, action)
                    except Exception as exc:
                        zcu["checked_at"] = utc_now()
                        zcu["error"] = f"{type(exc).__name__}: {exc}"
                        continue

                    latest_version = firmware_version(str(release.get("tag_name") or "")) or "0.0.0"
                    current_version = firmware_version(firmware_versions.get(target_name) or zcu.get("version"))
                    if not firmware_version_gt(latest_version, current_version):
                        zcu["checked_at"] = utc_now()
                        zcu["error"] = None
                        continue

                    update_item = self._pending_item(
                        item,
                        action,
                        record,
                        ota_kind="version_update",
                        update_scope="firmware",
                        current_version=current_version,
                        latest_version=latest_version,
                        latest_versions={target_name: latest_version},
                    )
                    update_item["release_tag"] = release.get("tag_name")
                    update_item["release_url"] = release.get("html_url")
                    self._upsert_pending_unlocked(next_pending, "to_update", update_item)

            data["pending_ota"] = next_pending
            self._save_unlocked(data)
            return next_pending

    def _queue_reset_trigger_unlocked(self, data: dict, item: dict, record: dict) -> None:
        actions = self._zcu_actions(item)
        if not actions or not record["zcu_ota"].get("applied"):
            return
        pending = data.setdefault("pending_ota", {"to_install": [], "to_update": [], "checked_at": None})
        firmware_versions = firmware_target_versions(record)
        for action in actions:
            target_name = ota_target_display_name(action.get("target", "zcu"))
            latest_version = (
                firmware_versions.get(target_name)
                or record.get("version")
                or item.get("latest_version")
            )
            pending_item = self._pending_item(
                item,
                action,
                record,
                ota_kind="initial_install",
                update_scope="install",
                current_version=firmware_version(firmware_versions.get(target_name)),
                latest_version=latest_version,
                latest_versions=firmware_versions or {target_name: latest_version},
            )
            pending_item["reset_required"] = True
            pending_item["full_name"] = f"{item.get('name', item['id'])} ECU reset trigger"
            self._upsert_pending_unlocked(pending, "to_install", pending_item)
        pending["checked_at"] = utc_now()

    def pending_ota(self) -> dict:
        return self.load().get("pending_ota", {"to_install": [], "to_update": [], "checked_at": None})

    def clear_pending_ota(self) -> dict:
        with self._lock:
            data = self._load_unlocked()
            data["pending_ota"] = {"to_install": [], "to_update": [], "checked_at": utc_now()}
            self._save_unlocked(data)
            return data["pending_ota"]

    def apply_pending_ota(self) -> dict:
        with self._lock:
            data = self._load_unlocked()
            pending = data.get("pending_ota", {})
            pending_items = [
                (bucket, item)
                for bucket in ("to_install", "to_update")
                for item in pending.get(bucket, [])
                if isinstance(item, dict)
            ]
            results = []
            failed_pending = {"to_install": [], "to_update": [], "checked_at": utc_now()}
            seen_features: set[str] = set()
            for bucket, pending_item in pending_items:
                feature_id = str(pending_item.get("feature_id", ""))
                if feature_id in seen_features or feature_id not in data["items"]:
                    continue
                seen_features.add(feature_id)
                item = self._catalog_item(feature_id)
                record = data["items"][feature_id]
                if not record["purchased"]:
                    continue

                try:
                    reset_only = bool(pending_item.get("reset_required")) and self._is_record_installed(item, record)
                    if not reset_only:
                        self._download_feature_unlocked(data, feature_id, force=True)
                    record = data["items"][feature_id]
                    zcu = record["zcu_ota"]
                    success = (
                        self._is_record_installed(item, record)
                        if self._zcu_actions(item)
                        else bool(record["package"]["downloaded"])
                    )
                    results.append(
                        {
                            "feature_id": feature_id,
                            "feature_name": item.get("name", feature_id),
                            "ecu_target": zcu.get("target", "zcu"),
                            "success": success,
                            "version": {
                                FEATURE_VERSION_TARGET: record.get("version"),
                                **firmware_target_versions(record),
                            },
                            "error": zcu.get("error") if not success else None,
                        }
                    )
                    if not success:
                        retry_item = dict(pending_item)
                        retry_item["last_error"] = zcu.get("error") or record["package"].get("error")
                        self._upsert_pending_unlocked(failed_pending, bucket, retry_item)
                except Exception as exc:
                    record["package"]["checked_at"] = utc_now()
                    record["package"]["error"] = f"{type(exc).__name__}: {exc}"
                    results.append(
                        {
                            "feature_id": feature_id,
                            "feature_name": item.get("name", feature_id),
                            "ecu_target": "zcu",
                            "success": False,
                            "version": {"ZCU": pending_item.get("latest_version")},
                            "error": record["package"]["error"],
                        }
                    )
                    retry_item = dict(pending_item)
                    retry_item["last_error"] = record["package"]["error"]
                    self._upsert_pending_unlocked(failed_pending, bucket, retry_item)

            data["pending_ota"] = failed_pending
            self._save_unlocked(data)
            return {
                "timestamp": utc_now(),
                "all_success": all(result["success"] for result in results) if results else True,
                "results": results,
            }

    @staticmethod
    def _remove_feature_pending_unlocked(data: dict, feature_id: str) -> None:
        pending = data.setdefault("pending_ota", {"to_install": [], "to_update": [], "checked_at": None})
        for bucket in ("to_install", "to_update"):
            pending[bucket] = [
                item
                for item in pending.get(bucket, [])
                if not isinstance(item, dict) or item.get("feature_id") != feature_id
            ]
        pending["checked_at"] = utc_now()

    def apply_feature_ota(self, feature_id: str) -> dict:
        item = self._catalog_item(feature_id)
        with self._lock:
            data = self._load_unlocked()
            record = data["items"][feature_id]
            if not record["purchased"]:
                raise ValueError(f"store item is not purchased: {feature_id}")
            if not self._zcu_actions(item):
                return {
                    "timestamp": utc_now(),
                    "all_success": True,
                    "results": [],
                }

            firmware_failed = False
            results = []
            firmware_count = max(1, len(self._zcu_actions(item)))
            for index, action in enumerate(self._zcu_actions(item)):
                action = dict(action)
                action["progress_start"] = int(index * 100 / firmware_count)
                action["progress_end"] = int((index + 1) * 100 / firmware_count)
                target_name = ota_target_display_name(action.get("target", "zcu"))
                zcu = record["zcu_ota"]
                firmware_versions = zcu.setdefault("versions", {})
                if not isinstance(firmware_versions, dict):
                    firmware_versions = {}
                    zcu["versions"] = firmware_versions
                try:
                    zcu_result = self.ota_manager.flash_downloaded_firmware_payload(item, action)
                except Exception as exc:
                    zcu["required"] = True
                    zcu["action_id"] = action.get("id", zcu.get("action_id"))
                    zcu["action_type"] = action.get("type", zcu.get("action_type"))
                    zcu["target"] = action.get("target", zcu.get("target"))
                    zcu["repo"] = action.get("release_repo") or item.get("release_repo")
                    zcu["checked_at"] = utc_now()
                    zcu["error"] = f"{type(exc).__name__}: {exc}"
                    firmware_failed = True
                    results.append(
                        {
                            "feature_id": feature_id,
                            "feature_name": item.get("name", feature_id),
                            "ecu_target": target_name,
                            "success": False,
                            "version": firmware_target_versions(record),
                            "error": zcu["error"],
                        }
                    )
                    break

                zcu["required"] = True
                zcu["downloaded"] = zcu_result.downloaded
                zcu["applied"] = zcu_result.applied
                zcu["path"] = (
                    str(zcu_result.path.relative_to(self.ota_manager.base_dir))
                    if zcu_result.path is not None and zcu_result.path.is_relative_to(self.ota_manager.base_dir)
                    else str(zcu_result.path) if zcu_result.path is not None else None
                )
                zcu["source"] = "github-release"
                zcu["action_id"] = zcu_result.action_id
                zcu["action_type"] = zcu_result.action_type
                zcu["target"] = zcu_result.target
                zcu["repo"] = action.get("release_repo") or item.get("release_repo")
                zcu["version"] = zcu_result.version
                zcu["release_tag"] = zcu_result.release_tag
                zcu["release_url"] = zcu_result.release_url
                zcu["asset_name"] = zcu_result.asset_name or zcu.get("asset_name")
                if zcu_result.version:
                    normalized_version = firmware_version(zcu_result.version) or zcu_result.version
                    firmware_versions[target_name] = normalized_version
                    zcu["versions"] = firmware_versions
                    if not bool(item.get("package_required", True)):
                        record["version"] = normalized_version
                zcu["checked_at"] = zcu_result.checked_at
                if zcu_result.applied:
                    zcu["applied_at"] = utc_now()
                zcu["error"] = zcu_result.error
                results.append(
                    {
                        "feature_id": feature_id,
                        "feature_name": item.get("name", feature_id),
                        "ecu_target": target_name,
                        "success": bool(zcu_result.applied),
                        "version": {
                            FEATURE_VERSION_TARGET: record.get("version"),
                            **firmware_target_versions(record),
                        },
                        "error": zcu_result.error,
                    }
                )
                if not zcu_result.applied:
                    firmware_failed = True
                    break

            if firmware_failed:
                record["zcu_ota"]["applied"] = False
                self._queue_initial_install_unlocked(data, item, record)
            else:
                self._remove_feature_pending_unlocked(data, feature_id)
                self.ota_manager.update_progress(
                    feature_id,
                    action_id="complete",
                    action_type="ota_sequence",
                    target="vehicle",
                    phase="complete",
                    status="complete",
                    percent=100,
                    message=f"{item.get('name', feature_id)} OTA complete",
                    active=False,
                )

            self._save_unlocked(data)
            return {
                "timestamp": utc_now(),
                "all_success": not firmware_failed,
                "results": results,
            }

    def _download_feature_unlocked(
        self,
        data: dict,
        feature_id: str,
        *,
        force: bool,
        run_flash: bool = True,
    ) -> bool:
        item = self._catalog_item(feature_id)
        if not item.get("downloadable"):
            return False

        record = data["items"][feature_id]
        zcu_actions = self._zcu_actions(item)
        firmware_action_progress = bool(force and zcu_actions)
        package_required = bool(item.get("package_required", True))
        package_progress_end = 25 if package_required else 0
        package = record["package"]
        if firmware_action_progress:
            self.ota_manager.update_progress(
                feature_id,
                action_id="prepare",
                action_type="ota_sequence",
                target="vehicle",
                phase="prepare",
                status="preparing",
                percent=0,
                message=(
                    f"{item.get('name', feature_id)} OTA 준비 중"
                    if run_flash
                    else f"{item.get('name', feature_id)} firmware download preparing"
                ),
                active=True,
            )

        if package_required:
            package_action = dict(self.ota_manager.python_package_action(item))
            if firmware_action_progress:
                package_action["progress_start"] = 0
                package_action["progress_end"] = package_progress_end
            result = self.ota_manager.run_action(
                item,
                package_action,
                current_version=record.get("version"),
                force=force,
            )
            package["downloaded"] = result.downloaded
            if result.updated or not result.downloaded:
                package["applied"] = result.applied
            package["path"] = (
                str(result.path.relative_to(self.ota_manager.base_dir))
                if result.path is not None and result.path.is_relative_to(self.ota_manager.base_dir)
                else str(result.path) if result.path is not None else None
            )
            package["source"] = "github-release" if result.release_url else "local-file"
            package["action_id"] = result.action_id
            package["action_type"] = result.action_type
            package["target"] = result.target
            package["repo"] = (
                package_action.get("release_repo") or item.get("release_repo")
                if result.release_url
                else None
            )
            package["release_tag"] = result.release_tag
            package["release_url"] = result.release_url
            package["asset_name"] = result.asset_name or package.get("asset_name")
            package["checked_at"] = result.checked_at
            if result.version:
                record["version"] = result.version
            module_version = python_module_version(result.path)
            if module_version:
                record["version"] = module_version
            if result.downloaded_at:
                package["downloaded_at"] = result.downloaded_at
            if result.updated:
                package["applied_at"] = None
            package["error"] = result.error
            updated = result.updated
        else:
            package["downloaded"] = True
            package["applied"] = True
            package["path"] = None
            package["source"] = "firmware-only"
            package["action_id"] = None
            package["action_type"] = "firmware_only"
            package["target"] = "firmware"
            package["repo"] = item.get("release_repo")
            package["release_tag"] = None
            package["release_url"] = None
            package["asset_name"] = None
            package["checked_at"] = utc_now()
            package["downloaded_at"] = package["downloaded_at"] or package["checked_at"]
            package["applied_at"] = package["applied_at"] or package["checked_at"]
            package["error"] = None
            updated = False
        firmware_failed = False

        firmware_count = max(1, len(zcu_actions))
        for index, action in enumerate(zcu_actions):
            action = dict(action)
            if firmware_action_progress:
                span_start = package_progress_end + int(index * (100 - package_progress_end) / firmware_count)
                span_end = package_progress_end + int((index + 1) * (100 - package_progress_end) / firmware_count)
                action["progress_start"] = span_start
                action["progress_end"] = span_end
            if action.get("type") not in ("doip_uds_flash", "doip_sensor_can_ota"):
                continue
            if not force:
                continue
            zcu = record["zcu_ota"]
            target_name = ota_target_display_name(action.get("target", "zcu"))
            firmware_versions = zcu.setdefault("versions", {})
            if not isinstance(firmware_versions, dict):
                firmware_versions = {}
                zcu["versions"] = firmware_versions
            try:
                current_firmware_version = (
                    firmware_versions.get(target_name)
                    or zcu.get("version")
                    or record.get("version")
                )
                if run_flash:
                    zcu_result = self.ota_manager.run_action(
                        item,
                        action,
                        current_version=current_firmware_version,
                        force=force,
                    )
                else:
                    zcu_result = self.ota_manager.download_firmware_payload(
                        item,
                        action,
                        current_version=current_firmware_version,
                        force=force,
                    )
            except Exception as exc:
                zcu["required"] = True
                zcu["action_id"] = action.get("id", zcu.get("action_id"))
                zcu["action_type"] = action.get("type", zcu.get("action_type"))
                zcu["target"] = action.get("target", zcu.get("target"))
                zcu["repo"] = action.get("release_repo") or item.get("release_repo")
                zcu["checked_at"] = utc_now()
                zcu["error"] = f"{type(exc).__name__}: {exc}"
                firmware_failed = True
                break

            zcu["required"] = True
            zcu["downloaded"] = zcu_result.downloaded
            zcu["applied"] = zcu_result.applied
            zcu["path"] = (
                str(zcu_result.path.relative_to(self.ota_manager.base_dir))
                if zcu_result.path is not None and zcu_result.path.is_relative_to(self.ota_manager.base_dir)
                else str(zcu_result.path) if zcu_result.path is not None else None
            )
            zcu["source"] = "github-release"
            zcu["action_id"] = zcu_result.action_id
            zcu["action_type"] = zcu_result.action_type
            zcu["target"] = zcu_result.target
            zcu["repo"] = action.get("release_repo") or item.get("release_repo")
            zcu["version"] = zcu_result.version
            zcu["release_tag"] = zcu_result.release_tag
            zcu["release_url"] = zcu_result.release_url
            zcu["asset_name"] = zcu_result.asset_name or zcu.get("asset_name")
            if zcu_result.version:
                normalized_version = firmware_version(zcu_result.version) or zcu_result.version
                firmware_versions[target_name] = normalized_version
                zcu["versions"] = firmware_versions
                if not package_required:
                    record["version"] = normalized_version
            zcu["checked_at"] = zcu_result.checked_at
            if zcu_result.downloaded_at:
                zcu["downloaded_at"] = zcu_result.downloaded_at
            if zcu_result.updated:
                zcu["applied_at"] = utc_now() if zcu_result.applied else None
            zcu["error"] = zcu_result.error
            updated = updated or zcu_result.updated
            if run_flash and not zcu_result.applied:
                firmware_failed = True
                break

        if firmware_failed:
            record["zcu_ota"]["applied"] = False
        elif run_flash and force and zcu_actions:
            self.ota_manager.update_progress(
                feature_id,
                action_id="complete",
                action_type="ota_sequence",
                target="vehicle",
                phase="complete",
                status="complete",
                percent=100,
                message=f"{item.get('name', feature_id)} OTA 완료",
                active=False,
            )
        elif force and zcu_actions:
            self.ota_manager.update_progress(
                feature_id,
                action_id="download_complete",
                action_type="ota_sequence",
                target="vehicle",
                phase="complete",
                status="downloaded",
                percent=100,
                message=f"{item.get('name', feature_id)} firmware download complete",
                active=False,
            )

        return updated

    def feature_record(self, feature_id: str) -> dict:
        self._catalog_item(feature_id)
        return self.load()["items"][feature_id]

    def is_feature_enabled(self, feature_id: str) -> bool:
        item = self._catalog_item(feature_id)
        record = self.feature_record(feature_id)
        return bool(record["enabled"] and self._is_record_installed(item, record))

    def is_feature_installed(self, feature_id: str) -> bool:
        item = self._catalog_item(feature_id)
        return self._is_record_installed(item, self.feature_record(feature_id))

    def set_feature_enabled(self, feature_id: str, enabled: bool) -> dict:
        self._catalog_item(feature_id)
        with self._lock:
            data = self._load_unlocked()
            data["items"][feature_id]["enabled"] = bool(enabled)
            self._save_unlocked(data)
            return data["items"][feature_id]

    def mark_feature_runtime(
        self,
        feature_id: str,
        *,
        downloaded: bool,
        applied: bool,
        error: str | None,
        version: str | None = None,
    ) -> None:
        item = self._catalog_item(feature_id)
        if item.get("downloadable") and not item.get("package_required", True):
            return
        with self._lock:
            data = self._load_unlocked()
            record = data["items"][feature_id]
            package = record["package"]
            changed = (
                package["downloaded"] != downloaded
                or package["applied"] != applied
                or package["error"] != error
                or (bool(version) and record.get("version") != version)
            )
            if not changed:
                return

            package["downloaded"] = downloaded
            package["applied"] = applied
            package["error"] = error
            package["applied_at"] = utc_now() if applied else None
            if version:
                record["version"] = version
            self._save_unlocked(data)

    def purchased_ids(self) -> list[str]:
        return self.load()["purchased"]

    def downloaded_ids(self) -> list[str]:
        data = self.load()
        return [
            feature_id
            for feature_id, record in data["items"].items()
            if self._catalog_item(feature_id).get("downloadable")
            and self._is_record_installed(self._catalog_item(feature_id), record)
        ]

    def versions(self) -> dict:
        data = self.load()
        versions = {}
        for feature_id, record in data["items"].items():
            item = self._catalog_item(feature_id)
            if not item.get("downloadable") or not self._is_record_installed(item, record):
                continue
            item_versions = {FEATURE_VERSION_TARGET: record["version"]}
            if record["zcu_ota"].get("applied"):
                item_versions.update(firmware_target_versions(record))
            versions[feature_id] = item_versions
        return versions


class VehicleStatus:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._gear = GEAR_P
        self._state = "parking"
        self._ui_theme = "dark"
        self._updated_at = utc_now()
        self._transition_count = 0
        self._speed_raw = 0
        self._speed_kmh = 0.0
        self._speed_updated_at: str | None = None
        self._tof_mm = 0
        self._tof_updated_at: str | None = None
        self._vehicle_event_updated_at: str | None = None
        self._vehicle_event_name: str | None = None
        self._vehicle_event_count = 0
        self._aeb_triggered_at: str | None = None
        self._aeb_trigger_count = 0

    def apply_gear(self, gear: str) -> dict:
        if gear not in (GEAR_P, GEAR_D):
            raise ValueError(f"unsupported gear: {gear}")

        next_state = "driving" if gear == GEAR_D else "parking"
        with self._lock:
            if gear != self._gear or next_state != self._state:
                self._gear = gear
                self._state = next_state
                self._updated_at = utc_now()
                self._transition_count += 1
                logger.info("vehicle status changed: gear=%s state=%s", self._gear, self._state)
            return self.to_dict()

    def apply_speed_event(self, speed_raw: int) -> dict:
        speed_raw = max(0, int(speed_raw))
        with self._lock:
            self._speed_raw = speed_raw
            self._speed_kmh = speed_raw / 100.0
            self._speed_updated_at = utc_now()
            self._vehicle_event_updated_at = self._speed_updated_at
            self._vehicle_event_name = "speed"
            self._vehicle_event_count += 1
            return self.to_dict()

    def apply_tof_event(self, tof_mm: int) -> dict:
        tof_mm = max(0, int(tof_mm))
        with self._lock:
            self._tof_mm = tof_mm
            self._tof_updated_at = utc_now()
            self._vehicle_event_updated_at = self._tof_updated_at
            self._vehicle_event_name = "tof"
            self._vehicle_event_count += 1
            return self.to_dict()

    def mark_vehicle_event(self, event_name: str) -> None:
        with self._lock:
            self._vehicle_event_updated_at = utc_now()
            self._vehicle_event_name = event_name
            self._vehicle_event_count += 1

    def mark_aeb_trigger(self) -> None:
        with self._lock:
            now = utc_now()
            self._vehicle_event_updated_at = now
            self._vehicle_event_name = "aeb_trigger"
            self._vehicle_event_count += 1
            self._aeb_triggered_at = now
            self._aeb_trigger_count += 1

    def aeb_trigger_snapshot(self, active_seconds: float = 5.0) -> dict[str, Any]:
        with self._lock:
            age_seconds = seconds_since(self._aeb_triggered_at)
            active = age_seconds is not None and age_seconds <= active_seconds
            return {
                "active": active,
                "triggered_at": self._aeb_triggered_at,
                "age_seconds": round(age_seconds, 3) if age_seconds is not None else None,
                "count": self._aeb_trigger_count,
            }

    def sensor_snapshot(self) -> dict:
        with self._lock:
            return {
                "tof_mm": self._tof_mm,
                "tof_updated_at": self._tof_updated_at,
                "speed_raw": self._speed_raw,
                "speed_kmh": self._speed_kmh,
                "speed_updated_at": self._speed_updated_at,
            }

    def set_theme(self, theme: str) -> dict:
        if theme not in ("dark", "light", "blue", "luffy"):
            raise ValueError(f"unsupported theme: {theme}")
        with self._lock:
            self._ui_theme = theme
            self._updated_at = utc_now()
            return self.to_dict()

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "gear": self._gear,
                "speed_raw": self._speed_raw,
                "speed_kmh": self._speed_kmh,
                "speed_updated_at": self._speed_updated_at,
                "tof_mm": self._tof_mm,
                "tof_updated_at": self._tof_updated_at,
                "vehicle_event_updated_at": self._vehicle_event_updated_at,
                "vehicle_event_name": self._vehicle_event_name,
                "vehicle_event_count": self._vehicle_event_count,
                "aeb_triggered_at": self._aeb_triggered_at,
                "aeb_trigger_count": self._aeb_trigger_count,
                "ui_theme": self._ui_theme,
                "updated_at": self._updated_at,
                "transition_count": self._transition_count,
            }

    def vehicle_event_snapshot(self, stale_seconds: float) -> dict[str, Any]:
        with self._lock:
            age_seconds = seconds_since(self._vehicle_event_updated_at)
            connected = age_seconds is not None and age_seconds <= stale_seconds
            return {
                "connected": connected,
                "status": "connected" if connected else "disconnected",
                "last_event": self._vehicle_event_name,
                "last_event_at": self._vehicle_event_updated_at,
                "last_event_age_seconds": round(age_seconds, 3) if age_seconds is not None else None,
                "event_count": self._vehicle_event_count,
                "stale_seconds": stale_seconds,
            }

    def dashboard_payload(self, purchased: list[str] | None = None) -> dict:
        purchased = purchased or []
        with self._lock:
            speed = round(self._speed_kmh, 1)
            return {
                "type": "status",
                "speed": speed,
                "gear": self._gear,
                "direction": "STOP" if speed == 0 else "DRIVE",
                "time": time.strftime("%H:%M:%S"),
                "date": time.strftime("%Y-%m-%d"),
                "network": {
                    "status": "connected",
                    "server_url": f"http://{HOST}:{PORT}",
                },
                "purchased": purchased,
                "installed": [],
                "active": {},
                "versions": {},
                "ui_theme": self._ui_theme,
                "balance": 0,
                "firmware_versions": {},
                "available_themes": ["luffy"] if "LUFFY_THEME" in purchased else [],
            }


class FirmwareVersionStore:
    def __init__(self, *, network_lock: Any | None = None) -> None:
        self._lock = threading.RLock()
        self._network_lock = network_lock
        self._versions: dict[str, str] = {
            VEHICLE_COMPUTER_VERSION_TARGET: VEHICLE_COMPUTER_FIRMWARE_VERSION,
            "ZCU": "-",
            "MotorECU": "-",
            "SensorECU": "-",
            "CameraECU": "-",
            "AEB": "-",
        }
        self._last_error: dict[str, str] = {}
        self._last_success_at: str | None = None
        self._last_success_target: str | None = None
        self._suspended_reason: str | None = None
        self._session_id = 0

    def snapshot(self) -> dict[str, str]:
        with self._lock:
            return dict(self._versions)

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "versions": dict(self._versions),
                "errors": dict(self._last_error),
                "last_success_at": self._last_success_at,
                "last_success_target": self._last_success_target,
                "suspended": self._suspended_reason is not None,
                "suspended_reason": self._suspended_reason,
                "enabled": FIRMWARE_VERSION_QUERY_ENABLED,
                "service_id": FIRMWARE_VERSION_SERVICE_ID,
                "methods": {
                    board_name: {
                        "method_id": config["method_id"],
                        "method_name": config["method_name"],
                    }
                    for board_name, config in BOARD_VERSION_CONFIGS.items()
                },
            }

    def poll_once(self) -> dict[str, str]:
        if not FIRMWARE_VERSION_QUERY_ENABLED:
            return self.snapshot()
        with self._lock:
            if self._suspended_reason is not None:
                return dict(self._versions)

        for board_name, config in BOARD_VERSION_CONFIGS.items():
            with self._lock:
                if self._suspended_reason is not None:
                    break
            try:
                version = self._query_board_version(board_name, config)
            except InterruptedError:
                break
            except Exception as exc:
                with self._lock:
                    self._last_error[board_name] = f"{type(exc).__name__}: {exc}"
                continue

            with self._lock:
                self._versions[board_name] = version
                self._last_success_at = utc_now()
                self._last_success_target = board_name
                self._last_error.pop(board_name, None)

        return self.snapshot()

    def communication_snapshot(self, stale_seconds: float) -> dict[str, Any]:
        with self._lock:
            age_seconds = seconds_since(self._last_success_at)
            connected = age_seconds is not None and age_seconds <= stale_seconds
            return {
                "connected": connected,
                "status": "connected" if connected else "disconnected",
                "last_success_at": self._last_success_at,
                "last_success_target": self._last_success_target,
                "last_success_age_seconds": round(age_seconds, 3) if age_seconds is not None else None,
                "stale_seconds": stale_seconds,
                "enabled": FIRMWARE_VERSION_QUERY_ENABLED,
                "suspended": self._suspended_reason is not None,
                "suspended_reason": self._suspended_reason,
            }

    def set_suspended(self, reason: str | None) -> None:
        with self._lock:
            self._suspended_reason = reason

    def _query_board_version(self, board_name: str, config: dict[str, Any]) -> str:
        with self._lock:
            self._session_id = (self._session_id + 1) & 0xFFFF
            session_id = self._session_id or 1

        method_id = int(config["method_id"])
        packet = build_someip_packet(
            b"",
            service_id=FIRMWARE_VERSION_SERVICE_ID,
            method_id=method_id,
            client_id=DRIVE_CLIENT_ID,
            session_id=session_id,
        )

        def send_query() -> Any:
            with self._lock:
                if self._suspended_reason is not None:
                    raise InterruptedError("firmware version polling suspended")
            return send_ethernet_message(
                FIRMWARE_VERSION_PROTOCOL,
                str(config["host"]),
                int(config["port"]),
                packet,
                timeout_seconds=FIRMWARE_VERSION_TIMEOUT_SECONDS,
                expect_response=True,
            )

        if self._network_lock is not None:
            with self._network_lock:
                response = send_query()
        else:
            response = send_query()
        if not response.response_base64:
            raise RuntimeError("empty SOME/IP response")

        raw = base64.b64decode(response.response_base64)
        message = parse_someip_packet(raw)
        if message is None:
            raise RuntimeError("invalid SOME/IP response")
        if message.service_id != FIRMWARE_VERSION_SERVICE_ID:
            raise RuntimeError(f"unexpected service id: 0x{message.service_id:04x}")
        if message.method_id != method_id:
            raise RuntimeError(f"unexpected method id: 0x{message.method_id:04x}")
        if message.return_code != 0:
            raise RuntimeError(f"SOME/IP return code: 0x{message.return_code:02x}")

        version = self._decode_version_payload(message.payload)
        if not version:
            raise RuntimeError(f"empty firmware version for {board_name}")
        return version

    @staticmethod
    def _decode_version_payload(payload: bytes) -> str:
        if not payload:
            return ""
        return payload[:10].rstrip(b"\x00 ").decode("ascii", errors="replace").strip()


class InternetConnectivityStatus:
    def __init__(
        self,
        *,
        host: str,
        port: int,
        timeout_seconds: float,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_seconds = timeout_seconds
        self._lock = threading.RLock()
        self._status = "checking"
        self._checked_at: str | None = None
        self._latency_ms: int | None = None
        self._error: str | None = None

    def check_once(self) -> dict[str, Any]:
        started = time.monotonic()
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout_seconds):
                latency_ms = int((time.monotonic() - started) * 1000)
            status = "connected"
            error = None
        except OSError as exc:
            latency_ms = None
            status = "disconnected"
            error = str(exc)

        with self._lock:
            self._status = status
            self._checked_at = utc_now()
            self._latency_ms = latency_ms
            self._error = error
            return self.snapshot()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "status": self._status,
                "connected": self._status == "connected",
                "target": f"{self.host}:{self.port}",
                "host": self.host,
                "port": self.port,
                "checked_at": self._checked_at,
                "latency_ms": self._latency_ms,
                "error": self._error,
            }


class PingReachabilityStatus:
    def __init__(
        self,
        *,
        host: str,
        timeout_seconds: float,
        enabled: bool = True,
        network_lock: Any | None = None,
    ) -> None:
        self.host = host
        self.timeout_seconds = timeout_seconds
        self.enabled = enabled
        self._network_lock = network_lock
        self._lock = threading.RLock()
        self._status = "checking" if enabled else "disabled"
        self._checked_at: str | None = None
        self._latency_ms: int | None = None
        self._error: str | None = None
        self._suspended_reason: str | None = None

    def check_once(self) -> dict[str, Any]:
        if not self.enabled:
            with self._lock:
                self._status = "disabled"
                self._checked_at = utc_now()
                self._latency_ms = None
                self._error = None
                return self.snapshot()
        with self._lock:
            if self._suspended_reason is not None:
                return self.snapshot()

        timeout_ms = max(1, int(self.timeout_seconds * 1000))
        if os.name == "nt":
            command = ["ping", "-n", "1", "-w", str(timeout_ms), self.host]
        else:
            command = ["ping", "-c", "1", "-W", str(max(1, int(self.timeout_seconds))), self.host]

        def run_ping() -> subprocess.CompletedProcess[str]:
            with self._lock:
                if self._suspended_reason is not None:
                    raise InterruptedError("ping suspended")
            return subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self.timeout_seconds + 0.5,
            )

        started = time.monotonic()
        try:
            if self._network_lock is not None:
                with self._network_lock:
                    completed = run_ping()
            else:
                completed = run_ping()
            latency_ms = int((time.monotonic() - started) * 1000)
            status = "connected" if completed.returncode == 0 else "disconnected"
            error = None if completed.returncode == 0 else (completed.stderr or completed.stdout).strip()
        except InterruptedError:
            with self._lock:
                return self.snapshot()
        except (OSError, subprocess.TimeoutExpired) as exc:
            latency_ms = None
            status = "disconnected"
            error = str(exc)

        with self._lock:
            self._status = status
            self._checked_at = utc_now()
            self._latency_ms = latency_ms if status == "connected" else None
            self._error = error
            return self.snapshot()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "status": self._status,
                "connected": self._status == "connected",
                "target": self.host,
                "host": self.host,
                "checked_at": self._checked_at,
                "latency_ms": self._latency_ms,
                "error": self._error,
                "enabled": self.enabled,
                "suspended": self._suspended_reason is not None,
                "suspended_reason": self._suspended_reason,
            }

    def set_suspended(self, reason: str | None) -> None:
        with self._lock:
            self._suspended_reason = reason


def placeholder_worker(name: str, interval_seconds: float) -> ChildTarget:
    def run(stop_event: threading.Event, heartbeat: HeartbeatCallback) -> None:
        worker_logger = logging.getLogger(f"vehicle-computer.{name}")
        worker_logger.debug("ready")
        while not stop_event.is_set():
            heartbeat()
            stop_event.wait(interval_seconds)
        worker_logger.debug("stop requested")

    return run


def internet_connectivity_worker(
    internet_status: InternetConnectivityStatus,
    interval_seconds: float,
) -> ChildTarget:
    def run(stop_event: threading.Event, heartbeat: HeartbeatCallback) -> None:
        net_logger = logging.getLogger("vehicle-computer.internet-connectivity")
        net_logger.debug("ready")
        while not stop_event.is_set():
            heartbeat()
            before = internet_status.snapshot().get("status")
            result = internet_status.check_once()
            if result.get("status") != before:
                net_logger.info(
                    "internet %s via %s",
                    result.get("status"),
                    result.get("target"),
                )
            stop_event.wait(interval_seconds)
        net_logger.debug("stop requested")

    return run


def vehicle_link_ping_worker(
    ping_status: PingReachabilityStatus,
    interval_seconds: float,
) -> ChildTarget:
    def run(stop_event: threading.Event, heartbeat: HeartbeatCallback) -> None:
        ping_logger = logging.getLogger("vehicle-computer.vehicle-link-ping")
        ping_logger.debug("ready")
        while not stop_event.is_set():
            heartbeat()
            before = ping_status.snapshot().get("status")
            result = ping_status.check_once()
            if result.get("status") != before:
                ping_logger.info(
                    "vehicle link ping %s via %s",
                    result.get("status"),
                    result.get("target"),
                )
            stop_event.wait(interval_seconds)
        ping_logger.debug("stop requested")

    return run


def vehicle_event_worker(
    vehicle_status: VehicleStatus,
    vehicle_control: VehicleControl,
    host: str,
    port: int,
) -> ChildTarget:
    def run(stop_event: threading.Event, heartbeat: HeartbeatCallback) -> None:
        event_logger = logging.getLogger("vehicle-computer.vehicle-events")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((host, port))
            sock.settimeout(0.5)
            event_logger.debug("vehicle event RX listening: %s:%s", host, port)

            while not stop_event.is_set():
                heartbeat()
                try:
                    data, _addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue

                message = parse_someip_packet(data)
                if message is None:
                    continue
                event_name = f"someip:{message.service_id:04x}/{message.method_id:04x}"

                if (
                    message.service_id == SENSOR_SERVICE_ID
                    and message.method_id == TOF_VALUE_UPDATED_EVENT_ID
                    and len(message.payload) >= 2
                ):
                    tof_mm = message.payload[0] | (message.payload[1] << 8)
                    vehicle_status.apply_tof_event(tof_mm)
                    continue

                if (
                    message.service_id == SENSOR_SERVICE_ID
                    and message.method_id == SPEED_UPDATED_EVENT_ID
                    and len(message.payload) >= 2
                ):
                    speed_kmh = message.payload[0] | (message.payload[1] << 8)
                    vehicle_status.apply_speed_event(speed_kmh)
                    continue

                if (
                    message.service_id == AEB_SERVICE_ID
                    and message.method_id == AEB_TRIGGER_EVENT_ID
                ):
                    vehicle_status.mark_aeb_trigger()
                    event_logger.info("AEB triggered event received")
                    vehicle_control.trigger_aeb()
                    continue

                vehicle_status.mark_vehicle_event(event_name)

    return run


def ota_update_worker(
    feature_state_store: FeatureStateStore,
    vehicle_control: VehicleControl,
    interval_seconds: float,
) -> ChildTarget:
    def run(stop_event: threading.Event, heartbeat: HeartbeatCallback) -> None:
        ota_logger = logging.getLogger("vehicle-computer.ota-updater")
        ota_logger.debug("ready")
        while not stop_event.is_set():
            heartbeat()
            if feature_state_store.ota_manager.is_flashing():
                stop_event.wait(interval_seconds)
                continue
            try:
                pending = feature_state_store.check_pending_zcu_updates()
                count = len(pending.get("to_install", [])) + len(pending.get("to_update", []))
                if count:
                    ota_logger.info("pending OTA install/update count: %s", count)
            except Exception as exc:
                ota_logger.warning("ZCU OTA update check failed: %s", exc)
            stop_event.wait(interval_seconds)
        ota_logger.debug("stop requested")

    return run


def firmware_version_worker(
    firmware_versions: FirmwareVersionStore,
    interval_seconds: float,
) -> ChildTarget:
    def run(stop_event: threading.Event, heartbeat: HeartbeatCallback) -> None:
        version_logger = logging.getLogger("vehicle-computer.firmware-versions")
        version_logger.debug("ready")
        while not stop_event.is_set():
            heartbeat()
            try:
                firmware_versions.poll_once()
            except Exception as exc:
                version_logger.warning("firmware version polling failed: %s", exc)
            stop_event.wait(interval_seconds)
        version_logger.debug("stop requested")

    return run


class VehiclePacketSender:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.enabled = VEHICLE_TX_ENABLED
        self.protocol = VEHICLE_TX_PROTOCOL
        self.host = VEHICLE_TX_HOST
        self.port = VEHICLE_TX_PORT
        self.timeout_seconds = VEHICLE_TX_TIMEOUT_SECONDS
        self._last_error: str | None = None
        self._last_payload_hex: str | None = None
        self._last_control_payload_hex: str | None = None
        self._last_attempt_at: str | None = None
        self._last_sent_at: str | None = None
        self._suspended_reason: str | None = None

    def send(self, payload: bytes) -> None:
        payload_hex = payload.hex(" ")
        control_payload_hex = payload[-2:].hex(" ") if len(payload) >= 2 else None
        with self._lock:
            self._last_payload_hex = payload_hex
            self._last_control_payload_hex = control_payload_hex
            suspended = self._suspended_reason is not None
        if suspended:
            return
        if not self.enabled:
            return
        attempted_at = utc_now()
        with self._lock:
            self._last_attempt_at = attempted_at
        try:
            send_ethernet_message(
                self.protocol,
                self.host,
                self.port,
                payload,
                timeout_seconds=self.timeout_seconds,
                expect_response=False,
            )
            with self._lock:
                if self._last_error is not None:
                    logger.debug("vehicle control packet TX recovered")
                self._last_error = None
                self._last_sent_at = attempted_at
        except OSError as exc:
            error = str(exc)
            with self._lock:
                if error != self._last_error:
                    logger.error("vehicle control packet TX failed: %s", error)
                self._last_error = error

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "enabled": self.enabled,
                "protocol": self.protocol,
                "host": self.host,
                "port": self.port,
                "timeout_seconds": self.timeout_seconds,
                "last_payload_hex": self._last_payload_hex,
                "last_control_payload_hex": self._last_control_payload_hex,
                "last_attempt_at": self._last_attempt_at,
                "last_sent_at": self._last_sent_at,
                "last_error": self._last_error,
                "suspended": self._suspended_reason is not None,
                "suspended_reason": self._suspended_reason,
            }

    def set_suspended(self, reason: str | None) -> None:
        with self._lock:
            if reason and self._suspended_reason != reason:
                logger.info("vehicle control TX suspended: %s", reason)
            if reason is None and self._suspended_reason is not None:
                logger.info("vehicle control TX resumed")
            self._suspended_reason = reason


def api_server_worker(
    supervisor: Supervisor,
    vehicle_status: VehicleStatus,
    firmware_versions: FirmwareVersionStore,
    vehicle_control: VehicleControl,
    packet_sender: VehiclePacketSender,
    internet_status: InternetConnectivityStatus,
    vehicle_link_ping: PingReachabilityStatus,
    front_zcu_ping: PingReachabilityStatus,
    feature_state_store: FeatureStateStore,
    ota_manager: OtaManager,
    host: str,
    port: int,
) -> ChildTarget:
    def run(stop_event: threading.Event, heartbeat: HeartbeatCallback) -> None:
        try:
            import asyncio
            import uvicorn
            from ethernet import parse_payload, send_ethernet_message
            from fastapi import BackgroundTasks, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
            from fastapi.responses import FileResponse, Response
            from fastapi.staticfiles import StaticFiles
            from pydantic import BaseModel, Field
            from starlette.concurrency import run_in_threadpool
        except ImportError as exc:
            raise RuntimeError(
                "FastAPI runtime dependencies are missing. Install fastapi and uvicorn."
            ) from exc

        class EthernetSendRequest(BaseModel):
            protocol: Literal["tcp", "udp"] = "udp"
            host: str
            port: int = Field(..., ge=1, le=65535)
            payload: Any = ""
            payload_format: Literal["text", "hex", "base64", "json", "empty"] = "text"
            timeout_seconds: float = Field(2.0, gt=0, le=30)
            expect_response: bool = False
            receive_bytes: int = Field(4096, ge=1, le=65535)
            local_host: str | None = None
            local_port: int | None = None

        class FeatureToggleRequest(BaseModel):
            enabled: bool

        class StorePurchaseRequest(BaseModel):
            feature_id: str

        class FeatureToggleByIdRequest(BaseModel):
            feature_id: str

        class ThemeRequest(BaseModel):
            theme: Literal["dark", "light", "blue", "luffy"]

        class OtaDecisionRequest(BaseModel):
            do_update: bool

        class ManualFlashRequest(BaseModel):
            board_id: Literal["zcu", "sensor-ecu"]
            filename: str
            content_base64: str

        weather_cache: dict[str, Any] = {"payload": None, "fetched_at": 0.0}

        def fallback_weather(error: str | None = None) -> dict:
            payload = {
                "current": {
                    "temperature_2m": 20,
                    "apparent_temperature": 20,
                    "relative_humidity_2m": 50,
                    "wind_speed_10m": 0,
                    "visibility": 10000,
                    "weather_code": 0,
                },
                "source": "fallback",
            }
            if error:
                payload["error"] = error
            return payload

        def fetch_weather() -> dict:
            cached = weather_cache.get("payload")
            fetched_at = float(weather_cache.get("fetched_at") or 0.0)
            if isinstance(cached, dict) and time.time() - fetched_at < WEATHER_CACHE_SECONDS:
                return {**cached, "cached": True}

            import urllib.parse
            import urllib.request

            query = urllib.parse.urlencode(
                {
                    "latitude": WEATHER_LATITUDE,
                    "longitude": WEATHER_LONGITUDE,
                    "current": ",".join(
                        [
                            "temperature_2m",
                            "apparent_temperature",
                            "relative_humidity_2m",
                            "wind_speed_10m",
                            "visibility",
                            "weather_code",
                        ]
                    ),
                    "timezone": "auto",
                }
            )
            url = f"https://api.open-meteo.com/v1/forecast?{query}"
            try:
                with urllib.request.urlopen(url, timeout=WEATHER_TIMEOUT_SECONDS) as response:
                    data = json.loads(response.read().decode("utf-8"))
                current = data.get("current")
                if not isinstance(current, dict):
                    raise RuntimeError("weather response missing current data")
                payload = {"current": current, "source": "open-meteo"}
                weather_cache["payload"] = payload
                weather_cache["fetched_at"] = time.time()
                return payload
            except Exception as exc:
                if isinstance(cached, dict):
                    return {**cached, "cached": True, "source": "cache", "error": str(exc)}
                logger.warning("weather fetch failed: %s", exc)
                return fallback_weather(str(exc))

        def refresh_feature_runtime(feature_id: str) -> None:
            if feature_id in ("AEB", "LKAS", "FVSA"):
                vehicle_control.reload_feature_state()
                vehicle_control.refresh_feature_runtime(feature_id)

        ota_reset_session_lock = threading.Lock()
        ota_reset_session_id = 0
        ota_reset_event_map = {
            "front-zcu": {
                "event_id": OTA_FRONT_ZCU_TRIGGER_EVENT_ID,
                "event_name": "OtaFrontZcuTriggerd",
            },
            "drive-ecu": {
                "event_id": OTA_DRIVE_ECU_TRIGGER_EVENT_ID,
                "event_name": "OtaDriveEcuTriggerd",
            },
            "sensor-ecu": {
                "event_id": OTA_SENSOR_ECU_TRIGGER_EVENT_ID,
                "event_name": "OtaSensorEcuTriggerd",
            },
        }
        ota_reset_target_aliases = {
            "zcu": "front-zcu",
            "front": "front-zcu",
            "frontzcu": "front-zcu",
            "front-zcu": "front-zcu",
            "drive": "drive-ecu",
            "driveecu": "drive-ecu",
            "drive-ecu": "drive-ecu",
            "motor": "drive-ecu",
            "motorecu": "drive-ecu",
            "motor-ecu": "drive-ecu",
            "sensor": "sensor-ecu",
            "sensorecu": "sensor-ecu",
            "sensor-ecu": "sensor-ecu",
        }

        def next_ota_reset_session_id() -> int:
            nonlocal ota_reset_session_id
            with ota_reset_session_lock:
                ota_reset_session_id = (ota_reset_session_id + 1) & 0xFFFF
                return ota_reset_session_id or 1

        def normalize_ota_reset_target(value: Any) -> str | None:
            if value is None:
                return None
            key = str(value).strip().lower().replace("_", "-").replace(" ", "")
            return ota_reset_target_aliases.get(key)

        def ota_reset_targets_from_pending(pending: dict, apply_result: dict) -> list[str]:
            apply_results = [result for result in apply_result.get("results", []) if isinstance(result, dict)]
            successful_features = {
                str(result.get("feature_id"))
                for result in apply_results
                if result.get("success")
            }
            targets: list[str] = []
            for bucket in ("to_install", "to_update"):
                for pending_item in pending.get(bucket, []):
                    if not isinstance(pending_item, dict):
                        continue
                    if apply_results and str(pending_item.get("feature_id")) not in successful_features:
                        continue
                    if pending_item.get("update_scope") == "feature":
                        continue

                    raw_targets = pending_item.get("targets")
                    values = list(raw_targets) if isinstance(raw_targets, list) else []
                    values.extend([pending_item.get("ecu_target"), pending_item.get("target")])
                    for value in values:
                        target = normalize_ota_reset_target(value)
                        if target and target not in targets:
                            targets.append(target)
            return targets

        def send_ota_reset_triggers(pending: dict, apply_result: dict) -> dict:
            targets = ota_reset_targets_from_pending(pending, apply_result)
            if not targets:
                return {"success": True, "enabled": OTA_RESET_TRIGGER_ENABLED, "triggers": []}

            triggers = []
            for target in targets:
                event = ota_reset_event_map[target]
                payload = {
                    "target": target,
                    "service_id": f"0x{OTA_RESET_SERVICE_ID:04X}",
                    "event_id": f"0x{event['event_id']:04X}",
                    "event_name": event["event_name"],
                    "host": OTA_RESET_TRIGGER_HOST,
                    "port": OTA_RESET_TRIGGER_PORT,
                    "success": False,
                }

                if not OTA_RESET_TRIGGER_ENABLED:
                    payload.update({"success": True, "skipped": True, "message": "OTA reset trigger is disabled"})
                    triggers.append(payload)
                    continue

                try:
                    packet = build_someip_packet(
                        b"",
                        service_id=OTA_RESET_SERVICE_ID,
                        method_id=event["event_id"],
                        client_id=OTA_RESET_TRIGGER_CLIENT_ID,
                        session_id=next_ota_reset_session_id(),
                        message_type=0x02,
                    )
                    with ZCU_NETWORK_LOCK:
                        response = send_ethernet_message(
                            OTA_RESET_TRIGGER_PROTOCOL,
                            OTA_RESET_TRIGGER_HOST,
                            OTA_RESET_TRIGGER_PORT,
                            packet,
                            timeout_seconds=OTA_RESET_TRIGGER_TIMEOUT_SECONDS,
                            expect_response=False,
                        )
                    payload.update({"success": True, "bytes_sent": response.bytes_sent})
                    logger.info(
                        "OTA reset trigger sent: %s service=0x%04x event=0x%04x target=%s:%s",
                        event["event_name"],
                        OTA_RESET_SERVICE_ID,
                        event["event_id"],
                        OTA_RESET_TRIGGER_HOST,
                        OTA_RESET_TRIGGER_PORT,
                    )
                except Exception as exc:
                    payload.update({"success": False, "error": f"{type(exc).__name__}: {exc}"})
                    logger.warning("OTA reset trigger failed for %s: %s", target, exc)

                triggers.append(payload)

            return {
                "success": all(trigger.get("success") for trigger in triggers),
                "enabled": OTA_RESET_TRIGGER_ENABLED,
                "triggers": triggers,
            }

        def vehicle_computer_connection_payload() -> dict[str, Any]:
            event_status = vehicle_status.vehicle_event_snapshot(VEHICLE_COMM_STALE_SECONDS)
            firmware_status = firmware_versions.communication_snapshot(VEHICLE_COMM_STALE_SECONDS)
            ping_status = vehicle_link_ping.snapshot()
            sender_status = packet_sender.to_dict()
            tx_age_seconds = seconds_since(sender_status.get("last_sent_at"))
            tx_recent = (
                bool(sender_status.get("enabled"))
                and sender_status.get("last_sent_at") is not None
                and tx_age_seconds is not None
                and tx_age_seconds <= VEHICLE_COMM_STALE_SECONDS
                and not sender_status.get("last_error")
            )
            ping_connected = bool(ping_status.get("connected"))
            connected = bool(event_status["connected"] or firmware_status["connected"] or ping_connected)
            if connected:
                if event_status["connected"]:
                    source = "event"
                elif firmware_status["connected"]:
                    source = "firmware"
                else:
                    source = "ping"
                status = "connected"
            elif tx_recent:
                source = "tx"
                status = "transmitting"
            else:
                source = "none"
                status = "disconnected"
            return {
                "status": status,
                "connected": connected,
                "source": source,
                "last_event": event_status["last_event"],
                "last_event_at": event_status["last_event_at"],
                "last_event_age_seconds": event_status["last_event_age_seconds"],
                "event_count": event_status["event_count"],
                "last_version_target": firmware_status["last_success_target"],
                "last_version_at": firmware_status["last_success_at"],
                "last_version_age_seconds": firmware_status["last_success_age_seconds"],
                "ping_connected": ping_connected,
                "ping_target": ping_status.get("target"),
                "ping_checked_at": ping_status.get("checked_at"),
                "ping_latency_ms": ping_status.get("latency_ms"),
                "ping_error": ping_status.get("error"),
                "ping_enabled": ping_status.get("enabled"),
                "tx_enabled": bool(sender_status.get("enabled")),
                "tx_target": f"{sender_status.get('host')}:{sender_status.get('port')}",
                "tx_recent": tx_recent,
                "tx_last_sent_at": sender_status.get("last_sent_at"),
                "tx_last_error": sender_status.get("last_error"),
                "stale_seconds": VEHICLE_COMM_STALE_SECONDS,
            }

        def store_feature_package_path(record: dict) -> Path | None:
            raw_path = record.get("package", {}).get("path")
            if not raw_path:
                return None
            path = Path(str(raw_path))
            return path if path.is_absolute() else BASE_DIR / path

        store_latest_version_cache: dict[str, dict[str, Any]] = {}
        store_latest_version_cache_lock = threading.Lock()
        store_latest_version_cache_seconds = float(os.getenv("VEHICLE_STORE_VERSION_CACHE_SECONDS", "60"))

        def github_latest_action_version(item: dict, action: dict) -> dict:
            repo = str(action.get("release_repo") or item.get("release_repo") or "")
            if not repo:
                latest_version = firmware_version(item.get("latest_version")) or str(item.get("latest_version") or "")
                return {
                    "latest_version": latest_version,
                    "latest_version_source": "catalog",
                    "latest_version_checked_at": None,
                    "latest_version_error": None,
                }

            cache_key = f"{item['id']}:{repo}:{action.get('release_patch_filter', '')}"
            now = time.time()
            with store_latest_version_cache_lock:
                cached = store_latest_version_cache.get(cache_key)
            if cached and now - float(cached.get("cached_at", 0)) < store_latest_version_cache_seconds:
                return dict(cached["payload"])

            try:
                release = ota_manager.fetch_latest_release(item, action)
                release_tag = str(release.get("tag_name") or "")
                latest_version = firmware_version(release_tag) or release_tag
                payload = {
                    "latest_version": latest_version,
                    "latest_versions": {FEATURE_VERSION_TARGET: latest_version} if latest_version else {},
                    "latest_version_source": "github",
                    "latest_version_checked_at": utc_now(),
                    "latest_version_error": None,
                    "release_tag": release_tag,
                    "release_url": release.get("html_url"),
                    "repo": repo,
                }
            except Exception as exc:
                fallback_version = firmware_version(item.get("latest_version")) or str(item.get("latest_version") or "")
                payload = {
                    "latest_version": fallback_version,
                    "latest_versions": {FEATURE_VERSION_TARGET: fallback_version} if fallback_version else {},
                    "latest_version_source": "catalog",
                    "latest_version_checked_at": utc_now(),
                    "latest_version_error": f"{type(exc).__name__}: {exc}",
                    "repo": repo,
                }

            with store_latest_version_cache_lock:
                store_latest_version_cache[cache_key] = {"cached_at": now, "payload": dict(payload)}
            return payload

        def github_latest_feature_version(item: dict) -> dict:
            actions: list[dict] = []
            if item.get("package_required", True):
                try:
                    actions = [ota_manager.python_package_action(item)]
                except ValueError:
                    actions = []
            else:
                actions = [
                    action
                    for action in ota_manager.actions_for(item)
                    if action.get("release_repo")
                ]

            if not actions:
                for candidate in ota_manager.actions_for(item):
                    if candidate.get("release_repo"):
                        actions = [candidate]
                        break

            if not actions:
                latest_version = firmware_version(item.get("latest_version")) or str(item.get("latest_version") or "")
                return {
                    "latest_version": latest_version,
                    "latest_versions": {FEATURE_VERSION_TARGET: latest_version} if latest_version else {},
                    "latest_version_source": "catalog",
                    "latest_version_checked_at": None,
                    "latest_version_error": None,
                }

            if len(actions) == 1:
                return github_latest_action_version(item, actions[0])

            latest_versions: dict[str, str] = {}
            repos: dict[str, str] = {}
            release_tags: dict[str, str] = {}
            errors: list[str] = []
            for action in actions:
                target_name = ota_target_display_name(action.get("target", FEATURE_VERSION_TARGET))
                payload = github_latest_action_version(item, action)
                latest_version = str(payload.get("latest_version") or "")
                if latest_version:
                    latest_versions[target_name] = latest_version
                if payload.get("repo"):
                    repos[target_name] = str(payload["repo"])
                if payload.get("release_tag"):
                    release_tags[target_name] = str(payload["release_tag"])
                if payload.get("latest_version_error"):
                    errors.append(f"{target_name}: {payload['latest_version_error']}")

            fallback_version = firmware_version(item.get("latest_version")) or str(item.get("latest_version") or "")
            unique_versions = list(dict.fromkeys(version for version in latest_versions.values() if version))
            latest_version = (
                unique_versions[0]
                if len(unique_versions) == 1
                else latest_versions.get("ZCU") or (unique_versions[0] if unique_versions else fallback_version)
            )
            return {
                "latest_version": latest_version,
                "latest_versions": latest_versions or ({FEATURE_VERSION_TARGET: fallback_version} if fallback_version else {}),
                "latest_version_source": "github" if latest_versions else "catalog",
                "latest_version_checked_at": utc_now(),
                "latest_version_error": "; ".join(errors) if errors else None,
                "release_tags": release_tags,
                "repos": repos,
            }

        def store_item_version_payload(item: dict, record: dict, installed: bool) -> dict:
            package_required = bool(item.get("package_required", True))
            package_downloaded = bool(record.get("package", {}).get("downloaded"))
            package_path = store_feature_package_path(record)
            package_version = python_module_version(package_path)
            firmware_versions = firmware_target_versions(record)
            firmware_version_values = [version for version in firmware_versions.values() if version]
            firmware_display_version = (
                firmware_version_values[0]
                if firmware_version_values and len(set(firmware_version_values)) == 1
                else None
            )
            recorded_version = (
                firmware_display_version
                if not package_required and firmware_display_version
                else firmware_version(record.get("version"))
                or firmware_display_version
                or str(record.get("version") or "")
            )
            latest_payload = github_latest_feature_version(item)
            latest_version = str(latest_payload.get("latest_version") or "")

            downloaded_version = None
            if package_downloaded:
                downloaded_version = package_version or recorded_version or None
            installed_version = downloaded_version if installed else None
            display_version = installed_version or downloaded_version or latest_version or "1.0.0"

            versions = {}
            if downloaded_version:
                versions[FEATURE_VERSION_TARGET] = downloaded_version
            if record.get("zcu_ota", {}).get("applied"):
                versions.update(firmware_versions)

            return {
                **latest_payload,
                "version": display_version,
                "versions": versions,
                "latest_versions": latest_payload.get("latest_versions", {}),
                "installed_version": installed_version,
                "downloaded_version": downloaded_version,
            }

        def store_items_payload() -> list[dict]:
            records = feature_state_store.load()["items"]
            items = []
            for item in STORE_CATALOG:
                record = records[item["id"]]
                installed = feature_state_store._is_record_installed(item, record)
                version_payload = store_item_version_payload(item, record, installed)
                items.append(
                    {
                        **item,
                        **version_payload,
                        "purchased": record["purchased"],
                        "enabled": record["enabled"],
                        "downloaded": record["package"]["downloaded"],
                        "applied": installed,
                        "runtime_error": record["zcu_ota"].get("error") or record["package"]["error"],
                        "ota_progress": ota_manager.progress_for(item["id"]),
                        "zcu_ota": record["zcu_ota"],
                        "installed": installed,
                        "update_available": False,
                    }
                )
            return items

        def dashboard_payload() -> dict:
            payload = vehicle_status.dashboard_payload(feature_state_store.purchased_ids())
            active = {
                "AEB": feature_state_store.is_feature_enabled("AEB"),
                "LKAS": feature_state_store.is_feature_enabled("LKAS"),
                "FVSA": feature_state_store.is_feature_enabled("FVSA"),
            }
            control = vehicle_control.snapshot()
            payload["installed"] = feature_state_store.downloaded_ids()
            payload["active"] = active
            payload["versions"] = feature_state_store.versions()
            aeb_alert = vehicle_status.aeb_trigger_snapshot()
            payload["aeb_triggered"] = bool(aeb_alert["active"]) or bool(control.get("aeb", {}).get("value", {}).get("alarm"))
            payload["aeb_triggered_at"] = aeb_alert["triggered_at"]
            payload["aeb_trigger_count"] = aeb_alert["count"]
            board_versions = firmware_versions.snapshot()
            aeb_zcu = feature_state_store.feature_record("AEB")["zcu_ota"]
            if aeb_zcu.get("applied") and aeb_zcu.get("version"):
                board_versions["ZCU"] = firmware_version(aeb_zcu["version"]) or board_versions.get("ZCU", "-")
            payload["firmware_versions"] = board_versions
            payload["pending_ota"] = feature_state_store.pending_ota()
            payload["available_themes"] = (
                ["luffy"] if feature_state_store.feature_record("LUFFY_THEME")["purchased"] else []
            )
            internet = internet_status.snapshot()
            vehicle_link = vehicle_computer_connection_payload()
            payload["network"] = {
                "status": internet["status"],
                "server_url": f"http://{host}:{port}",
                "internet": internet,
                "vehicle_computer": vehicle_link,
                "pings": {
                    "front_zcu": front_zcu_ping.snapshot(),
                },
            }
            return payload

        app = FastAPI(title="vehicle-computer", version="0.1.0")
        if FRONTEND_DIR.exists():
            app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

        store_ota_lock = threading.Lock()
        store_ota_active: set[str] = set()

        def run_store_feature_ota(feature_id: str) -> None:
            with store_ota_lock:
                if feature_id in store_ota_active:
                    return
                store_ota_active.add(feature_id)
            try:
                result = feature_state_store.apply_feature_ota(feature_id)
                logger.info("store background OTA finished for %s: %s", feature_id, result.get("all_success"))
                if feature_id in ("AEB", "LKAS", "FVSA"):
                    vehicle_control.refresh_feature_runtime(feature_id)
            except Exception as exc:
                logger.warning("store background OTA failed for %s: %s", feature_id, exc)
            finally:
                with store_ota_lock:
                    store_ota_active.discard(feature_id)

        def no_store_headers() -> dict[str, str]:
            return {
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "Pragma": "no-cache",
                "Expires": "0",
            }

        @app.get("/", include_in_schema=False)
        async def index() -> FileResponse:
            dashboard_index = FRONTEND_DIR / "index.html"
            if not dashboard_index.exists():
                raise HTTPException(status_code=404, detail="dashboard index.html not found")
            return FileResponse(dashboard_index, headers=no_store_headers())

        @app.get("/store", include_in_schema=False)
        @app.get("/store.html", include_in_schema=False)
        async def store_page() -> FileResponse:
            dashboard_store = FRONTEND_DIR / "store.html"
            if not dashboard_store.exists():
                raise HTTPException(status_code=404, detail="dashboard store.html not found")
            return FileResponse(dashboard_store, headers=no_store_headers())

        @app.get("/themes/{theme_id}.css", include_in_schema=False)
        async def theme_css(theme_id: str) -> Response:
            if theme_id != "luffy":
                raise HTTPException(status_code=404, detail="theme css not found")
            css = """
[data-theme="luffy"] .luffy-deco{display:block}
[data-theme="luffy"] .storebtn,
[data-theme="luffy"] .ota-banner-btn,
[data-theme="luffy"] .bpri,
[data-theme="luffy"] .lactbtn{background:linear-gradient(135deg,#c87de8,#ff8bc8);color:#4a2060}
"""
            return Response(content=css.strip(), media_type="text/css")

        @app.get("/api/health")
        async def health() -> dict:
            heartbeat()
            payload = supervisor.health_snapshot()
            payload["vehicle_status"] = vehicle_status.to_dict()
            return payload

        @app.get("/api/children")
        async def children() -> dict:
            heartbeat()
            return {"children": supervisor.children_snapshot()}

        @app.get("/api/vehicle/status")
        async def get_vehicle_status() -> dict:
            heartbeat()
            return vehicle_status.to_dict()

        @app.get("/api/firmware/versions")
        async def get_firmware_versions() -> dict:
            heartbeat()
            return firmware_versions.status()

        @app.get("/api/status")
        async def dashboard_status() -> dict:
            heartbeat()
            return dashboard_payload()

        @app.get("/api/network/status")
        async def network_status() -> dict:
            heartbeat()
            return {
                "internet": internet_status.snapshot(),
                "vehicle_computer": vehicle_computer_connection_payload(),
                "pings": {
                    "front_zcu": front_zcu_ping.snapshot(),
                },
            }

        @app.get("/api/store/items")
        async def store_items() -> dict:
            heartbeat()
            return {"items": await run_in_threadpool(store_items_payload)}

        @app.get("/api/store/purchases")
        async def store_purchases() -> dict:
            heartbeat()
            return feature_state_store.load()

        @app.get("/api/store/state")
        async def store_state() -> dict:
            heartbeat()
            return feature_state_store.load()

        @app.get("/api/ota/status")
        async def ota_status() -> dict:
            heartbeat()
            return {
                "state": feature_state_store.load(),
                "ota": ota_manager.status(),
            }

        @app.get("/api/ota/progress/{feature_id}")
        async def ota_progress(feature_id: str) -> dict:
            heartbeat()
            if feature_id != MANUAL_FLASHER_PROGRESS_ID and feature_id not in STORE_ITEM_IDS:
                raise HTTPException(status_code=404, detail=f"unknown store item: {feature_id}")
            return ota_manager.progress_for(feature_id)

        @app.get("/api/flasher/boards")
        async def flasher_boards() -> dict:
            heartbeat()
            boards = []
            for board in FLASHER_BOARD_CONFIGS.values():
                payload = {
                    "id": board["id"],
                    "name": board["name"],
                    "transport": board["transport"],
                    "implemented": board["implemented"],
                    "note": board.get("note"),
                }
                if "ecu_ip" in board:
                    payload.update(
                        {
                            "ecu_ip": board["ecu_ip"],
                            "doip_port": board["doip_port"],
                            "ecu_address": board["ecu_address"],
                        }
                    )
                boards.append(payload)
            return {"boards": boards}

        @app.post("/api/flasher/flash")
        async def flash_binary(body: ManualFlashRequest) -> dict:
            heartbeat()
            board = FLASHER_BOARD_CONFIGS[body.board_id]
            if not board.get("implemented"):
                ota_manager.update_progress(
                    MANUAL_FLASHER_PROGRESS_ID,
                    action_id="manual_flash",
                    action_type=f"{board['transport']}_flash",
                    target=body.board_id,
                    phase="not_implemented",
                    status="not_implemented",
                    percent=None,
                    message=f"{board['name']} flashing will use CAN later",
                    active=False,
                )
                raise HTTPException(
                    status_code=501,
                    detail=f"{board['name']} flashing is not implemented",
                )
            filename = safe_filename(body.filename)
            if not filename.lower().endswith(".bin"):
                raise HTTPException(status_code=400, detail="binary file must have .bin extension")
            try:
                firmware = base64.b64decode(body.content_base64, validate=True)
            except (binascii.Error, ValueError) as exc:
                raise HTTPException(status_code=400, detail="invalid base64 payload") from exc
            if not firmware:
                raise HTTPException(status_code=400, detail="binary file is empty")

            manual_dir = FIRMWARE_DIR / "manual"
            manual_dir.mkdir(parents=True, exist_ok=True)
            saved_path = manual_dir / f"{int(time.time())}_{filename}"
            saved_path.write_bytes(firmware)

            transport = str(board["transport"])
            if transport == "doip":
                action = {
                    "id": "manual_flash",
                    "type": "doip_uds_flash",
                    "target": body.board_id,
                    "ecu_ip": board["ecu_ip"],
                    "doip_port": board["doip_port"],
                    "tester_address": board["tester_address"],
                    "ecu_address": board["ecu_address"],
                    "bank_start": board["bank_start"],
                    "timeout_seconds": board["timeout_seconds"],
                    "p2_timeout_seconds": board["p2_timeout_seconds"],
                    "p2_star_timeout_seconds": board["p2_star_timeout_seconds"],
                    "use_server_timing": board["use_server_timing"],
                }
                flash_func = ota_manager.flash_bin_via_doip
                flash_message = f"{board['name']} flashing"
            elif transport == "doip_sensor_can_ota":
                action = {
                    "id": "manual_flash",
                    "type": "doip_sensor_can_ota",
                    "target": body.board_id,
                    "ecu_ip": board["ecu_ip"],
                    "doip_port": board["doip_port"],
                    "tester_address": board["tester_address"],
                    "ecu_address": board["ecu_address"],
                    "app_addr": board["app_addr"],
                    "block_size": board["block_size"],
                    "timeout_seconds": board["timeout_seconds"],
                    "block_delay_seconds": board["block_delay_seconds"],
                    "progress_update_interval_blocks": 1,
                    "activate_after_transfer": board["activate_after_transfer"],
                }
                flash_func = ota_manager.flash_sensor_can_ota_via_doip
                flash_message = f"{board['name']} CAN OTA via ZCU"
            else:
                raise HTTPException(status_code=501, detail=f"unsupported transport: {transport}")

            action_type = str(action["type"])
            ota_manager.update_progress(
                MANUAL_FLASHER_PROGRESS_ID,
                action_id="manual_flash",
                action_type=action_type,
                target=body.board_id,
                phase="upload",
                status="uploaded",
                percent=5,
                message=f"{filename} uploaded for {board['name']}",
                active=True,
                bytes_downloaded=len(firmware),
                total_bytes=len(firmware),
            )
            try:
                success = await run_in_threadpool(
                    flash_func,
                    saved_path,
                    MANUAL_FLASHER_PROGRESS_ID,
                    action,
                    progress={
                        "feature_id": MANUAL_FLASHER_PROGRESS_ID,
                        "action_id": "manual_flash",
                        "action_type": action_type,
                        "target": body.board_id,
                        "message": flash_message,
                        "percent_start": 5,
                        "percent_end": 100,
                    },
                )
            except (OSError, RuntimeError, ValueError) as exc:
                raise HTTPException(status_code=502, detail=str(exc)) from exc
            ota_manager.update_progress(
                MANUAL_FLASHER_PROGRESS_ID,
                action_id="manual_flash",
                action_type=action_type,
                target=body.board_id,
                phase="complete" if success else "error",
                status="complete" if success else "failed",
                percent=100 if success else None,
                message=f"{board['name']} flash complete" if success else f"{board['name']} flash failed",
                active=False,
                bytes_downloaded=len(firmware),
                total_bytes=len(firmware),
            )
            return {
                "ok": success,
                "board": {"id": board["id"], "name": board["name"]},
                "filename": filename,
                "path": str(saved_path.relative_to(BASE_DIR)),
                "bytes": len(firmware),
                "message": "flash complete" if success else "flash failed",
            }

        @app.post("/api/demo/reset-settings")
        async def demo_reset_settings() -> dict:
            heartbeat()
            try:
                result = await run_in_threadpool(
                    feature_state_store.reset,
                    clear_downloads=RESET_SETTINGS_CLEAR_DOWNLOADS,
                )
                vehicle_control.reload_feature_state()
                vehicle_control.poll_once()
            except (OSError, RuntimeError, ValueError) as exc:
                raise HTTPException(status_code=500, detail=f"설정 파일 초기화 실패: {exc}") from exc
            return {
                **result,
                "message": "설정 파일 초기화 완료",
            }

        @app.post("/api/store/purchase")
        async def store_purchase(body: StorePurchaseRequest, background_tasks: BackgroundTasks) -> dict:
            heartbeat()
            auto_flash = body.feature_id == "AEB"
            try:
                result = await run_in_threadpool(
                    feature_state_store.purchase,
                    body.feature_id,
                    run_flash=not auto_flash,
                )
            except (OSError, RuntimeError, ValueError) as exc:
                raise HTTPException(status_code=404, detail=str(exc)) from exc
            refresh_feature_runtime(body.feature_id)
            if body.feature_id in ("AEB", "LKAS", "FVSA"):
                result["item"] = feature_state_store.feature_record(body.feature_id)
                result["downloaded"] = result["item"]["package"]["downloaded"]
            if auto_flash and result.get("downloaded"):
                background_tasks.add_task(run_store_feature_ota, body.feature_id)
                result["background_ota_started"] = True

            name = next(
                item["name"] for item in STORE_CATALOG if item["id"] == body.feature_id
            )
            if result["downloaded"]:
                action = "다운로드 완료"
            else:
                action = "구매 완료"
            message = f"{name} already purchased" if result["already_purchased"] else f"{name} {action}"
            return {**result, "message": message}

        @app.post("/api/store/download")
        async def store_download(body: StorePurchaseRequest, background_tasks: BackgroundTasks) -> dict:
            heartbeat()
            auto_flash = body.feature_id == "AEB"
            try:
                result = await run_in_threadpool(
                    feature_state_store.download_feature,
                    body.feature_id,
                    run_flash=not auto_flash,
                )
            except (OSError, RuntimeError, ValueError) as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            refresh_feature_runtime(body.feature_id)
            if body.feature_id in ("AEB", "LKAS", "FVSA"):
                result["item"] = feature_state_store.feature_record(body.feature_id)
            if auto_flash:
                background_tasks.add_task(run_store_feature_ota, body.feature_id)
                result["background_ota_started"] = True
            return {**result, "message": "다운로드 완료"}

        @app.get("/api/weather")
        async def dashboard_weather() -> dict:
            heartbeat()
            return await run_in_threadpool(fetch_weather)

        @app.get("/api/theme-available/{theme_id}")
        async def theme_available(theme_id: str) -> dict:
            heartbeat()
            available = theme_id != "luffy" or feature_state_store.feature_record("LUFFY_THEME")["purchased"]
            return {"theme": theme_id, "available": available}

        @app.post("/api/theme")
        async def set_theme(body: ThemeRequest) -> dict:
            heartbeat()
            if body.theme == "luffy" and not feature_state_store.feature_record("LUFFY_THEME")["purchased"]:
                raise HTTPException(status_code=400, detail="luffy theme is not purchased")
            return {"success": True, "vehicle_status": vehicle_status.set_theme(body.theme)}

        @app.post("/api/ota/check")
        async def check_ota_now() -> dict:
            heartbeat()
            try:
                vehicle_control.poll_once()
                pending = await run_in_threadpool(feature_state_store.check_pending_zcu_updates)
            except Exception as exc:
                raise HTTPException(status_code=502, detail=f"최신버전 확인 실패: {exc}") from exc
            count = len(pending.get("to_install", [])) + len(pending.get("to_update", []))
            return {
                "success": True,
                "count": count,
                "pending_ota": pending,
                "message": f"업데이트 {count}건이 있습니다." if count else "최신버전입니다.",
            }

        @app.post("/api/ota/decision")
        async def ota_decision(body: OtaDecisionRequest) -> dict:
            heartbeat()
            if not body.do_update:
                pending_now = feature_state_store.pending_ota()
                pending_items = list(pending_now.get("to_install", [])) + list(pending_now.get("to_update", []))
                def pending_target_text(item: dict) -> str:
                    targets = item.get("targets") or []
                    target_list = ",".join(str(target) for target in targets) if isinstance(targets, list) else str(targets)
                    return str(item.get("ecu_target") or item.get("target") or target_list)
                has_mandatory_aeb_zcu = any(
                    (item.get("feature_id") or item.get("id")) == "AEB"
                    and "zcu" in pending_target_text(item).lower()
                    and bool(item.get("install_required"))
                    for item in pending_items
                    if isinstance(item, dict)
                )
                if has_mandatory_aeb_zcu:
                    raise HTTPException(status_code=409, detail="AEB ZCU OTA cannot be skipped")
                pending = feature_state_store.clear_pending_ota()
                return {"success": True, "accepted": False, "pending_ota": pending}

            vehicle = vehicle_status.to_dict()
            if vehicle.get("gear") != GEAR_P:
                return {
                    "success": True,
                    "accepted": False,
                    "waiting_for_park": True,
                    "message": "기어를 P단으로 바꾸면 업데이트가 시작됩니다.",
                    "pending_ota": feature_state_store.pending_ota(),
                }

            pending_before = feature_state_store.pending_ota()
            result = await run_in_threadpool(feature_state_store.apply_pending_ota)
            reset_trigger = await run_in_threadpool(send_ota_reset_triggers, pending_before, result)
            result["reset_trigger"] = reset_trigger
            if not reset_trigger.get("success", True):
                result["all_success"] = False
            refreshed = False
            for ota_result in result.get("results", []):
                if not isinstance(ota_result, dict) or not ota_result.get("success"):
                    continue
                feature_id = str(ota_result.get("feature_id") or "")
                if feature_id in ("AEB", "LKAS", "FVSA"):
                    refresh_feature_runtime(feature_id)
                    refreshed = True
            if not refreshed:
                vehicle_control.poll_once()
            return {
                "success": True,
                "accepted": True,
                "result": result,
                "pending_ota": feature_state_store.pending_ota(),
            }

        @app.post("/api/demo/reveal-luffy")
        async def demo_reveal_luffy() -> dict:
            heartbeat()
            return {
                "success": True,
                "items": [
                    {
                        "id": "LUFFY_THEME",
                        "icon": "THEME",
                        "name": "테마(루피)",
                        "desc": "루피 테마가 스토어에 표시됩니다.",
                    }
                ],
            }

        @app.post("/api/demo/fvsa-buzzer")
        async def demo_fvsa_buzzer() -> dict:
            heartbeat()
            result = vehicle_control.trigger_fvsa_buzzer()
            if not result.get("ok"):
                raise HTTPException(status_code=400, detail=result.get("message", "FVSA buzzer failed"))
            return {"success": True, **result}

        @app.websocket("/ws")
        async def websocket_status(websocket: WebSocket) -> None:
            await websocket.accept()
            try:
                while not stop_event.is_set():
                    heartbeat()
                    await websocket.send_json(dashboard_payload())
                    await asyncio.sleep(1.0)
            except WebSocketDisconnect:
                return

        @app.websocket("/ws/debug-terminal")
        async def debug_terminal(websocket: WebSocket) -> None:
            await websocket.accept()
            heartbeat()
            out_queue: queue.Queue[tuple[str, str]] = queue.Queue()
            connected = True

            async def safe_send(payload: dict[str, Any]) -> bool:
                nonlocal connected
                if not connected:
                    return False
                try:
                    await websocket.send_json(payload)
                    return True
                except (RuntimeError, WebSocketDisconnect):
                    connected = False
                    return False

            def read_stream(stream: Any, stream_name: str) -> None:
                while True:
                    try:
                        chunk = stream.readline()
                    except OSError:
                        return
                    if not chunk:
                        return
                    out_queue.put((stream_name, chunk))

            debug_service = os.getenv("VEHICLE_DEBUG_JOURNAL_SERVICE", "vehicle-computer-app.service")
            if os.name == "nt":
                shell_cmd = [
                    os.getenv("VEHICLE_DEBUG_SHELL", "powershell.exe"),
                    "-NoLogo",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                ]
                debug_label = "PowerShell"
            else:
                shell_cmd = [
                    os.getenv("VEHICLE_DEBUG_JOURNALCTL", "journalctl"),
                    "-u",
                    debug_service,
                    "-f",
                    "-n",
                    os.getenv("VEHICLE_DEBUG_JOURNAL_LINES", "80"),
                ]
                debug_label = " ".join(shell_cmd)
            creationflags = subprocess.CREATE_NO_WINDOW if hasattr(subprocess, "CREATE_NO_WINDOW") else 0
            try:
                process = subprocess.Popen(
                    shell_cmd,
                    cwd=str(BASE_DIR),
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    bufsize=1,
                    creationflags=creationflags,
                )
            except OSError as exc:
                await safe_send({"type": "output", "stream": "stderr", "data": f"{exc}\n"})
                await websocket.close()
                return

            for stream_name, stream in (("stdout", process.stdout), ("stderr", process.stderr)):
                if stream is None:
                    continue
                threading.Thread(
                    name=f"debug-terminal-{stream_name}",
                    target=read_stream,
                    args=(stream, stream_name),
                    daemon=True,
                ).start()

            await safe_send(
                {
                    "type": "output",
                    "stream": "stdout",
                    "data": f"Debug terminal ready: {debug_label}\n",
                }
            )

            async def pump_output() -> None:
                nonlocal connected
                while connected and (process.poll() is None or not out_queue.empty()):
                    heartbeat()
                    batches: dict[str, list[str]] = {}
                    batch_chars = 0
                    while connected and not out_queue.empty():
                        stream_name, chunk = out_queue.get_nowait()
                        batches.setdefault(stream_name, []).append(chunk)
                        batch_chars += len(chunk)
                        if batch_chars >= 16000:
                            break
                    for stream_name, chunks in batches.items():
                        await safe_send({"type": "output", "stream": stream_name, "data": "".join(chunks)})
                    await asyncio.sleep(0.01)
                await safe_send({"type": "exit", "code": process.returncode})

            async def receive_input() -> None:
                nonlocal connected
                try:
                    while connected and process.poll() is None:
                        message = await websocket.receive_json()
                        if message.get("type") != "input":
                            continue
                        data = str(message.get("data", ""))
                        if process.stdin is None:
                            continue
                        process.stdin.write(data)
                        process.stdin.flush()
                except WebSocketDisconnect:
                    connected = False

            pump_task = asyncio.create_task(pump_output())
            input_task = asyncio.create_task(receive_input())
            try:
                done, pending = await asyncio.wait(
                    {pump_task, input_task},
                    return_when=asyncio.FIRST_COMPLETED,
                )
                for task in done:
                    task.result()
                for task in pending:
                    task.cancel()
                if pending:
                    await asyncio.gather(*pending, return_exceptions=True)
            except (WebSocketDisconnect, asyncio.CancelledError):
                return
            finally:
                connected = False
                if process.poll() is None:
                    process.terminate()
                heartbeat()

        @app.get("/api/vehicle/control")
        async def get_vehicle_control() -> dict:
            heartbeat()
            payload = vehicle_control.snapshot()
            payload["ethernet_tx"] = packet_sender.to_dict()
            return payload

        @app.post("/api/vehicle/control/lkas")
        async def set_lkas(body: FeatureToggleRequest) -> dict:
            heartbeat()
            return {"ok": True, "control": vehicle_control.set_feature_enabled("LKAS", body.enabled)}

        @app.post("/api/vehicle/control/fvsa")
        async def set_fvsa(body: FeatureToggleRequest) -> dict:
            heartbeat()
            return {"ok": True, "control": vehicle_control.set_feature_enabled("FVSA", body.enabled)}

        @app.post("/api/vehicle/control/aeb")
        async def set_aeb(body: FeatureToggleRequest) -> dict:
            heartbeat()
            return {"ok": True, "control": vehicle_control.set_feature_enabled("AEB", body.enabled)}

        @app.post("/api/aeb")
        async def aeb_alert() -> dict:
            heartbeat()
            vehicle_status.mark_aeb_trigger()
            return {"ok": True, "control": vehicle_control.trigger_aeb()}

        @app.post("/api/features/toggle")
        async def toggle_feature(body: FeatureToggleByIdRequest) -> dict:
            heartbeat()
            feature_id = body.feature_id
            if feature_id not in FEATURE_IDS:
                raise HTTPException(status_code=400, detail="toggle is only available for AEB/LKAS/FVSA")

            enabled = not feature_state_store.is_feature_enabled(feature_id)
            control = vehicle_control.set_feature_enabled(feature_id, enabled)
            effective = feature_state_store.is_feature_enabled(feature_id)
            feature_state = control[feature_id.lower()]
            if enabled and (not feature_state["downloaded"] or not feature_state["applied"]):
                message = f"{feature_id} 파일이 다운로드/적용되지 않아 기본 조종으로 유지됩니다."
            else:
                message = f"{feature_id} {'enabled' if effective else 'disabled'}"
            return {"success": True, "message": message, "control": control}

        @app.post("/api/shutdown")
        async def shutdown() -> dict:
            logger.debug("shutdown requested by API")
            supervisor.request_stop()
            heartbeat()
            return {"status": "stopping"}

        @app.post("/api/ethernet/send")
        async def ethernet_send(body: EthernetSendRequest) -> dict:
            heartbeat()
            try:
                payload = parse_payload(body.payload, body.payload_format)
                logger.debug(
                    "ethernet send requested: protocol=%s host=%s port=%s payload_bytes=%s expect_response=%s",
                    body.protocol,
                    body.host,
                    body.port,
                    len(payload),
                    body.expect_response,
                )
                response = await run_in_threadpool(
                    send_ethernet_message,
                    body.protocol,
                    body.host,
                    body.port,
                    payload,
                    timeout_seconds=body.timeout_seconds,
                    expect_response=body.expect_response,
                    receive_bytes=body.receive_bytes,
                    local_host=body.local_host,
                    local_port=body.local_port,
                )
            except (OSError, ValueError) as exc:
                logger.error("ethernet send failed: %s", exc)
                raise HTTPException(status_code=400, detail=str(exc)) from exc

            return {
                "ok": True,
                "payload_bytes": len(payload),
                "result": asdict(response),
            }

        config = uvicorn.Config(
            app=app,
            host=host,
            port=port,
            log_level="info",
            access_log=False,
        )
        server = uvicorn.Server(config)

        def watch_stop() -> None:
            while not stop_event.is_set():
                heartbeat()
                stop_event.wait(0.5)
            server.should_exit = True

        watcher = threading.Thread(
            name="api-server-stop-watcher",
            target=watch_stop,
            daemon=True,
        )
        watcher.start()

        logger.debug("api server listening: http://localhost:%s", port)
        server.run()

    return run


def build_supervisor() -> Supervisor:
    supervisor = Supervisor()
    vehicle_status = VehicleStatus()
    firmware_versions = FirmwareVersionStore(network_lock=ZCU_NETWORK_LOCK)
    packet_sender = VehiclePacketSender()
    internet_status = InternetConnectivityStatus(
        host=INTERNET_CHECK_HOST,
        port=INTERNET_CHECK_PORT,
        timeout_seconds=INTERNET_CHECK_TIMEOUT_SECONDS,
    )
    vehicle_link_ping = PingReachabilityStatus(
        host=VEHICLE_LINK_PING_HOST,
        timeout_seconds=VEHICLE_LINK_PING_TIMEOUT_SECONDS,
        enabled=VEHICLE_LINK_PING_ENABLED,
        network_lock=ZCU_NETWORK_LOCK,
    )
    front_zcu_ping = PingReachabilityStatus(
        host=FRONT_ZCU_PING_HOST,
        timeout_seconds=VEHICLE_LINK_PING_TIMEOUT_SECONDS,
        enabled=VEHICLE_LINK_PING_ENABLED,
        network_lock=ZCU_NETWORK_LOCK,
    )

    ota_manager = OtaManager(
        base_dir=BASE_DIR,
        downloaded_features_dir=DOWNLOADED_FEATURES_DIR,
        firmware_dir=FIRMWARE_DIR,
        timeout_seconds=OTA_DOWNLOAD_TIMEOUT_SECONDS,
    )
    feature_state_store = FeatureStateStore(
        FEATURE_STATE_FILE,
        ota_manager=ota_manager,
        legacy_path=LEGACY_PURCHASES_FILE,
    )
    vehicle_control = VehicleControl(
        on_gear_change=vehicle_status.apply_gear,
        sensor_provider=vehicle_status.sensor_snapshot,
        packet_sender=packet_sender.send,
        feature_state_store=feature_state_store,
        features_dir=DOWNLOADED_FEATURES_DIR,
        someip_service_id=DRIVE_SERVICE_ID,
        someip_method_id=DRIVE_METHOD_ID,
        someip_client_id=DRIVE_CLIENT_ID,
        aeb_service_id=AEB_SERVICE_ID,
        aeb_control_method_id=AEB_CONTROL_METHOD_ID,
        aeb_trigger_event_id=AEB_TRIGGER_EVENT_ID,
    )
    logger.debug("vehicle control TX config: %s", packet_sender.to_dict())

    supervisor.register(
        ChildService(
            "api-server",
            api_server_worker(
                supervisor,
                vehicle_status,
                firmware_versions,
                vehicle_control,
                packet_sender,
                internet_status,
                vehicle_link_ping,
                front_zcu_ping,
                feature_state_store,
                ota_manager,
                HOST,
                PORT,
            ),
        )
    )
    supervisor.register(ChildService("frontend-worker", placeholder_worker("frontend-worker", 2.0)))
    supervisor.register(
        ChildService(
            "internet-connectivity",
            internet_connectivity_worker(internet_status, INTERNET_CHECK_INTERVAL_SECONDS),
        )
    )
    supervisor.register(
        ChildService(
            "vehicle-link-ping",
            vehicle_link_ping_worker(vehicle_link_ping, VEHICLE_LINK_PING_INTERVAL_SECONDS),
        )
    )
    supervisor.register(
        ChildService(
            "front-zcu-ping",
            vehicle_link_ping_worker(front_zcu_ping, VEHICLE_LINK_PING_INTERVAL_SECONDS),
        )
    )
    supervisor.register(
        ChildService(
            "vehicle-control",
            vehicle_control.run,
        )
    )
    supervisor.register(
        ChildService(
            "vehicle-events",
            vehicle_event_worker(vehicle_status, vehicle_control, VEHICLE_EVENT_HOST, VEHICLE_EVENT_PORT),
        )
    )
    supervisor.register(
        ChildService(
            "ota-updater",
            ota_update_worker(feature_state_store, vehicle_control, OTA_POLL_INTERVAL_SECONDS),
        )
    )
    supervisor.register(
        ChildService(
            "firmware-versions",
            firmware_version_worker(firmware_versions, FIRMWARE_VERSION_POLL_INTERVAL_SECONDS),
        )
    )
    supervisor.register(ChildService("background-worker", placeholder_worker("background-worker", 5.0)))
    return supervisor


def main() -> int:
    configure_logging()
    try:
        ensure_api_port_available(HOST, PORT)
    except RuntimeError as exc:
        logger.error("%s", exc)
        return 1

    supervisor = build_supervisor()

    logger.debug("vehicle-computer threaded runtime starting")
    logger.debug("host=%s port=%s log_level=%s", HOST, PORT, LOG_LEVEL)
    logger.debug(
        "vehicle TX enabled=%s protocol=%s host=%s port=%s",
        VEHICLE_TX_ENABLED,
        VEHICLE_TX_PROTOCOL,
        VEHICLE_TX_HOST,
        VEHICLE_TX_PORT,
    )
    logger.debug(
        "vehicle event RX host=%s port=%s tof_event=%04x/%04x speed_event=%04x/%04x aeb_event=%04x/%04x",
        VEHICLE_EVENT_HOST,
        VEHICLE_EVENT_PORT,
        SENSOR_SERVICE_ID,
        TOF_VALUE_UPDATED_EVENT_ID,
        SENSOR_SERVICE_ID,
        SPEED_UPDATED_EVENT_ID,
        AEB_SERVICE_ID,
        AEB_TRIGGER_EVENT_ID,
    )
    supervisor.start_all()

    try:
        while not supervisor.stop_event.is_set():
            time.sleep(0.25)
    except KeyboardInterrupt:
        logger.debug("Ctrl+C received")
        supervisor.request_stop()
    finally:
        logger.debug("stopping children")
        supervisor.request_stop()
        supervisor.join_all()
        logger.debug("shutdown complete")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
