from __future__ import annotations

import json
import html
import os
import re
import shutil
import socket
import struct
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import zlib
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


SENSOR_GATEWAY_DOIP_VERSION = 0x02
SENSOR_GATEWAY_PT_ROUTING_ACT_REQ = 0x0005
SENSOR_GATEWAY_PT_ROUTING_ACT_RES = 0x0006
SENSOR_GATEWAY_PT_DIAG_MESSAGE = 0x8001
SENSOR_GATEWAY_PT_DIAG_ACK = 0x8002
SENSOR_CAN_OTA_MAX_FRAME_PAYLOAD = 64
SENSOR_CAN_OTA_TRANSFER_OVERHEAD = 2
SENSOR_CAN_OTA_MAX_DATA_BLOCK_SIZE = SENSOR_CAN_OTA_MAX_FRAME_PAYLOAD - SENSOR_CAN_OTA_TRANSFER_OVERHEAD


class SensorGatewayDoipError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class OtaPackageResult:
    feature_id: str
    action_id: str
    action_type: str
    target: str
    downloaded: bool
    applied: bool
    updated: bool
    path: Path | None
    version: str | None
    release_tag: str | None
    release_url: str | None
    asset_name: str | None
    downloaded_at: str | None
    checked_at: str | None
    error: str | None = None


@dataclass(frozen=True)
class SparseOtaSegment:
    index: int
    name: str
    offset: int
    size: int
    crc32: int | None
    path: Path
    data: bytes


@dataclass(frozen=True)
class SparseOtaPackage:
    manifest_path: Path
    package_dir: Path
    virtual_size: int
    virtual_crc32: int
    gap_fill: int
    total_payload_size: int
    segments: tuple[SparseOtaSegment, ...]

    @property
    def payload(self) -> bytes:
        return b"".join(segment.data for segment in self.segments)


class OtaAction:
    action_type = "base"

    def run(
        self,
        manager: "OtaManager",
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None,
        force: bool,
    ) -> OtaPackageResult:
        raise NotImplementedError


class GithubReleaseFileAction(OtaAction):
    action_type = "github_release_file"

    def run(
        self,
        manager: "OtaManager",
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None,
        force: bool,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        action_id = str(action.get("id", "python_package"))
        target_name = str(action.get("target", "rpi"))
        checked_at = utc_now()
        percent_start, percent_end = action_progress_span(action)
        download_file = str(action.get("download_file") or catalog_item["download_file"])
        target = manager.resolve_target_path(action, download_file)
        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="checking",
            status="checking",
            percent=percent_start,
            message="최신 릴리즈 확인 중",
            active=True,
        )

        release = manager.fetch_latest_release(catalog_item, action)
        release_tag = str(release.get("tag_name") or "")
        version = clean_version(release_tag) or str(catalog_item.get("latest_version", "1.0.0"))
        release_url = release.get("html_url")

        if not force and target.exists() and clean_version(current_version) == version:
            manager.update_progress(
                feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                phase="complete",
                status="current",
                percent=percent_end,
                message="최신 버전입니다",
                active=False,
            )
            return OtaPackageResult(
                feature_id=feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                downloaded=True,
                applied=False,
                updated=False,
                path=target,
                version=version,
                release_tag=release_tag,
                release_url=str(release_url) if release_url else None,
                asset_name=None,
                downloaded_at=None,
                checked_at=checked_at,
            )

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="download",
            status="downloading",
            percent=percent_start,
            message=f"{download_file} 다운로드 중",
            active=True,
        )
        module_bytes, asset_name = manager.download_release_asset(
            release,
            download_file,
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": self.action_type,
                "target": target_name,
                "message": f"{download_file} 다운로드 중",
                "percent_start": percent_start,
                "percent_end": percent_end,
            },
        )
        target.parent.mkdir(parents=True, exist_ok=True)
        temp_target = target.with_suffix(target.suffix + ".tmp")
        temp_target.write_bytes(module_bytes)
        temp_target.replace(target)
        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="complete",
            status="complete",
            percent=percent_end,
            message=f"{download_file} 다운로드 완료",
            active=False,
            bytes_downloaded=len(module_bytes),
            total_bytes=len(module_bytes),
        )
        return OtaPackageResult(
            feature_id=feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            downloaded=True,
            applied=False,
            updated=True,
            path=target,
            version=version,
            release_tag=release_tag,
            release_url=str(release_url) if release_url else None,
            asset_name=asset_name,
            downloaded_at=checked_at,
            checked_at=checked_at,
        )


class LocalFileAction(OtaAction):
    action_type = "local_file"

    def run(
        self,
        manager: "OtaManager",
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None,
        force: bool,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        action_id = str(action.get("id", "local_file"))
        target_name = str(action.get("target", "rpi"))
        checked_at = utc_now()
        percent_start, percent_end = action_progress_span(action)
        source = manager.resolve_source_path(action)
        download_file = str(action.get("download_file") or source.name)
        target = manager.resolve_target_path(action, download_file)
        version = str(action.get("version") or catalog_item.get("latest_version", "1.0.0"))

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="download",
            status="copying",
            percent=percent_start,
            message=f"{download_file} 설치 중",
            active=True,
        )
        if not source.exists():
            raise FileNotFoundError(f"local OTA source not found: {source}")

        if not force and target.exists() and clean_version(current_version) == clean_version(version):
            manager.update_progress(
                feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                phase="complete",
                status="current",
                percent=percent_end,
                message="최신 버전입니다",
                active=False,
            )
            return OtaPackageResult(
                feature_id=feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                downloaded=True,
                applied=False,
                updated=False,
                path=target,
                version=version,
                release_tag=None,
                release_url=None,
                asset_name=source.name,
                downloaded_at=None,
                checked_at=checked_at,
            )

        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="complete",
            status="complete",
            percent=percent_end,
            message=f"{download_file} 설치 완료",
            active=False,
            bytes_downloaded=target.stat().st_size,
            total_bytes=target.stat().st_size,
        )
        return OtaPackageResult(
            feature_id=feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            downloaded=True,
            applied=False,
            updated=True,
            path=target,
            version=version,
            release_tag=None,
            release_url=None,
            asset_name=source.name,
            downloaded_at=checked_at,
            checked_at=checked_at,
        )


class DoipUdsFlashAction(OtaAction):
    action_type = "doip_uds_flash"

    def run(
        self,
        manager: "OtaManager",
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None,
        force: bool,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        action_id = str(action.get("id", "zcu_flash"))
        target_name = str(action.get("target", "zcu"))
        checked_at = utc_now()
        percent_start, percent_end = action_progress_span(action)
        percent_flash_start = percent_start + int((percent_end - percent_start) * 0.4)

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="checking",
            status="checking",
            percent=percent_start,
            message="ZCU OTA 패키지 릴리즈 확인 중",
            active=True,
        )

        release = manager.fetch_latest_release(catalog_item, action)
        release_tag = str(release.get("tag_name") or "")
        version = clean_firmware_version(release_tag) or str(catalog_item.get("latest_version", "1.0.0"))
        release_url = release.get("html_url")
        release_asset = manager.release_ota_package_asset(release, action)
        download_file = str(release_asset["name"])
        package_path = manager.resolve_target_path(
            {**action, "target_dir": str(action.get("target_dir", "firmware"))},
            download_file,
        )

        if not force and package_path.exists() and clean_firmware_version(current_version) == version:
            manager.update_progress(
                feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                phase="complete",
                status="current",
                percent=percent_end,
                message="ZCU OTA 패키지가 최신 버전입니다",
                active=False,
            )
            return OtaPackageResult(
                feature_id=feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                downloaded=True,
                applied=True,
                updated=False,
                path=package_path,
                version=version,
                release_tag=release_tag,
                release_url=str(release_url) if release_url else None,
                asset_name=download_file,
                downloaded_at=None,
                checked_at=checked_at,
            )

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="download",
            status="downloading",
            percent=percent_start,
            message="ZCU OTA 패키지 다운로드 중",
            active=True,
        )

        package_bytes = manager._request_bytes(
            str(release_asset["browser_download_url"]),
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": self.action_type,
                "target": target_name,
                "message": "ZCU OTA 패키지 다운로드 중",
                "percent_start": percent_start,
                "percent_end": percent_flash_start,
            },
        )

        package_path.parent.mkdir(parents=True, exist_ok=True)
        temp_target = package_path.with_suffix(package_path.suffix + ".tmp")
        temp_target.write_bytes(package_bytes)
        temp_target.replace(package_path)

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="install",
            status="flashing",
            percent=percent_flash_start,
            message="ZCU sparse OTA 전송 중",
            active=True,
            bytes_downloaded=len(package_bytes),
            total_bytes=len(package_bytes),
        )

        success = manager.flash_sparse_package_via_doip(
            package_path,
            feature_id,
            action,
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": self.action_type,
                "target": target_name,
                "message": "ZCU sparse OTA 전송 중",
                "percent_start": percent_flash_start,
                "percent_end": percent_end,
            },
        )

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="complete" if success else "error",
            status="complete" if success else "failed",
            percent=percent_end if success else None,
            message="ZCU sparse OTA 전송 완료" if success else "ZCU sparse OTA 전송 실패",
            active=False,
            bytes_downloaded=len(package_bytes),
            total_bytes=len(package_bytes),
        )

        return OtaPackageResult(
            feature_id=feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            downloaded=True,
            applied=success,
            updated=True,
            path=package_path,
            version=version,
            release_tag=release_tag,
            release_url=str(release_url) if release_url else None,
            asset_name=download_file,
            downloaded_at=checked_at,
            checked_at=checked_at,
            error=None if success else "ZCU sparse OTA failed",
        )


class DoipSensorCanOtaAction(OtaAction):
    action_type = "doip_sensor_can_ota"

    def run(
        self,
        manager: "OtaManager",
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None,
        force: bool,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        action_id = str(action.get("id", "sensor_ecu_ota"))
        target_name = str(action.get("target", "sensor-ecu"))
        checked_at = utc_now()
        percent_start, percent_end = action_progress_span(action)
        percent_transfer_start = percent_start + int((percent_end - percent_start) * 0.25)

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="checking",
            status="checking",
            percent=percent_start,
            message="Sensor ECU OTA 패키지 릴리즈 확인 중",
            active=True,
        )

        release = manager.fetch_latest_release(catalog_item, action)
        release_tag = str(release.get("tag_name") or "")
        version = clean_firmware_version(release_tag) or str(catalog_item.get("latest_version", "1.0.0"))
        release_url = release.get("html_url")
        release_asset = manager.release_ota_package_asset(release, action)
        download_file = str(release_asset["name"])
        package_path = manager.resolve_target_path(
            {**action, "target_dir": str(action.get("target_dir", "firmware"))},
            download_file,
        )

        if not force and package_path.exists() and clean_firmware_version(current_version) == version:
            manager.update_progress(
                feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                phase="complete",
                status="current",
                percent=percent_end,
                message="Sensor ECU OTA 패키지가 최신 버전입니다",
                active=False,
            )
            return OtaPackageResult(
                feature_id=feature_id,
                action_id=action_id,
                action_type=self.action_type,
                target=target_name,
                downloaded=True,
                applied=True,
                updated=False,
                path=package_path,
                version=version,
                release_tag=release_tag,
                release_url=str(release_url) if release_url else None,
                asset_name=download_file,
                downloaded_at=None,
                checked_at=checked_at,
            )

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="download",
            status="downloading",
            percent=percent_start,
            message="Sensor ECU OTA 패키지 다운로드 중",
            active=True,
        )

        package_bytes = manager._request_bytes(
            str(release_asset["browser_download_url"]),
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": self.action_type,
                "target": target_name,
                "message": "Sensor ECU OTA 패키지 다운로드 중",
                "percent_start": percent_start,
                "percent_end": percent_transfer_start,
            },
        )

        package_path.parent.mkdir(parents=True, exist_ok=True)
        temp_target = package_path.with_suffix(package_path.suffix + ".tmp")
        temp_target.write_bytes(package_bytes)
        temp_target.replace(package_path)

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="install",
            status="flashing",
            percent=percent_transfer_start,
            message="ZCU 경유 Sensor ECU sparse OTA 중",
            active=True,
            bytes_downloaded=len(package_bytes),
            total_bytes=len(package_bytes),
        )

        success = manager.flash_sensor_sparse_package_via_doip(
            package_path,
            feature_id,
            action,
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": self.action_type,
                "target": target_name,
                "message": "ZCU 경유 Sensor ECU sparse OTA 중",
                "percent_start": percent_transfer_start,
                "percent_end": percent_end,
            },
        )

        manager.update_progress(
            feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            phase="complete" if success else "error",
            status="complete" if success else "failed",
            percent=percent_end if success else None,
            message="Sensor ECU sparse OTA 완료" if success else "Sensor ECU sparse OTA 실패",
            active=False,
            bytes_downloaded=len(package_bytes),
            total_bytes=len(package_bytes),
        )

        return OtaPackageResult(
            feature_id=feature_id,
            action_id=action_id,
            action_type=self.action_type,
            target=target_name,
            downloaded=True,
            applied=success,
            updated=True,
            path=package_path,
            version=version,
            release_tag=release_tag,
            release_url=str(release_url) if release_url else None,
            asset_name=download_file,
            downloaded_at=checked_at,
            checked_at=checked_at,
            error=None if success else "Sensor ECU sparse OTA failed",
        )


def action_progress_span(action: dict[str, Any]) -> tuple[int, int]:
    start = int(action.get("progress_start", 0))
    end = int(action.get("progress_end", 100))
    start = max(0, min(100, start))
    end = max(0, min(100, end))
    if end < start:
        end = start
    return start, end


def clean_version(tag: str | None) -> str | None:
    if not tag:
        return None
    value = str(tag).strip()
    return value[1:] if value.lower().startswith("v") and len(value) > 1 else value


def clean_firmware_version(tag: str | None) -> str | None:
    value = clean_version(tag)
    if not value:
        return None
    match = re.search(r"(?<!\d)(\d+)[._-](\d+)[._-](\d+)(?!\d)", value)
    if match:
        return ".".join(str(int(part)) for part in match.groups())

    match = re.search(r"(?<!\d)(\d+)[._-](\d+)(?!\d)", value)
    if not match:
        return None
    return f"{int(match.group(1))}.{int(match.group(2))}.0"


def release_version_tuple(tag: str | None) -> tuple[int, int, int] | None:
    normalized = clean_firmware_version(tag)
    if not normalized:
        return None
    major, minor, patch = normalized.split(".", 2)
    return int(major), int(minor), int(patch)


class OtaManager:
    """Middle layer for downloadable vehicle software."""

    def __init__(
        self,
        *,
        base_dir: Path,
        downloaded_features_dir: Path,
        firmware_dir: Path | None = None,
        timeout_seconds: float = 10.0,
        flash_state_callback: Callable[[bool], None] | None = None,
        flash_network_lock: Any | None = None,
    ) -> None:
        self.base_dir = base_dir
        self.downloaded_features_dir = downloaded_features_dir
        self.firmware_dir = firmware_dir or (base_dir / "firmware")
        self.timeout_seconds = timeout_seconds
        self._flash_state_callback = flash_state_callback
        self._flash_network_lock = flash_network_lock
        self._flash_state_lock = threading.Lock()
        self._flash_state_depth = 0
        self._progress_lock = threading.Lock()
        self._progress: dict[str, dict[str, Any]] = {}
        self._actions: dict[str, OtaAction] = {
            LocalFileAction.action_type: LocalFileAction(),
            GithubReleaseFileAction.action_type: GithubReleaseFileAction(),
            DoipUdsFlashAction.action_type: DoipUdsFlashAction(),
            DoipSensorCanOtaAction.action_type: DoipSensorCanOtaAction(),
        }

    def _enter_flash(self) -> None:
        callback = None
        with self._flash_state_lock:
            self._flash_state_depth += 1
            if self._flash_state_depth == 1:
                callback = self._flash_state_callback
        if callback is not None:
            callback(True)
        time.sleep(0.2)

    def _leave_flash(self) -> None:
        callback = None
        with self._flash_state_lock:
            if self._flash_state_depth > 0:
                self._flash_state_depth -= 1
            if self._flash_state_depth == 0:
                callback = self._flash_state_callback
        if callback is not None:
            callback(False)

    def is_flashing(self) -> bool:
        with self._flash_state_lock:
            return self._flash_state_depth > 0

    def download_feature_package(
        self,
        catalog_item: dict[str, Any],
        *,
        current_version: str | None = None,
        force: bool = True,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        checked_at = utc_now()
        if not catalog_item.get("downloadable"):
            return OtaPackageResult(
                feature_id=feature_id,
                action_id="none",
                action_type="none",
                target="none",
                downloaded=False,
                applied=False,
                updated=False,
                path=None,
                version=None,
                release_tag=None,
                release_url=None,
                asset_name=None,
                downloaded_at=None,
                checked_at=checked_at,
            )

        action = self.python_package_action(catalog_item)
        return self.run_action(
            catalog_item,
            action,
            current_version=current_version,
            force=force,
        )

    def run_action(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None = None,
        force: bool = True,
    ) -> OtaPackageResult:
        action_type = str(action.get("type", ""))
        handler = self._actions.get(action_type)
        if handler is None:
            raise ValueError(f"unsupported OTA action type: {action_type}")
        try:
            return handler.run(
                self,
                catalog_item,
                action,
                current_version=current_version,
                force=force,
            )
        except Exception as exc:
            self.update_progress(
                str(catalog_item["id"]),
                action_id=str(action.get("id", "unknown")),
                action_type=action_type,
                target=str(action.get("target", "unknown")),
                phase="error",
                status="error",
                percent=None,
                message=str(exc),
                active=False,
            )
            raise

    def _firmware_action_context(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
    ) -> dict[str, Any]:
        release = self.fetch_latest_release(catalog_item, action)
        release_tag = str(release.get("tag_name") or "")
        version = clean_firmware_version(release_tag) or str(catalog_item.get("latest_version", "1.0.0"))
        if str(action.get("type", "")) in (
            DoipUdsFlashAction.action_type,
            DoipSensorCanOtaAction.action_type,
        ):
            release_asset = self.release_ota_package_asset(release, action)
        else:
            release_asset = self.release_bin_asset(release)
        download_file = str(release_asset["name"])
        bin_path = self.resolve_target_path(
            {**action, "target_dir": str(action.get("target_dir", "firmware"))},
            download_file,
        )
        return {
            "release": release,
            "release_tag": release_tag,
            "release_url": release.get("html_url"),
            "version": version,
            "asset_name": download_file,
            "asset_url": release_asset["browser_download_url"],
            "path": bin_path,
        }

    def download_firmware_payload(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None = None,
        force: bool = True,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        action_id = str(action.get("id", "firmware"))
        action_type = str(action.get("type", "firmware"))
        target_name = str(action.get("target", "firmware"))
        checked_at = utc_now()
        percent_start, percent_end = action_progress_span(action)

        self.update_progress(
            feature_id,
            action_id=action_id,
            action_type=action_type,
            target=target_name,
            phase="checking",
            status="checking",
            percent=percent_start,
            message=f"{target_name} firmware release checking",
            active=True,
        )
        context = self._firmware_action_context(catalog_item, action)
        bin_path = context["path"]
        version = str(context["version"])

        if not force and bin_path.exists() and clean_firmware_version(current_version) == version:
            self.update_progress(
                feature_id,
                action_id=action_id,
                action_type=action_type,
                target=target_name,
                phase="complete",
                status="current",
                percent=percent_end,
                message=f"{target_name} firmware already downloaded",
                active=False,
                bytes_downloaded=bin_path.stat().st_size,
                total_bytes=bin_path.stat().st_size,
            )
            return OtaPackageResult(
                feature_id=feature_id,
                action_id=action_id,
                action_type=action_type,
                target=target_name,
                downloaded=True,
                applied=False,
                updated=False,
                path=bin_path,
                version=version,
                release_tag=str(context["release_tag"]),
                release_url=str(context["release_url"]) if context["release_url"] else None,
                asset_name=str(context["asset_name"]),
                downloaded_at=None,
                checked_at=checked_at,
            )

        firmware_bytes = self._request_bytes(
            str(context["asset_url"]),
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": action_type,
                "target": target_name,
                "message": f"{target_name} firmware downloading",
                "percent_start": percent_start,
                "percent_end": percent_end,
            },
        )
        asset_name = str(context["asset_name"])
        bin_path.parent.mkdir(parents=True, exist_ok=True)
        temp_target = bin_path.with_suffix(bin_path.suffix + ".tmp")
        temp_target.write_bytes(firmware_bytes)
        temp_target.replace(bin_path)
        self.update_progress(
            feature_id,
            action_id=action_id,
            action_type=action_type,
            target=target_name,
            phase="complete",
            status="downloaded",
            percent=percent_end,
            message=f"{target_name} firmware download complete",
            active=False,
            bytes_downloaded=len(firmware_bytes),
            total_bytes=len(firmware_bytes),
        )
        return OtaPackageResult(
            feature_id=feature_id,
            action_id=action_id,
            action_type=action_type,
            target=target_name,
            downloaded=True,
            applied=False,
            updated=True,
            path=bin_path,
            version=version,
            release_tag=str(context["release_tag"]),
            release_url=str(context["release_url"]) if context["release_url"] else None,
            asset_name=asset_name,
            downloaded_at=checked_at,
            checked_at=checked_at,
        )

    def _downloaded_firmware_action_context(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
    ) -> dict[str, Any]:
        def metadata(path: Path, asset_name: Any = None) -> dict[str, Any]:
            release_tag = str(action.get("downloaded_release_tag") or "")
            version = (
                str(action.get("downloaded_version") or "")
                or clean_firmware_version(release_tag)
                or str(catalog_item.get("latest_version", "1.0.0"))
            )
            return {
                "release": None,
                "release_tag": release_tag,
                "release_url": action.get("downloaded_release_url"),
                "version": version,
                "asset_name": str(asset_name or path.name),
                "path": path,
            }

        downloaded_asset_name = action.get("downloaded_asset_name")
        if downloaded_asset_name:
            path = self.resolve_target_path(
                {**action, "target_dir": str(action.get("target_dir", "firmware"))},
                str(downloaded_asset_name),
            )
            if not path.exists():
                raise FileNotFoundError(f"downloaded firmware not found: {path}")
            return metadata(path, downloaded_asset_name)

        return self._firmware_action_context(catalog_item, action)

    def _existing_firmware_action_context(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
    ) -> dict[str, Any] | None:
        def metadata(path: Path, asset_name: Any = None) -> dict[str, Any]:
            release_tag = str(action.get("downloaded_release_tag") or "")
            version = (
                str(action.get("downloaded_version") or "")
                or clean_firmware_version(release_tag)
                or str(catalog_item.get("latest_version", "1.0.0"))
            )
            return {
                "release_tag": release_tag,
                "release_url": action.get("downloaded_release_url"),
                "version": version,
                "asset_name": str(asset_name or path.name),
                "path": path,
            }

        downloaded_asset_name = action.get("downloaded_asset_name")
        if downloaded_asset_name:
            path = self.resolve_target_path(
                {**action, "target_dir": str(action.get("target_dir", "firmware"))},
                str(downloaded_asset_name),
            )
            if not path.exists():
                raise FileNotFoundError(f"downloaded firmware not found: {path}")
            return metadata(path, downloaded_asset_name)
        return None

    def flash_firmware_file(
        self,
        bin_path: Path,
        feature_id: str,
        action: dict[str, Any],
        *,
        progress: dict[str, Any] | None = None,
        version: str | None = None,
        release_tag: str | None = None,
        release_url: str | None = None,
        asset_name: str | None = None,
        downloaded_at: str | None = None,
        checked_at: str | None = None,
        updated: bool = True,
    ) -> OtaPackageResult:
        action_id = str(action.get("id", "firmware"))
        action_type = str(action.get("type", "firmware"))
        target_name = str(action.get("target", "firmware"))
        checked_at = checked_at or utc_now()
        percent_start, percent_end = action_progress_span(action)
        if progress:
            percent_start = int(progress.get("percent_start", percent_start))
            percent_end = int(progress.get("percent_end", percent_end))
        firmware_size = bin_path.stat().st_size
        flash_progress = {
            "feature_id": feature_id,
            "action_id": action_id,
            "action_type": action_type,
            "target": target_name,
            "message": f"{target_name} OTA flashing",
            "percent_start": percent_start,
            "percent_end": percent_end,
        }
        if progress:
            flash_progress.update(progress)
            flash_progress["percent_start"] = percent_start
            flash_progress["percent_end"] = percent_end

        self.update_progress(
            feature_id,
            action_id=action_id,
            action_type=action_type,
            target=target_name,
            phase="install",
            status="flashing",
            percent=percent_start,
            message=str(flash_progress.get("message", f"{target_name} OTA flashing")),
            active=True,
            bytes_downloaded=0,
            total_bytes=firmware_size,
        )

        if action_type == DoipSensorCanOtaAction.action_type:
            if self.is_sparse_package_path(bin_path):
                success = self.flash_sensor_sparse_package_via_doip(
                    bin_path,
                    feature_id,
                    action,
                    progress=flash_progress,
                )
            else:
                success = self.flash_sensor_can_ota_via_doip(
                    bin_path,
                    feature_id,
                    action,
                    progress=flash_progress,
                )
        elif action_type == DoipUdsFlashAction.action_type:
            if self.is_sparse_package_path(bin_path):
                success = self.flash_sparse_package_via_doip(
                    bin_path,
                    feature_id,
                    action,
                    progress=flash_progress,
                )
            else:
                success = self.flash_bin_via_doip(
                    bin_path,
                    feature_id,
                    action,
                    progress=flash_progress,
                )
        else:
            raise ValueError(f"unsupported firmware flash action type: {action_type}")

        complete_message = str(flash_progress.get("complete_message", f"{target_name} OTA complete"))
        failed_message = str(flash_progress.get("failed_message", f"{target_name} OTA failed"))
        self.update_progress(
            feature_id,
            action_id=action_id,
            action_type=action_type,
            target=target_name,
            phase="complete" if success else "error",
            status="complete" if success else "failed",
            percent=percent_end if success else None,
            message=complete_message if success else failed_message,
            active=False,
            bytes_downloaded=firmware_size,
            total_bytes=firmware_size,
        )
        return OtaPackageResult(
            feature_id=feature_id,
            action_id=action_id,
            action_type=action_type,
            target=target_name,
            downloaded=True,
            applied=success,
            updated=updated,
            path=bin_path,
            version=version,
            release_tag=release_tag,
            release_url=release_url,
            asset_name=asset_name or bin_path.name,
            downloaded_at=downloaded_at,
            checked_at=checked_at,
            error=None if success else failed_message,
        )

    def flash_firmware_payload(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        current_version: str | None = None,
        force: bool = True,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        action_id = str(action.get("id", "firmware"))
        action_type = str(action.get("type", "firmware"))
        target_name = str(action.get("target", "firmware"))
        checked_at = utc_now()
        percent_start, percent_end = action_progress_span(action)
        existing_context = self._existing_firmware_action_context(catalog_item, action)
        if existing_context is not None:
            return self.flash_firmware_file(
                existing_context["path"],
                feature_id,
                action,
                progress={
                    "feature_id": feature_id,
                    "action_id": action_id,
                    "action_type": action_type,
                    "target": target_name,
                    "message": f"{target_name} OTA flashing",
                    "percent_start": percent_start,
                    "percent_end": percent_end,
                },
                version=str(existing_context["version"]),
                release_tag=str(existing_context["release_tag"]) if existing_context["release_tag"] else None,
                release_url=str(existing_context["release_url"]) if existing_context["release_url"] else None,
                asset_name=str(existing_context["asset_name"]),
                checked_at=checked_at,
            )

        self.update_progress(
            feature_id,
            action_id=action_id,
            action_type=action_type,
            target=target_name,
            phase="checking",
            status="checking",
            percent=percent_start,
            message=f"{target_name} firmware release checking",
            active=True,
        )
        context = self._firmware_action_context(catalog_item, action)
        bin_path = context["path"]
        version = str(context["version"])
        if not force and bin_path.exists() and clean_firmware_version(current_version) == version:
            return self.flash_firmware_file(
                bin_path,
                feature_id,
                action,
                progress={
                    "feature_id": feature_id,
                    "action_id": action_id,
                    "action_type": action_type,
                    "target": target_name,
                    "message": f"{target_name} OTA flashing",
                    "percent_start": percent_start,
                    "percent_end": percent_end,
                },
                version=version,
                release_tag=str(context["release_tag"]),
                release_url=str(context["release_url"]) if context["release_url"] else None,
                asset_name=str(context["asset_name"]),
                checked_at=checked_at,
                updated=False,
            )

        flash_start = percent_start + int((percent_end - percent_start) * 0.25)
        firmware_bytes = self._request_bytes(
            str(context["asset_url"]),
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": action_type,
                "target": target_name,
                "message": f"{target_name} firmware downloading",
                "percent_start": percent_start,
                "percent_end": flash_start,
            },
        )
        asset_name = str(context["asset_name"])
        bin_path.parent.mkdir(parents=True, exist_ok=True)
        temp_target = bin_path.with_suffix(bin_path.suffix + ".tmp")
        temp_target.write_bytes(firmware_bytes)
        temp_target.replace(bin_path)
        return self.flash_firmware_file(
            bin_path,
            feature_id,
            action,
            progress={
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": action_type,
                "target": target_name,
                "message": f"{target_name} OTA flashing",
                "percent_start": flash_start,
                "percent_end": percent_end,
            },
            version=version,
            release_tag=str(context["release_tag"]),
            release_url=str(context["release_url"]) if context["release_url"] else None,
            asset_name=asset_name,
            downloaded_at=checked_at,
            checked_at=checked_at,
        )

    def flash_downloaded_firmware_payload(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        progress: dict[str, Any] | None = None,
    ) -> OtaPackageResult:
        feature_id = str(catalog_item["id"])
        action_id = str(action.get("id", "firmware"))
        action_type = str(action.get("type", "firmware"))
        target_name = str(action.get("target", "firmware"))
        checked_at = utc_now()
        percent_start, percent_end = action_progress_span(action)
        context = self._downloaded_firmware_action_context(catalog_item, action)
        bin_path = context["path"]
        if not bin_path.exists():
            raise FileNotFoundError(f"downloaded firmware not found: {bin_path}")

        return self.flash_firmware_file(
            bin_path,
            feature_id,
            action,
            progress=progress or {
                "feature_id": feature_id,
                "action_id": action_id,
                "action_type": action_type,
                "target": target_name,
                "message": f"{target_name} OTA flashing",
                "percent_start": percent_start,
                "percent_end": percent_end,
            },
            version=str(context["version"]),
            release_tag=str(context["release_tag"]),
            release_url=str(context["release_url"]) if context["release_url"] else None,
            asset_name=str(context["asset_name"]),
            downloaded_at=None,
            checked_at=checked_at,
        )

    def update_progress(
        self,
        feature_id: str,
        *,
        action_id: str,
        action_type: str,
        target: str,
        phase: str,
        status: str,
        percent: int | None,
        message: str,
        active: bool,
        bytes_downloaded: int | None = None,
        total_bytes: int | None = None,
    ) -> None:
        progress = {
            "feature_id": feature_id,
            "action_id": action_id,
            "action_type": action_type,
            "target": target,
            "phase": phase,
            "status": status,
            "percent": percent,
            "message": message,
            "active": active,
            "bytes_downloaded": bytes_downloaded,
            "total_bytes": total_bytes,
            "updated_at": utc_now(),
        }
        with self._progress_lock:
            self._progress[feature_id] = progress

    def progress_for(self, feature_id: str) -> dict[str, Any]:
        with self._progress_lock:
            progress = self._progress.get(feature_id)
            if progress is None:
                return {
                    "feature_id": feature_id,
                    "active": False,
                    "percent": None,
                    "status": "idle",
                    "phase": "idle",
                    "message": "",
                }
            return dict(progress)

    def clear_progress(self, feature_id: str | None = None) -> None:
        with self._progress_lock:
            if feature_id is None:
                self._progress.clear()
            else:
                self._progress.pop(feature_id, None)

    def actions_for(self, catalog_item: dict[str, Any]) -> list[dict[str, Any]]:
        actions = catalog_item.get("ota_actions")
        if isinstance(actions, list) and actions:
            return [action for action in actions if isinstance(action, dict)]

        if catalog_item.get("release_repo") and catalog_item.get("download_file"):
            return [
                {
                    "id": "python_package",
                    "type": "github_release_file",
                    "target": "rpi",
                    "release_repo": catalog_item["release_repo"],
                    "download_file": catalog_item["download_file"],
                    "target_dir": "features",
                }
            ]

        return []

    def python_package_action(self, catalog_item: dict[str, Any]) -> dict[str, Any]:
        for action in self.actions_for(catalog_item):
            if action.get("type") in ("github_release_file", "local_file"):
                return action
        raise ValueError(f"python package OTA action is not configured: {catalog_item['id']}")

    def resolve_target_path(self, action: dict[str, Any], download_file: str) -> Path:
        target_dir = str(action.get("target_dir", "features"))
        if Path(download_file).is_absolute():
            return Path(download_file)
        if target_dir == "features":
            return self.downloaded_features_dir / download_file
        if target_dir == "firmware":
            return self.firmware_dir / download_file
        return self.base_dir / target_dir / download_file

    def resolve_source_path(self, action: dict[str, Any]) -> Path:
        source_file = action.get("source_file")
        if not source_file:
            raise ValueError("local_file OTA action requires source_file")
        source = Path(str(source_file))
        if source.is_absolute():
            return source
        return self.base_dir / source

    def fetch_latest_release(self, catalog_item: dict[str, Any], action: dict[str, Any]) -> dict[str, Any]:
        repo = action.get("release_repo") or catalog_item.get("release_repo")
        if not repo:
            raise ValueError(f"release repo is not configured: {catalog_item['id']}")

        if action.get("release_patch_filter") is not None:
            expected_patch = int(action["release_patch_filter"])
            try:
                return self.fetch_highest_patch_release(
                    catalog_item,
                    action,
                    expected_patch=expected_patch,
                )
            except RuntimeError as exc:
                if "download failed (403)" not in str(exc):
                    raise
                return self.fetch_highest_patch_release_from_github_web(
                    repo,
                    expected_patch=expected_patch,
                )

        url = f"https://api.github.com/repos/{repo}/releases/latest"
        try:
            data = self._request_bytes(url, accept="application/vnd.github+json")
            payload = json.loads(data.decode("utf-8"))
            if not isinstance(payload, dict) or not payload.get("tag_name"):
                raise RuntimeError(f"latest release payload is invalid: {repo}")
            return payload
        except RuntimeError as exc:
            if "download failed (403)" not in str(exc):
                raise
            return self.fetch_latest_release_from_github_web(repo)

    def fetch_highest_patch_release(
        self,
        catalog_item: dict[str, Any],
        action: dict[str, Any],
        *,
        expected_patch: int,
    ) -> dict[str, Any]:
        repo = action.get("release_repo") or catalog_item.get("release_repo")
        if not repo:
            raise ValueError(f"release repo is not configured: {catalog_item['id']}")

        url = f"https://api.github.com/repos/{repo}/releases?per_page=100"
        data = self._request_bytes(url, accept="application/vnd.github+json")
        payload = json.loads(data.decode("utf-8"))
        if not isinstance(payload, list):
            raise RuntimeError(f"release list payload is invalid: {repo}")

        candidates: list[tuple[tuple[int, int, int], dict[str, Any]]] = []
        for release in payload:
            if not isinstance(release, dict):
                continue
            if release.get("draft") or release.get("prerelease"):
                continue
            version = release_version_tuple(str(release.get("tag_name") or ""))
            if version is None or version[2] != expected_patch:
                continue
            candidates.append((version, release))

        if not candidates:
            raise RuntimeError(f"no x.x.{expected_patch} release found: {repo}")
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    def fetch_latest_release_from_github_web(self, repo: str) -> dict[str, Any]:
        _, final_url = self._request_text(f"https://github.com/{repo}/releases/latest")
        tag = self._release_tag_from_url(final_url)
        if not tag:
            raise RuntimeError(f"latest release redirect is invalid: {repo}")
        return self.fetch_github_web_release(repo, tag)

    def fetch_highest_patch_release_from_github_web(
        self,
        repo: str,
        *,
        expected_patch: int,
    ) -> dict[str, Any]:
        text, _ = self._request_text(f"https://github.com/{repo}/releases")
        tag_pattern = re.compile(
            rf"/{re.escape(repo)}/releases/tag/([^\"?#<>]+)"
        )
        candidates: list[tuple[tuple[int, int, int], str]] = []
        seen: set[str] = set()
        for raw_tag in tag_pattern.findall(text):
            tag = urllib.parse.unquote(html.unescape(raw_tag))
            if tag in seen:
                continue
            seen.add(tag)
            version = release_version_tuple(tag)
            if version is None or version[2] != expected_patch:
                continue
            candidates.append((version, tag))

        if not candidates:
            raise RuntimeError(f"no x.x.{expected_patch} release found: {repo}")
        candidates.sort(key=lambda item: item[0], reverse=True)
        return self.fetch_github_web_release(repo, candidates[0][1])

    def fetch_github_web_release(self, repo: str, tag: str) -> dict[str, Any]:
        quoted_tag = urllib.parse.quote(tag, safe="")
        assets_url = f"https://github.com/{repo}/releases/expanded_assets/{quoted_tag}"
        text, _ = self._request_text(assets_url)
        assets = self._github_web_assets(repo, text)
        if not assets:
            release_url = f"https://github.com/{repo}/releases/tag/{quoted_tag}"
            text, _ = self._request_text(release_url)
            assets = self._github_web_assets(repo, text)
        return {
            "tag_name": tag,
            "html_url": f"https://github.com/{repo}/releases/tag/{quoted_tag}",
            "assets": assets,
        }

    @staticmethod
    def _release_tag_from_url(url: str) -> str | None:
        path = urllib.parse.urlparse(url).path
        marker = "/releases/tag/"
        if marker not in path:
            return None
        return urllib.parse.unquote(path.split(marker, 1)[1].strip("/"))

    @staticmethod
    def _github_web_assets(repo: str, text: str) -> list[dict[str, Any]]:
        href_pattern = re.compile(
            rf"href=\"([^\"]*/{re.escape(repo)}/releases/download/[^\"]+)\""
        )
        assets: list[dict[str, Any]] = []
        seen: set[str] = set()
        for raw_href in href_pattern.findall(text):
            href = html.unescape(raw_href)
            url = urllib.parse.urljoin("https://github.com", href)
            path = urllib.parse.unquote(urllib.parse.urlparse(url).path)
            name = path.rsplit("/", 1)[-1]
            if not name or url in seen:
                continue
            seen.add(url)
            assets.append({"name": name, "browser_download_url": url})
        return assets

    def download_release_asset(
        self,
        release: dict[str, Any],
        download_file: str,
        *,
        progress: dict[str, Any] | None = None,
    ) -> tuple[bytes, str]:
        assets = release.get("assets", [])
        if not isinstance(assets, list):
            assets = []

        for asset in assets:
            if not isinstance(asset, dict):
                continue
            if asset.get("name") != download_file:
                continue
            asset_url = asset.get("browser_download_url")
            if not asset_url:
                break
            return self._request_bytes(str(asset_url), progress=progress), download_file

        raise FileNotFoundError(f"release asset not found: {download_file}")

    @staticmethod
    def release_bin_asset(release: dict[str, Any]) -> dict[str, Any]:
        assets = release.get("assets", [])
        if not isinstance(assets, list):
            assets = []

        bin_assets = [
            asset
            for asset in assets
            if isinstance(asset, dict)
            and asset.get("browser_download_url")
            and str(asset.get("name", "")).lower().endswith(".bin")
        ]
        if len(bin_assets) != 1:
            names = ", ".join(str(asset.get("name")) for asset in bin_assets) or "none"
            raise FileNotFoundError(f"expected exactly one .bin release asset, found: {names}")
        return bin_assets[0]

    @staticmethod
    def release_ota_package_asset(
        release: dict[str, Any],
        action: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        assets = release.get("assets", [])
        if not isinstance(assets, list):
            assets = []

        valid_assets = [
            asset
            for asset in assets
            if isinstance(asset, dict)
            and asset.get("browser_download_url")
            and str(asset.get("name", ""))
        ]

        if action:
            package_file = action.get("package_file")
            if package_file:
                expected_name = str(package_file)
                if not expected_name.lower().endswith(".zip"):
                    raise ValueError(f"action.package_file must be a .zip OTA package: {expected_name}")
                for asset in valid_assets:
                    if str(asset.get("name", "")) == expected_name:
                        return asset
                raise FileNotFoundError(f"configured OTA package release asset not found: {expected_name}")

            # Backward-compatible fallback only when download_file itself is already a zip.
            download_file = action.get("download_file")
            if download_file and str(download_file).lower().endswith(".zip"):
                expected_name = str(download_file)
                for asset in valid_assets:
                    if str(asset.get("name", "")) == expected_name:
                        return asset
                raise FileNotFoundError(f"configured OTA package release asset not found: {expected_name}")

        package_assets = [
            asset
            for asset in valid_assets
            if str(asset.get("name", "")).lower().endswith(".zip")
            and (
                "ota" in str(asset.get("name", "")).lower()
                or "package" in str(asset.get("name", "")).lower()
                or "sparse" in str(asset.get("name", "")).lower()
            )
        ]

        if len(package_assets) == 1:
            return package_assets[0]

        zip_assets = [
            asset
            for asset in valid_assets
            if str(asset.get("name", "")).lower().endswith(".zip")
        ]

        if len(zip_assets) == 1:
            return zip_assets[0]

        names = ", ".join(str(asset.get("name")) for asset in valid_assets) or "none"
        raise FileNotFoundError(
            "expected exactly one OTA package .zip release asset "
            "or configure action.package_file. assets: "
            f"{names}"
        )

    def _request_bytes(
        self,
        url: str,
        *,
        accept: str = "application/octet-stream",
        progress: dict[str, Any] | None = None,
    ) -> bytes:
        request = urllib.request.Request(url, headers=self._request_headers(accept))
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_seconds) as response:
                total_header = response.headers.get("Content-Length")
                total_bytes = int(total_header) if total_header and total_header.isdigit() else None
                if progress is None:
                    return response.read()

                chunks: list[bytes] = []
                downloaded = 0
                last_percent: int | None = None
                percent_start = int(progress.get("percent_start", 0))
                percent_end = int(progress.get("percent_end", 100))
                while True:
                    chunk = response.read(64 * 1024)
                    if not chunk:
                        break
                    chunks.append(chunk)
                    downloaded += len(chunk)
                    percent = (
                        min(100, int(downloaded * 100 / total_bytes))
                        if total_bytes
                        else None
                    )
                    if percent is not None:
                        percent = percent_start + int(percent * (percent_end - percent_start) / 100)
                    if percent != last_percent:
                        last_percent = percent
                        self.update_progress(
                            str(progress["feature_id"]),
                            action_id=str(progress["action_id"]),
                            action_type=str(progress["action_type"]),
                            target=str(progress["target"]),
                            phase="download",
                            status="downloading",
                            percent=percent,
                            message=str(progress["message"]),
                            active=True,
                            bytes_downloaded=downloaded,
                            total_bytes=total_bytes,
                        )

                data = b"".join(chunks)
                self.update_progress(
                    str(progress["feature_id"]),
                    action_id=str(progress["action_id"]),
                    action_type=str(progress["action_type"]),
                    target=str(progress["target"]),
                    phase="download",
                    status="downloading",
                    percent=percent_end,
                    message=str(progress["message"]),
                    active=True,
                    bytes_downloaded=len(data),
                    total_bytes=total_bytes or len(data),
                )
                return data
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"download failed ({exc.code}): {url}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"download failed: {url}: {exc.reason}") from exc

    def _request_text(self, url: str) -> tuple[str, str]:
        request = urllib.request.Request(
            url,
            headers=self._request_headers("text/html,application/xhtml+xml"),
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_seconds) as response:
                raw = response.read()
                content_type = response.headers.get_content_charset() or "utf-8"
                return raw.decode(content_type, errors="replace"), response.geturl()
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"download failed ({exc.code}): {url}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"download failed: {url}: {exc.reason}") from exc

    @staticmethod
    def _request_headers(accept: str) -> dict[str, str]:
        headers = {
            "Accept": accept,
            "User-Agent": "vehicle-computer-ota",
            "X-GitHub-Api-Version": "2022-11-28",
        }
        token = os.getenv("GITHUB_TOKEN") or os.getenv("GH_TOKEN")
        if token:
            headers["Authorization"] = f"Bearer {token}"
        return headers

    def download_zcu_payload(self, catalog_item: dict[str, Any]) -> OtaPackageResult:
        return self.run_action(catalog_item, self.zcu_flash_action(catalog_item), force=False)

    def flash_zcu_payload(self, catalog_item: dict[str, Any]) -> OtaPackageResult:
        return self.run_action(catalog_item, self.zcu_flash_action(catalog_item), force=True)

    def zcu_flash_action(self, catalog_item: dict[str, Any]) -> dict[str, Any]:
        for action in self.actions_for(catalog_item):
            if action.get("type") == "doip_uds_flash":
                return action
        return {
            "id": "zcu_flash",
            "type": "doip_uds_flash",
            "target": "zcu",
        }

    @staticmethod
    def _parse_u32(value: Any, field_name: str) -> int:
        if value is None:
            raise ValueError(f"missing OTA package manifest field: {field_name}")
        if isinstance(value, str):
            raw = value.strip()
            if raw.lower().startswith("0x"):
                return int(raw, 16)
            return int(raw, 10)
        return int(value)

    @staticmethod
    def _first_present(mapping: dict[str, Any], *keys: str) -> Any:
        for key in keys:
            if key in mapping:
                return mapping[key]
        return None

    @staticmethod
    def is_sparse_package_path(path: Path) -> bool:
        if path.is_dir():
            return (path / "manifest.json").exists() or (path / "ota_segments" / "manifest.json").exists()
        return path.suffix.lower() == ".zip"

    def _prepare_sparse_package_dir(self, package_path: Path) -> Path:
        if package_path.is_dir():
            return package_path

        if package_path.suffix.lower() != ".zip":
            raise ValueError(f"not an OTA sparse package: {package_path}")

        extract_dir = package_path.with_suffix("")
        if extract_dir.exists():
            shutil.rmtree(extract_dir)
        extract_dir.mkdir(parents=True, exist_ok=True)

        with zipfile.ZipFile(package_path, "r") as archive:
            root = extract_dir.resolve()
            for member in archive.infolist():
                target = (extract_dir / member.filename).resolve()
                if not str(target).startswith(str(root)):
                    raise ValueError(f"unsafe path in OTA package zip: {member.filename}")
            archive.extractall(extract_dir)

        return extract_dir

    def load_sparse_ota_package(self, package_path: Path) -> SparseOtaPackage:
        package_dir = self._prepare_sparse_package_dir(package_path)

        manifest_candidates = [
            package_dir / "manifest.json",
            package_dir / "ota_segments" / "manifest.json",
        ]
        manifest_path = next((path for path in manifest_candidates if path.exists()), None)
        if manifest_path is None:
            matches = list(package_dir.rglob("manifest.json"))
            if len(matches) == 1:
                manifest_path = matches[0]
        if manifest_path is None or not manifest_path.exists():
            raise FileNotFoundError(f"manifest.json not found in OTA package: {package_path}")

        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

        virtual_size = self._parse_u32(manifest.get("virtualSize"), "virtualSize")
        virtual_crc32 = self._parse_u32(
            self._first_present(
                manifest,
                "virtualImageCrc32",
                "virtualCRC32",
                "virtualCrc32",
                "virtualCRC",
                "crc32",
            ),
            "virtualImageCrc32",
        )
        gap_fill_value = self._first_present(manifest, "virtualGapFill", "gapFill")
        gap_fill = self._parse_u32(0 if gap_fill_value is None else gap_fill_value, "virtualGapFill")
        total_payload_size = self._parse_u32(
            manifest.get("totalPayloadSize"),
            "totalPayloadSize",
        )

        raw_segments = manifest.get("segments")
        if not isinstance(raw_segments, list) or not raw_segments:
            raise ValueError("OTA package manifest has no segments")

        manifest_dir = manifest_path.parent
        project_dir = manifest_dir.parent
        segments: list[SparseOtaSegment] = []

        for idx, raw_segment in enumerate(raw_segments):
            if not isinstance(raw_segment, dict):
                raise ValueError(f"invalid segment entry at index {idx}")

            segment_index = int(raw_segment.get("index", idx + 1))
            name = str(raw_segment.get("name") or f"segment{segment_index}")
            offset = self._parse_u32(
                self._first_present(
                    raw_segment,
                    "offsetFromAppBase",
                    "offset",
                    "segmentOffset",
                    "virtualOffset",
                    "appOffset",
                ),
                f"segments[{idx}].offsetFromAppBase",
            )
            size = self._parse_u32(raw_segment.get("size"), f"segments[{idx}].size")
            crc_value = raw_segment.get("crc32")
            crc32 = self._parse_u32(crc_value, f"segments[{idx}].crc32") if crc_value is not None else None

            file_name = str(raw_segment.get("file") or f"segment{segment_index}.bin")
            file_path = Path(file_name)
            candidates: list[Path]
            if file_path.is_absolute():
                candidates = [file_path]
            else:
                candidates = [
                    file_path,
                    project_dir / file_path,
                    manifest_dir / file_path,
                    manifest_dir / file_path.name,
                    package_dir / file_path,
                    package_dir / file_path.name,
                    package_dir / "ota_segments" / file_path.name,
                ]
            segment_path = next((path for path in candidates if path.exists()), None)
            if segment_path is None:
                raise FileNotFoundError(
                    f"segment file not found for {name}: {file_name}"
                )

            data = segment_path.read_bytes()
            if len(data) != size:
                raise ValueError(
                    f"segment size mismatch for {name}: manifest={size}, file={len(data)}, path={segment_path}"
                )

            segments.append(
                SparseOtaSegment(
                    index=segment_index,
                    name=name,
                    offset=offset,
                    size=size,
                    crc32=crc32,
                    path=segment_path,
                    data=data,
                )
            )

        segments.sort(key=lambda segment: segment.offset)
        payload_size = sum(segment.size for segment in segments)
        if payload_size != total_payload_size:
            raise ValueError(
                f"totalPayloadSize mismatch: manifest={total_payload_size}, segments={payload_size}"
            )

        for index, segment in enumerate(segments):
            if segment.size <= 0:
                raise ValueError(f"invalid segment size: {segment.name}")
            end_offset = segment.offset + segment.size
            if end_offset > virtual_size:
                raise ValueError(
                    f"segment outside virtual image: {segment.name}, end=0x{end_offset:X}, virtual=0x{virtual_size:X}"
                )
            if index > 0:
                prev = segments[index - 1]
                prev_end = prev.offset + prev.size
                if segment.offset < prev_end:
                    raise ValueError(
                        f"overlapping segments: {prev.name} and {segment.name}"
                    )

        return SparseOtaPackage(
            manifest_path=manifest_path,
            package_dir=package_dir,
            virtual_size=virtual_size,
            virtual_crc32=virtual_crc32,
            gap_fill=gap_fill,
            total_payload_size=total_payload_size,
            segments=tuple(segments),
        )

    @staticmethod
    def _u32be(value: int) -> bytes:
        return int(value).to_bytes(4, byteorder="big", signed=False)

    @staticmethod
    def _build_sparse_manifest_uds(package: SparseOtaPackage) -> bytes:
        if len(package.segments) <= 0 or len(package.segments) > 255:
            raise ValueError("invalid sparse segment count")

        payload = bytearray()
        payload.append(0xB4)
        payload += OtaManager._u32be(package.virtual_size)
        payload += OtaManager._u32be(package.virtual_crc32)
        payload.append(len(package.segments) & 0xFF)
        payload.append(package.gap_fill & 0xFF)

        for segment in package.segments:
            payload += OtaManager._u32be(segment.offset)
            payload += OtaManager._u32be(segment.size)

        return bytes(payload)

    def _send_sparse_manifest_and_download_request(
        self,
        sock: socket.socket,
        package: SparseOtaPackage,
        *,
        tester_address: int,
        zcu_address: int,
        request_timeout: float,
        stage_prefix: str,
    ) -> int:
        response = self._sensor_gateway_send_uds(
            sock,
            bytes([0x10, 0x03]),
            tester_address,
            zcu_address,
            request_timeout,
        )
        self._expect_uds_positive(response, 0x10)

        response = self._sensor_gateway_send_uds(
            sock,
            self._build_sparse_manifest_uds(package),
            tester_address,
            zcu_address,
            request_timeout,
        )
        self._expect_uds_positive(response, 0xB4)

        request_download = (
            bytes([0x34, 0x00, 0x44])
            + b"\x00\x00\x00\x00"
            + self._u32be(package.total_payload_size)
        )
        response = self._sensor_gateway_send_uds(
            sock,
            request_download,
            tester_address,
            zcu_address,
            request_timeout,
        )
        self._expect_uds_positive(response, 0x34)

        if len(response) >= 4:
            max_block = int.from_bytes(response[2:4], "big")
            max_payload = max_block - 2
            if max_payload <= 0:
                raise SensorGatewayDoipError(
                    f"{stage_prefix} invalid RequestDownload maxBlock={max_block}"
                )
            return max_payload

        return 512

    def _wait_sparse_gateway_ready(
        self,
        sock: socket.socket,
        *,
        tester_address: int,
        zcu_address: int,
        request_timeout: float,
        ready_timeout_seconds: float,
    ) -> None:
        deadline = time.monotonic() + ready_timeout_seconds
        while time.monotonic() < deadline:
            response = self._sensor_gateway_send_uds(
                sock,
                bytes([0xB5]),
                tester_address,
                zcu_address,
                request_timeout,
            )
            if len(response) >= 2 and response[0] == 0xF5 and response[1] == 0x01:
                return
            if len(response) >= 2 and response[0] == 0xF5 and response[1] == 0x7F:
                raise SensorGatewayDoipError("OTA gateway entered error state during ReadyCheck")
            time.sleep(1.0)
        raise SensorGatewayDoipError("timeout waiting OTA gateway ready")

    def _transfer_sparse_package_stream(
        self,
        sock: socket.socket,
        package: SparseOtaPackage,
        *,
        tester_address: int,
        zcu_address: int,
        request_timeout: float,
        chunk_size: int,
        block_delay_seconds: float,
        progress: dict[str, Any] | None,
        percent_start: int,
        percent_end: int,
        progress_update_interval_blocks: int,
        progress_message: str,
    ) -> int:
        payload = package.payload
        total = len(payload)
        offset = 0
        seq = 1
        block_count = 0
        last_progress_block = -progress_update_interval_blocks

        while offset < total:
            chunk = payload[offset:offset + chunk_size]
            response = self._sensor_gateway_send_uds(
                sock,
                bytes([0x36, seq]) + chunk,
                tester_address,
                zcu_address,
                request_timeout,
            )
            self._expect_uds_positive(response, 0x36)
            if len(response) >= 2 and response[1] != seq:
                raise SensorGatewayDoipError(
                    f"TransferData sequence mismatch: sent=0x{seq:02X}, got=0x{response[1]:02X}"
                )

            offset += len(chunk)
            seq = 0 if seq >= 0xFF else seq + 1
            block_count += 1

            if progress:
                raw_percent = min(100, int(offset * 100 / total)) if total else 100
                percent = percent_start + int(raw_percent * (percent_end - percent_start) / 100)
                if (
                    block_count - last_progress_block >= progress_update_interval_blocks
                    or offset >= total
                ):
                    last_progress_block = block_count
                    self.update_progress(
                        str(progress["feature_id"]),
                        action_id=str(progress["action_id"]),
                        action_type=str(progress["action_type"]),
                        target=str(progress["target"]),
                        phase="install",
                        status="flashing",
                        percent=percent,
                        message=progress_message,
                        active=True,
                        bytes_downloaded=offset,
                        total_bytes=total,
                    )

            if block_delay_seconds > 0:
                time.sleep(block_delay_seconds)

        return block_count

    def flash_sparse_package_via_doip(
        self,
        package_path: Path,
        feature_id: str,
        action: dict[str, Any],
        *,
        progress: dict[str, Any] | None = None,
    ) -> bool:
        if not package_path.exists():
            raise FileNotFoundError(f"OTA package not found: {package_path}")

        package = self.load_sparse_ota_package(package_path)
        total = package.total_payload_size
        ecu_ip = str(action.get("ecu_ip", "192.168.10.2"))
        doip_port = int(action.get("doip_port", 13400))
        tester_address = int(action.get("tester_address", 0x0E00))
        zcu_address = int(action.get("ecu_address", action.get("zcu_address", 0x0001)))
        request_timeout = float(action.get("timeout_seconds", 60.0))
        chunk_size = int(action.get("chunk_size", 512))
        block_delay = float(action.get("block_delay_seconds", 0.0))
        percent_start = int((progress or {}).get("percent_start", 50))
        percent_end = int((progress or {}).get("percent_end", 100))
        progress_update_interval_blocks = max(1, int(action.get("progress_update_interval_blocks", 10)))

        if chunk_size <= 0 or chunk_size > 512:
            raise ValueError("ZCU sparse OTA chunk_size must be 1..512")

        stage = "connecting ZCU self DoIP"
        network_lock_acquired = False
        try:
            self._enter_flash()
            if self._flash_network_lock is not None:
                self._flash_network_lock.acquire()
                network_lock_acquired = True

            with socket.create_connection((ecu_ip, doip_port), timeout=request_timeout) as sock:
                try:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                sock.settimeout(request_timeout)

                stage = "routing activation"
                self._sensor_gateway_activate_routing(sock, tester_address)

                max_payload = self._send_sparse_manifest_and_download_request(
                    sock,
                    package,
                    tester_address=tester_address,
                    zcu_address=zcu_address,
                    request_timeout=request_timeout,
                    stage_prefix="ZCU sparse OTA",
                )
                data_block_size = min(chunk_size, max_payload)

                if progress:
                    self.update_progress(
                        str(progress["feature_id"]),
                        action_id=str(progress["action_id"]),
                        action_type=str(progress["action_type"]),
                        target=str(progress["target"]),
                        phase="install",
                        status="flashing",
                        percent=percent_start,
                        message=(
                            f"ZCU sparse OTA 시작: "
                            f"payload={total}, block={data_block_size}, crc=0x{package.virtual_crc32:08X}"
                        ),
                        active=True,
                        bytes_downloaded=0,
                        total_bytes=total,
                    )

                stage = "TransferData"
                block_count = self._transfer_sparse_package_stream(
                    sock,
                    package,
                    tester_address=tester_address,
                    zcu_address=zcu_address,
                    request_timeout=request_timeout,
                    chunk_size=data_block_size,
                    block_delay_seconds=block_delay,
                    progress=progress,
                    percent_start=percent_start,
                    percent_end=percent_end,
                    progress_update_interval_blocks=progress_update_interval_blocks,
                    progress_message=str((progress or {}).get("message", "ZCU sparse OTA 전송 중")),
                )

                stage = f"RequestTransferExit crc32=0x{package.virtual_crc32:08X}"
                response = self._sensor_gateway_send_uds(
                    sock,
                    bytes([0x37]) + self._u32be(package.virtual_crc32),
                    tester_address,
                    zcu_address,
                    request_timeout,
                )
                self._expect_uds_positive(response, 0x37)

                if progress:
                    self.update_progress(
                        str(progress["feature_id"]),
                        action_id=str(progress["action_id"]),
                        action_type=str(progress["action_type"]),
                        target=str(progress["target"]),
                        phase="install",
                        status="flashing",
                        percent=percent_end,
                        message=f"ZCU TransferExit OK: blocks={block_count}",
                        active=True,
                        bytes_downloaded=total,
                        total_bytes=total,
                    )

            return True
        except Exception as exc:
            if isinstance(exc, RuntimeError):
                raise
            raise RuntimeError(
                f"ZCU sparse OTA failed during {stage}: {type(exc).__name__}: {exc}"
            ) from exc
        finally:
            if network_lock_acquired and self._flash_network_lock is not None:
                self._flash_network_lock.release()
            self._leave_flash()

    def flash_sensor_sparse_package_via_doip(
        self,
        package_path: Path,
        feature_id: str,
        action: dict[str, Any],
        *,
        progress: dict[str, Any] | None = None,
    ) -> bool:
        if not package_path.exists():
            raise FileNotFoundError(f"OTA package not found: {package_path}")

        package = self.load_sparse_ota_package(package_path)
        total = package.total_payload_size
        ecu_ip = str(action.get("ecu_ip", "192.168.10.2"))
        doip_port = int(action.get("doip_port", 13401))
        tester_address = int(action.get("tester_address", 0x0E00))
        zcu_address = int(action.get("ecu_address", action.get("zcu_address", 0x0001)))
        request_timeout = float(action.get("timeout_seconds", 60.0))
        ready_timeout_seconds = float(action.get("ready_check_timeout_seconds", 120.0))
        block_size = int(action.get("block_size", action.get("chunk_size", 32)))
        block_delay = float(action.get("block_delay_seconds", 0.0))
        activate = bool(action.get("activate_after_transfer", False))
        percent_start = int((progress or {}).get("percent_start", 0))
        percent_end = int((progress or {}).get("percent_end", 100))
        progress_update_interval_blocks = max(1, int(action.get("progress_update_interval_blocks", 50)))

        if block_size <= 0 or block_size > SENSOR_CAN_OTA_MAX_DATA_BLOCK_SIZE:
            raise ValueError(
                f"block_size must be 1..{SENSOR_CAN_OTA_MAX_DATA_BLOCK_SIZE} "
                "for Sensor ECU sparse OTA gateway"
            )

        stage = "connecting Sensor ECU sparse OTA DoIP gateway"
        network_lock_acquired = False
        try:
            self._enter_flash()
            if self._flash_network_lock is not None:
                self._flash_network_lock.acquire()
                network_lock_acquired = True

            with socket.create_connection((ecu_ip, doip_port), timeout=request_timeout) as sock:
                try:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                sock.settimeout(request_timeout)

                stage = "routing activation"
                self._sensor_gateway_activate_routing(sock, tester_address)

                max_payload = self._send_sparse_manifest_and_download_request(
                    sock,
                    package,
                    tester_address=tester_address,
                    zcu_address=zcu_address,
                    request_timeout=request_timeout,
                    stage_prefix="Sensor sparse OTA",
                )
                data_block_size = min(block_size, max_payload)

                stage = "B5 ReadyCheck"
                self._wait_sparse_gateway_ready(
                    sock,
                    tester_address=tester_address,
                    zcu_address=zcu_address,
                    request_timeout=request_timeout,
                    ready_timeout_seconds=ready_timeout_seconds,
                )

                if progress:
                    self.update_progress(
                        str(progress["feature_id"]),
                        action_id=str(progress["action_id"]),
                        action_type=str(progress["action_type"]),
                        target=str(progress["target"]),
                        phase="install",
                        status="flashing",
                        percent=percent_start,
                        message=(
                            f"Sensor sparse OTA 시작: "
                            f"payload={total}, block={data_block_size}, crc=0x{package.virtual_crc32:08X}"
                        ),
                        active=True,
                        bytes_downloaded=0,
                        total_bytes=total,
                    )

                stage = "TransferData"
                block_count = self._transfer_sparse_package_stream(
                    sock,
                    package,
                    tester_address=tester_address,
                    zcu_address=zcu_address,
                    request_timeout=request_timeout,
                    chunk_size=data_block_size,
                    block_delay_seconds=block_delay,
                    progress=progress,
                    percent_start=percent_start,
                    percent_end=percent_end,
                    progress_update_interval_blocks=progress_update_interval_blocks,
                    progress_message=str((progress or {}).get("message", "Sensor ECU sparse OTA 중")),
                )

                stage = f"RequestTransferExit crc32=0x{package.virtual_crc32:08X}"
                response = self._sensor_gateway_send_uds(
                    sock,
                    bytes([0x37]) + self._u32be(package.virtual_crc32),
                    tester_address,
                    zcu_address,
                    request_timeout,
                )
                self._expect_uds_positive(response, 0x37)

                if activate:
                    stage = "ECUReset"
                    response = self._sensor_gateway_send_uds(
                        sock,
                        bytes([0x11, 0x01]),
                        tester_address,
                        zcu_address,
                        request_timeout,
                    )
                    self._expect_uds_positive(response, 0x11)
                    if len(response) >= 2 and response[1] != 0x01:
                        raise SensorGatewayDoipError(
                            f"ECUReset subfunction mismatch: expected=0x01, got=0x{response[1]:02X}"
                        )

                if progress:
                    self.update_progress(
                        str(progress["feature_id"]),
                        action_id=str(progress["action_id"]),
                        action_type=str(progress["action_type"]),
                        target=str(progress["target"]),
                        phase="install",
                        status="flashing",
                        percent=percent_end,
                        message=f"Sensor TransferExit OK: blocks={block_count}",
                        active=True,
                        bytes_downloaded=total,
                        total_bytes=total,
                    )

            return True
        except Exception as exc:
            if isinstance(exc, RuntimeError):
                raise
            raise RuntimeError(
                f"Sensor ECU sparse OTA failed during {stage}: {type(exc).__name__}: {exc}"
            ) from exc
        finally:
            if network_lock_acquired and self._flash_network_lock is not None:
                self._flash_network_lock.release()
            self._leave_flash()

    def flash_bin_via_doip(
        self,
        bin_path: Path,
        feature_id: str,
        action: dict[str, Any],
        *,
        progress: dict[str, Any] | None = None,
    ) -> bool:
        if not bin_path.exists():
            raise FileNotFoundError(f"bin file not found: {bin_path}")

        firmware = bin_path.read_bytes()
        total = len(firmware)
        if total == 0:
            raise ValueError("firmware bin is empty")

        ecu_ip = str(action.get("ecu_ip", "192.168.10.2"))
        doip_port = int(action.get("doip_port", 13400))
        ecu_address = int(action.get("ecu_address", 0x0001))
        bank_start = int(action.get("bank_start", 0x80300000))
        request_timeout = float(action.get("timeout_seconds", 60.0))
        percent_start = int((progress or {}).get("percent_start", 50))
        percent_end = int((progress or {}).get("percent_end", 100))

        def fail(message: str, *, offset: int = 0) -> None:
            if progress:
                self.update_progress(
                    str(progress["feature_id"]),
                    action_id=str(progress["action_id"]),
                    action_type=str(progress["action_type"]),
                    target=str(progress["target"]),
                    phase="error",
                    status="failed",
                    percent=None,
                    message=message,
                    active=False,
                    bytes_downloaded=offset,
                    total_bytes=total,
                )
            raise RuntimeError(message)

        def set_progress(
            phase: str,
            status: str,
            percent: int | None,
            message: str,
            *,
            offset: int = 0,
            active: bool = True,
        ) -> None:
            if not progress:
                return
            self.update_progress(
                str(progress["feature_id"]),
                action_id=str(progress["action_id"]),
                action_type=str(progress["action_type"]),
                target=str(progress["target"]),
                phase=phase,
                status=status,
                percent=percent,
                message=message,
                active=active,
                bytes_downloaded=offset,
                total_bytes=total,
            )

        doip_connection = None
        stage = "importing DoIP/UDS libraries"
        network_lock_acquired = False
        try:
            self._enter_flash()
            if self._flash_network_lock is not None:
                self._flash_network_lock.acquire()
                network_lock_acquired = True
            import doipclient
            import udsoncan
            import udsoncan.configs
            import udsoncan.services
            from doipclient.connectors import DoIPClientUDSConnector
            from udsoncan.client import Client

            stage = f"connecting DoIP {ecu_ip}:{doip_port}"
            doip_connection = doipclient.DoIPClient(
                ecu_ip,
                ecu_address,
                tcp_port=doip_port,
            )
            uds_connection = DoIPClientUDSConnector(doip_connection)
            client_config = dict(udsoncan.configs.default_client_config)
            client_config.update(
                {
                    "request_timeout": request_timeout,
                    "p2_timeout": float(action.get("p2_timeout_seconds", request_timeout)),
                    "p2_star_timeout": float(action.get("p2_star_timeout_seconds", request_timeout)),
                    "use_server_timing": bool(action.get("use_server_timing", False)),
                }
            )
            client = Client(uds_connection, config=client_config)

            set_progress("install", "flashing", percent_start, f"DoIP connected: {ecu_ip}:{doip_port}")
            with client:
                stage = "DiagnosticSessionControl extended"
                client.change_session(
                    udsoncan.services.DiagnosticSessionControl.Session.extendedDiagnosticSession
                )
                set_progress("install", "flashing", percent_start, "Extended Session active")

                memory_location = udsoncan.MemoryLocation(
                    address=bank_start,
                    memorysize=total,
                    address_format=32,
                    memorysize_format=32,
                )
                stage = f"RequestDownload addr=0x{bank_start:08X} size={total}"
                response = client.request_download(memory_location)
                max_block = int(response.service_data.max_length)
                max_payload = max(1, max_block - 2)
                set_progress(
                    "install",
                    "flashing",
                    percent_start,
                    f"RequestDownload complete: addr=0x{bank_start:08X} maxBlock={max_block}",
                )

                offset = 0
                sequence = 1
                last_percent: int | None = None

                while offset < total:
                    chunk = firmware[offset:offset + max_payload]
                    stage = f"TransferData seq={sequence} offset={offset}/{total}"
                    client.transfer_data(sequence, chunk)

                    offset += len(chunk)
                    sequence = (sequence + 1) & 0xFF
                    raw_percent = min(100, int(offset * 100 / total)) if total else 100
                    percent = percent_start + int(raw_percent * (percent_end - percent_start) / 100)
                    if percent != last_percent:
                        last_percent = percent
                        set_progress(
                            "install",
                            "flashing",
                            percent,
                            str((progress or {}).get("message", "ZCU firmware flashing")),
                            offset=offset,
                        )

                transfer_exit_delay = float(action.get("transfer_exit_delay_seconds", 0.5))
                if transfer_exit_delay > 0:
                    time.sleep(transfer_exit_delay)
                crc32 = zlib.crc32(firmware) & 0xFFFFFFFF
                stage = f"RequestTransferExit crc32=0x{crc32:08X}"
                try:
                    client.request_transfer_exit(crc32.to_bytes(4, byteorder="big"))
                except Exception as exc:
                    if bool(action.get("transfer_exit_timeout_success", True)) and "respond in time" in str(exc).lower():
                        set_progress(
                            "install",
                            "flashing",
                            percent_end,
                            f"TransferExit sent; ECU reset/no response treated as complete: crc32=0x{crc32:08X}",
                            offset=total,
                        )
                        return True
                    raise
                set_progress(
                    "install",
                    "flashing",
                    percent_end,
                    f"TransferExit complete: crc32=0x{crc32:08X}",
                    offset=total,
                )

            return True
        except Exception as exc:
            if isinstance(exc, RuntimeError):
                raise
            fail(f"DoIP/UDS flash failed during {stage}: {type(exc).__name__}: {exc}")
        finally:
            if doip_connection is not None:
                try:
                    doip_connection.close()
                except Exception:
                    pass
            if network_lock_acquired and self._flash_network_lock is not None:
                self._flash_network_lock.release()
            self._leave_flash()

    def flash_sensor_can_ota_via_doip(
        self,
        bin_path: Path,
        feature_id: str,
        action: dict[str, Any],
        *,
        progress: dict[str, Any] | None = None,
    ) -> bool:
        firmware = bin_path.read_bytes()
        total = len(firmware)
        if total == 0:
            raise ValueError("firmware bin is empty")

        ecu_ip = str(action.get("ecu_ip", "192.168.10.2"))
        doip_port = int(action.get("doip_port", 13401))
        tester_address = int(action.get("tester_address", 0x0E00))
        zcu_address = int(action.get("ecu_address", action.get("zcu_address", 0x0001)))
        app_addr = int(action.get("app_addr", action.get("bank_start", 0x80020000)))
        block_size = int(action.get("block_size", SENSOR_CAN_OTA_MAX_DATA_BLOCK_SIZE))
        request_timeout = float(action.get("timeout_seconds", 60.0))
        block_delay = float(action.get("block_delay_seconds", 0.0))
        activate = bool(action.get("activate_after_transfer", False))
        percent_start = int((progress or {}).get("percent_start", 0))
        percent_end = int((progress or {}).get("percent_end", 100))
        progress_update_interval_blocks = max(1, int(action.get("progress_update_interval_blocks", 1)))

        if block_size <= 0 or block_size > SENSOR_CAN_OTA_MAX_DATA_BLOCK_SIZE:
            raise ValueError(
                f"block_size must be 1..{SENSOR_CAN_OTA_MAX_DATA_BLOCK_SIZE} "
                "for Sensor ECU CAN OTA gateway"
            )

        crc32 = zlib.crc32(firmware) & 0xFFFFFFFF

        def fail(message: str) -> None:
            if progress:
                self.update_progress(
                    str(progress["feature_id"]),
                    action_id=str(progress["action_id"]),
                    action_type=str(progress["action_type"]),
                    target=str(progress["target"]),
                    phase="error",
                    status="failed",
                    percent=None,
                    message=message,
                    active=False,
                    bytes_downloaded=None,
                    total_bytes=total,
                )
            raise RuntimeError(message)

        def set_progress(
            phase: str,
            status: str,
            percent: int | None,
            message: str,
            *,
            offset: int = 0,
            active: bool = True,
        ) -> None:
            if not progress:
                return
            self.update_progress(
                str(progress["feature_id"]),
                action_id=str(progress["action_id"]),
                action_type=str(progress["action_type"]),
                target=str(progress["target"]),
                phase=phase,
                status=status,
                percent=percent,
                message=message,
                active=active,
                bytes_downloaded=offset,
                total_bytes=total,
            )

        stage = "connecting Sensor ECU OTA DoIP gateway"
        network_lock_acquired = False
        try:
            self._enter_flash()
            if self._flash_network_lock is not None:
                self._flash_network_lock.acquire()
                network_lock_acquired = True
            with socket.create_connection((ecu_ip, doip_port), timeout=request_timeout) as sock:
                try:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                sock.settimeout(request_timeout)
                set_progress("install", "flashing", percent_start, f"DoIP gateway connected: {ecu_ip}:{doip_port}")

                stage = "routing activation"
                self._sensor_gateway_activate_routing(sock, tester_address)
                set_progress("install", "flashing", percent_start, "Sensor OTA routing activation OK")

                stage = "DiagnosticSessionControl extended"
                response = self._sensor_gateway_send_uds(
                    sock,
                    bytes([0x10, 0x03]),
                    tester_address,
                    zcu_address,
                    request_timeout,
                )
                self._expect_uds_positive(response, 0x10)

                stage = f"RequestDownload addr=0x{app_addr:08X} size={total}"
                request_download = bytes([0x34, 0x00, 0x44]) + app_addr.to_bytes(4, "big") + total.to_bytes(4, "big")
                response = self._sensor_gateway_send_uds(
                    sock,
                    request_download,
                    tester_address,
                    zcu_address,
                    request_timeout,
                )
                self._expect_uds_positive(response, 0x34)

                if len(response) >= 4:
                    max_block = int.from_bytes(response[2:4], "big")
                    max_payload = max_block - SENSOR_CAN_OTA_TRANSFER_OVERHEAD
                    if max_payload <= 0:
                        raise SensorGatewayDoipError(
                            f"invalid Sensor RequestDownload maxBlock={max_block}"
                        )
                    data_block_size = min(block_size, max_payload)
                else:
                    data_block_size = block_size
                set_progress("install", "flashing", percent_start, f"Sensor RequestDownload OK: block={data_block_size}")

                offset = 0
                seq = 1
                block_count = 0
                last_progress_block = -progress_update_interval_blocks

                while offset < total:
                    chunk = firmware[offset:offset + data_block_size]
                    block_index = block_count
                    stage = f"TransferData block={block_index} seq={seq} offset={offset}/{total}"
                    try:
                        response = self._sensor_gateway_send_uds(
                            sock,
                            bytes([0x36, seq]) + chunk,
                            tester_address,
                            zcu_address,
                            request_timeout,
                        )
                    except RuntimeError as exc:
                        raise SensorGatewayDoipError(
                            f"TransferData failed at block={block_index}, "
                            f"seq=0x{seq:02X}, offset={offset}, length={len(chunk)}: {exc}"
                        ) from exc
                    self._expect_uds_positive(response, 0x36)
                    if len(response) >= 2 and response[1] != seq:
                        raise SensorGatewayDoipError(
                            f"TransferData sequence mismatch at block={block_index}, "
                            f"offset={offset}: sent=0x{seq:02X}, got=0x{response[1]:02X}"
                        )

                    offset += len(chunk)
                    seq = 0 if seq >= 0xFF else seq + 1
                    block_count += 1
                    raw_percent = min(100, int(offset * 100 / total))
                    percent = percent_start + int(raw_percent * (percent_end - percent_start) / 100)
                    if (
                        block_count - last_progress_block >= progress_update_interval_blocks
                        or offset >= total
                    ):
                        last_progress_block = block_count
                        set_progress(
                            "install",
                            "flashing",
                            percent,
                            str((progress or {}).get("message", "Sensor ECU CAN OTA")),
                            offset=offset,
                        )
                    if block_delay > 0:
                        time.sleep(block_delay)

                stage = f"RequestTransferExit crc32=0x{crc32:08X}"
                response = self._sensor_gateway_send_uds(
                    sock,
                    bytes([0x37]) + crc32.to_bytes(4, "big"),
                    tester_address,
                    zcu_address,
                    request_timeout,
                )
                self._expect_uds_positive(response, 0x37)
                set_progress(
                    "install",
                    "flashing",
                    percent_end,
                    f"Sensor TransferExit OK: blocks={block_count}",
                    offset=total,
                )

                if activate:
                    stage = "ECUReset"
                    response = self._sensor_gateway_send_uds(
                        sock,
                        bytes([0x11, 0x01]),
                        tester_address,
                        zcu_address,
                        request_timeout,
                    )
                    self._expect_uds_positive(response, 0x11)
                    if len(response) >= 2 and response[1] != 0x01:
                        raise SensorGatewayDoipError(
                            f"ECUReset subfunction mismatch: expected=0x01, got=0x{response[1]:02X}"
                        )

            return True
        except Exception as exc:
            fail(f"Sensor ECU CAN OTA failed during {stage}: {type(exc).__name__}: {exc}")
        finally:
            if network_lock_acquired and self._flash_network_lock is not None:
                self._flash_network_lock.release()
            self._leave_flash()

    @staticmethod
    def _build_doip(payload_type: int, payload: bytes) -> bytes:
        version = SENSOR_GATEWAY_DOIP_VERSION
        header = struct.pack(">BBHI", version, (~version) & 0xFF, payload_type, len(payload))
        return header + payload

    @staticmethod
    def _recv_exact(sock: socket.socket, length: int) -> bytes:
        chunks: list[bytes] = []
        remaining = length
        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                raise SensorGatewayDoipError("socket closed while waiting for DoIP response")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    @classmethod
    def _recv_doip(cls, sock: socket.socket) -> tuple[int, bytes]:
        header = cls._recv_exact(sock, 8)
        version, inverse, payload_type, payload_len = struct.unpack(">BBHI", header)
        if version != SENSOR_GATEWAY_DOIP_VERSION or inverse != ((~SENSOR_GATEWAY_DOIP_VERSION) & 0xFF):
            raise SensorGatewayDoipError(f"bad DoIP header: version=0x{version:02X}, inverse=0x{inverse:02X}")
        return payload_type, cls._recv_exact(sock, payload_len)

    @classmethod
    def _sensor_gateway_activate_routing(cls, sock: socket.socket, tester_address: int) -> None:
        payload = struct.pack(">HB", tester_address, 0x00) + b"\x00\x00\x00\x00"
        sock.sendall(cls._build_doip(SENSOR_GATEWAY_PT_ROUTING_ACT_REQ, payload))
        payload_type, response = cls._recv_doip(sock)
        if payload_type != SENSOR_GATEWAY_PT_ROUTING_ACT_RES:
            raise SensorGatewayDoipError(f"expected routing activation response, got 0x{payload_type:04X}")
        if len(response) < 5:
            raise SensorGatewayDoipError("short routing activation response")
        response_code = response[4]
        if response_code != 0x10:
            raise SensorGatewayDoipError(f"routing activation failed: code=0x{response_code:02X}")

    @classmethod
    def _sensor_gateway_send_uds(
        cls,
        sock: socket.socket,
        uds: bytes,
        tester_address: int,
        zcu_address: int,
        timeout_seconds: float,
    ) -> bytes:
        diag_payload = struct.pack(">HH", tester_address, zcu_address) + uds
        sock.sendall(cls._build_doip(SENSOR_GATEWAY_PT_DIAG_MESSAGE, diag_payload))
        deadline = time.monotonic() + timeout_seconds
        got_ack = False

        while time.monotonic() < deadline:
            sock.settimeout(max(0.1, deadline - time.monotonic()))
            payload_type, payload = cls._recv_doip(sock)
            if payload_type == SENSOR_GATEWAY_PT_DIAG_ACK:
                if len(payload) >= 5 and payload[4] != 0x00:
                    raise SensorGatewayDoipError(f"diagnostic ACK failed: code=0x{payload[4]:02X}")
                got_ack = True
                continue

            if payload_type == SENSOR_GATEWAY_PT_DIAG_MESSAGE:
                if len(payload) < 4:
                    raise SensorGatewayDoipError("short diagnostic response")
                response = payload[4:]
                if not got_ack:
                    # Some ZCU builds may send the diagnostic response before the ACK.
                    pass
                if len(response) >= 3 and response[0] == 0x7F:
                    raise SensorGatewayDoipError(
                        f"UDS negative response: sid=0x{response[1]:02X}, nrc=0x{response[2]:02X}"
                    )
                return response

            raise SensorGatewayDoipError(f"unexpected DoIP payload type: 0x{payload_type:04X}")

        raise SensorGatewayDoipError("timeout waiting for diagnostic response")

    @staticmethod
    def _expect_uds_positive(response: bytes, expected_sid: int) -> None:
        if not response:
            raise SensorGatewayDoipError("empty UDS response")
        expected = (expected_sid + 0x40) & 0xFF
        if response[0] != expected:
            raise SensorGatewayDoipError(
                f"unexpected UDS response for 0x{expected_sid:02X}: got {response.hex(' ').upper()}"
            )

    def clear_downloaded_feature_packages(self) -> list[str]:
        removed: list[str] = []
        if not self.downloaded_features_dir.exists():
            return removed

        base = self.downloaded_features_dir.resolve()
        for path in self.downloaded_features_dir.iterdir():
            resolved = path.resolve()
            if resolved == base or base not in resolved.parents:
                raise RuntimeError(f"refusing to remove outside downloaded features dir: {path}")

            if path.is_dir():
                if path.name != "__pycache__":
                    continue
                shutil.rmtree(path)
            else:
                path.unlink()
            removed.append(str(path))

        return removed

    def clear_downloaded_firmware_packages(self) -> list[str]:
        removed: list[str] = []
        if not self.firmware_dir.exists():
            return removed

        base = self.firmware_dir.resolve()
        for path in self.firmware_dir.iterdir():
            resolved = path.resolve()
            if resolved == base or base not in resolved.parents:
                raise RuntimeError(f"refusing to remove outside firmware dir: {path}")
            if path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()
            removed.append(str(path))
        return removed

    def status(self) -> dict[str, Any]:
        return {
            "actions": {
                "local_file": "implemented",
                "github_release_file": "implemented",
                "doip_uds_flash": "implemented",
                "doip_sensor_can_ota": "implemented",
            },
            "downloaded_features_dir": str(self.downloaded_features_dir),
            "firmware_dir": str(self.firmware_dir),
            "timeout_seconds": self.timeout_seconds,
        }
