from __future__ import annotations

import os
import importlib.util
import logging
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from ethernet import build_someip_packet


GEAR_P = "P"
GEAR_D = "D"
STEER_CENTER_BYTE = 127
JOYSTICK_BUTTON_P = int(os.getenv("VEHICLE_JOYSTICK_BUTTON_P", "0"))
JOYSTICK_BUTTON_D = int(os.getenv("VEHICLE_JOYSTICK_BUTTON_D", "1"))
JOYSTICK_AXIS_SPEED = int(os.getenv("VEHICLE_JOYSTICK_AXIS_SPEED", "1"))
JOYSTICK_AXIS_STEER = int(os.getenv("VEHICLE_JOYSTICK_AXIS_STEER", "2"))
SPEED_CENTER_BYTE = int(os.getenv("VEHICLE_SPEED_CENTER_BYTE", "127"))
MANUAL_SPEED_DEADZONE = float(os.getenv("VEHICLE_MANUAL_SPEED_DEADZONE", "0.05"))
MANUAL_STEER_DEADZONE = float(os.getenv("VEHICLE_MANUAL_STEER_DEADZONE", "0.02"))
MANUAL_STEER_BYTE_DEADZONE = int(os.getenv("VEHICLE_MANUAL_STEER_BYTE_DEADZONE", "3"))
JOYSTICK_RECONNECT_SETTLE_SECONDS = float(os.getenv("VEHICLE_JOYSTICK_RECONNECT_SETTLE_SECONDS", "0.75"))
JOYSTICK_DISCONNECT_GRACE_SECONDS = float(os.getenv("VEHICLE_JOYSTICK_DISCONNECT_GRACE_SECONDS", "0.5"))


logger = logging.getLogger("vehicle-computer.vehicle-control")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def axis_to_byte(axis_value: float) -> int:
    value = int((axis_value + 1.0) * 127.5)
    return max(0, min(255, value))


def angle_to_byte(angle_deg: float, max_angle: float) -> int:
    if max_angle <= 0:
        return 127
    normalized = max(-1.0, min(1.0, angle_deg / max_angle))
    return axis_to_byte(normalized)


@dataclass(frozen=True)
class JoystickSignal:
    connected: bool
    gear: str
    axis_speed: float
    axis_steer: float
    speed_byte: int
    steer_byte: int
    source: str
    error: str | None = None
    device_name: str | None = None
    axes_count: int = 0
    buttons_count: int = 0
    hats_count: int = 0
    button_p: bool = False
    button_d: bool = False
    raw_axis_speed: float = 0.0
    raw_axis_steer: float = 0.0
    drive_ready: bool = True
    hold_reason: str | None = None


@dataclass(frozen=True)
class FeatureState:
    enabled: bool
    mode: str
    value: dict
    downloaded: bool = False
    applied: bool = False
    error: str | None = None


@dataclass(frozen=True)
class VehicleControlSnapshot:
    joystick: JoystickSignal
    lkas: FeatureState
    fvsa: FeatureState
    aeb: FeatureState
    control_payload_hex: str
    control_packet_hex: str
    updated_at: str

    def to_dict(self) -> dict:
        return asdict(self)


class JoystickReader:
    def __init__(
        self,
        *,
        button_p: int,
        button_d: int,
        axis_speed: int,
        axis_steer: int,
    ) -> None:
        self._pygame = None
        self._joystick = None
        self._gear = GEAR_P
        self._prev_button_p = False
        self._prev_button_d = False
        self._next_retry_at = 0.0
        self._was_connected = False
        self._last_unavailable_reason: str | None = None
        self._name: str | None = None
        self._axes = 0
        self._buttons = 0
        self._hats = 0
        self._button_p = button_p
        self._button_d = button_d
        self._axis_speed = axis_speed
        self._axis_steer = axis_steer
        self._last_mapping_warning: str | None = None
        self._settle_until = 0.0
        self._last_drive_block_reason: str | None = None
        self._last_drive_block_logged_at = 0.0
        self._disconnect_started_at: float | None = None
        self._disconnect_reason: str | None = None
        self._drive_ready = False

    def read(self) -> JoystickSignal:
        if self._joystick is None:
            self._try_connect()

        if self._joystick is None:
            return self._neutral("no joystick connected")

        try:
            try:
                self._pygame.event.pump()
            except Exception as exc:
                reason = f"pygame event pump failed: {exc}"
                if self._disconnect_grace_expired(reason):
                    self._mark_disconnected(reason)
                    self._joystick = None
                return self._neutral(reason)

            connection_issue = self._connection_issue()
            if connection_issue is not None:
                if self._disconnect_grace_expired(connection_issue):
                    self._mark_disconnected(connection_issue)
                    self._joystick = None
                    return self._neutral("joystick disconnected")
                return self._neutral(f"joystick disconnect grace: {connection_issue}")

            self._reset_disconnect_grace()

            if time.time() < self._settle_until:
                return self._neutral("joystick reconnect settling", connected=True)

            raw_axis_speed = self._read_axis_direct(self._axis_speed, "speed")
            raw_axis_steer = self._read_axis_direct(self._axis_steer, "steer")
            button_p = self._read_button_direct(self._button_p, "P")
            button_d = self._read_button_direct(self._button_d, "D")
            self._update_gear(
                button_p=button_p,
                button_d=button_d,
                axis_speed=raw_axis_speed,
                axis_steer=raw_axis_steer,
            )

            axis_speed = raw_axis_speed
            axis_steer = raw_axis_steer
            hold_reason = None
            drive_ready = self._drive_ready

            if abs(raw_axis_speed) < MANUAL_SPEED_DEADZONE:
                axis_speed = 0.0
                speed_byte = SPEED_CENTER_BYTE
            else:
                speed_byte = axis_to_byte(raw_axis_speed)
            steer_byte = axis_to_byte(raw_axis_steer)

            if self._gear == GEAR_P:
                axis_speed = 0.0
                axis_steer = 0.0
                speed_byte = SPEED_CENTER_BYTE
                steer_byte = STEER_CENTER_BYTE
                drive_ready = False
            elif not self._drive_ready:
                axis_speed = 0.0
                axis_steer = 0.0
                speed_byte = SPEED_CENTER_BYTE
                steer_byte = STEER_CENTER_BYTE
                drive_ready = False
                hold_reason = self._last_drive_block_reason or "drive output held until axes are neutral"

            return JoystickSignal(
                connected=True,
                gear=self._gear,
                axis_speed=axis_speed,
                axis_steer=axis_steer,
                speed_byte=speed_byte,
                steer_byte=steer_byte,
                source="pygame",
                device_name=self._name,
                axes_count=self._axes,
                buttons_count=self._buttons,
                hats_count=self._hats,
                button_p=button_p,
                button_d=button_d,
                raw_axis_speed=raw_axis_speed,
                raw_axis_steer=raw_axis_steer,
                drive_ready=drive_ready,
                hold_reason=hold_reason,
            )
        except Exception as exc:
            self._mark_disconnected(str(exc))
            self._joystick = None
            return self._neutral(str(exc))

    def _is_still_connected(self) -> bool:
        return self._connection_issue() is None

    def _connection_issue(self) -> str | None:
        if self._pygame is None or self._joystick is None:
            return "pygame joystick object is not available"

        try:
            if hasattr(self._joystick, "get_init") and not self._joystick.get_init():
                return "pygame joystick is not initialized"

            if self._pygame.joystick.get_count() == 0:
                return "pygame joystick count is zero"

            return None
        except Exception as exc:
            return f"pygame joystick status failed: {exc}"

    def _disconnect_grace_expired(self, reason: str) -> bool:
        now = time.time()
        if self._disconnect_started_at is None or reason != self._disconnect_reason:
            self._disconnect_started_at = now
            self._disconnect_reason = reason
            logger.warning("joystick disconnect suspected: name=%s reason=%s", self._name, reason)
            return JOYSTICK_DISCONNECT_GRACE_SECONDS <= 0
        return now - self._disconnect_started_at >= JOYSTICK_DISCONNECT_GRACE_SECONDS

    def _reset_disconnect_grace(self) -> None:
        if self._disconnect_started_at is not None:
            logger.info("joystick disconnect recovered: name=%s reason=%s", self._name, self._disconnect_reason)
        self._disconnect_started_at = None
        self._disconnect_reason = None

    def _try_connect(self) -> None:
        now = time.time()
        if now < self._next_retry_at:
            return
        self._next_retry_at = now + 2.0

        try:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
            import pygame

            self._pygame = pygame
            if not pygame.get_init():
                pygame.init()
            # Bluetooth controllers can appear after the process starts. Rebuild
            # SDL's joystick list on each retry so hot-plugged devices are seen.
            if pygame.joystick.get_init():
                pygame.joystick.quit()
            pygame.joystick.init()
            pygame.event.pump()
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                self._mark_unavailable("no joystick connected")
                return

            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self._name = self._joystick.get_name()
            self._axes = self._joystick.get_numaxes()
            self._buttons = self._joystick.get_numbuttons()
            self._hats = self._joystick.get_numhats()
            self._gear = GEAR_P
            self._prev_button_p = self._read_button_direct(self._button_p, "P")
            self._prev_button_d = self._read_button_direct(self._button_d, "D")
            self._drive_ready = False
            self._settle_until = time.time() + JOYSTICK_RECONNECT_SETTLE_SECONDS
            self._was_connected = True
            self._last_unavailable_reason = None
            self._last_drive_block_reason = None
            self._reset_disconnect_grace()
            logger.debug(
                "joystick connected: name=%s axes=%s buttons=%s hats=%s mapping(button_P=%s,button_D=%s,axis_speed=%s,axis_steer=%s)",
                self._name,
                self._axes,
                self._buttons,
                self._hats,
                self._button_p,
                self._button_d,
                self._axis_speed,
                self._axis_steer,
            )
        except Exception as exc:
            self._mark_unavailable(str(exc))
            self._joystick = None

    def _update_gear(self, *, button_p: bool, button_d: bool, axis_speed: float, axis_steer: float) -> None:
        if button_p and (not self._prev_button_p or self._gear != GEAR_P):
            self._gear = GEAR_P
            self._drive_ready = False
            self._last_drive_block_reason = None
        elif button_d and (not self._prev_button_d or self._gear != GEAR_D):
            self._gear = GEAR_D
            if self._drive_axes_centered(axis_speed=axis_speed, axis_steer=axis_steer):
                self._drive_ready = True
                self._last_drive_block_reason = None
            else:
                self._drive_ready = False

        if self._gear == GEAR_D and not self._drive_ready:
            if self._drive_axes_centered(axis_speed=axis_speed, axis_steer=axis_steer):
                logger.info("drive output armed after axes returned to neutral")
                self._drive_ready = True
                self._last_drive_block_reason = None

        self._prev_button_p = button_p
        self._prev_button_d = button_d

    def _capture_button_state(self) -> None:
        self._prev_button_p = self._read_button_direct(self._button_p, "P")
        self._prev_button_d = self._read_button_direct(self._button_d, "D")

    def _read_button_direct(self, button: int, label: str) -> bool:
        button_count = self._joystick.get_numbuttons()
        if 0 <= button < button_count:
            return bool(self._joystick.get_button(button))

        warning = f"{label} button {button} is out of range for {button_count} buttons"
        if warning != self._last_mapping_warning:
            logger.warning(warning)
            self._last_mapping_warning = warning
        return False

    def _drive_axes_centered(self, *, axis_speed: float, axis_steer: float) -> bool:
        centered = abs(axis_speed) < MANUAL_SPEED_DEADZONE and abs(axis_steer) < MANUAL_STEER_DEADZONE
        if not centered:
            reason = f"drive output held until axes are neutral: speed={axis_speed:.3f} steer={axis_steer:.3f}"
            now = time.time()
            if reason != self._last_drive_block_reason or now - self._last_drive_block_logged_at >= 2.0:
                logger.warning(reason)
                self._last_drive_block_logged_at = now
            self._last_drive_block_reason = reason
        return centered

    def _read_axis_direct(self, axis: int, label: str) -> float:
        axis_count = self._joystick.get_numaxes()
        if 0 <= axis < axis_count:
            return float(self._joystick.get_axis(axis))

        warning = f"{label} axis {axis} is out of range for {axis_count} axes"
        if warning != self._last_mapping_warning:
            logger.warning(warning)
            self._last_mapping_warning = warning
        return 0.0

    def _neutral(self, reason: str, *, connected: bool = False) -> JoystickSignal:
        return JoystickSignal(
            connected=connected,
            gear=GEAR_P,
            axis_speed=0.0,
            axis_steer=0.0,
            speed_byte=SPEED_CENTER_BYTE,
            steer_byte=STEER_CENTER_BYTE,
            source="neutral",
            error=reason,
            device_name=self._name,
            axes_count=self._axes,
            buttons_count=self._buttons,
            hats_count=self._hats,
            drive_ready=False,
        )

    def _mark_disconnected(self, reason: str) -> None:
        if self._was_connected:
            logger.warning("joystick disconnected: name=%s reason=%s", self._name, reason)
        self._was_connected = False
        self._gear = GEAR_P
        self._prev_button_p = False
        self._prev_button_d = False
        self._name = None
        self._axes = 0
        self._buttons = 0
        self._hats = 0
        self._last_unavailable_reason = reason
        self._settle_until = 0.0
        self._disconnect_started_at = None
        self._disconnect_reason = None
        self._drive_ready = False

    def _mark_unavailable(self, reason: str) -> None:
        if self._was_connected:
            logger.warning("joystick disconnected: name=%s reason=%s", self._name, reason)
        elif reason != self._last_unavailable_reason:
            logger.debug("joystick unavailable: %s", reason)
        self._was_connected = False
        self._gear = GEAR_P
        self._prev_button_p = False
        self._prev_button_d = False
        self._name = None
        self._axes = 0
        self._buttons = 0
        self._hats = 0
        self._last_unavailable_reason = reason
        self._settle_until = 0.0
        self._disconnect_started_at = None
        self._disconnect_reason = None
        self._drive_ready = False


class DownloadedPythonFeature:
    def __init__(
        self,
        *,
        feature_id: str,
        module_path: Path,
        class_name: str,
        factory_kwargs: dict[str, Any] | None = None,
    ) -> None:
        self.feature_id = feature_id
        self.module_path = module_path
        self.class_name = class_name
        self.factory_kwargs = factory_kwargs or {}
        self.instance: Any | None = None
        self.downloaded = False
        self.applied = False
        self.error: str | None = None
        self.version: str | None = None
        self._loaded_mtime_ns: int | None = None

    def refresh(self) -> bool:
        before = self.status_tuple()
        if not self.module_path.exists():
            self._dispose_instance()
            self.instance = None
            self.downloaded = False
            self.applied = False
            self.error = None
            self.version = None
            self._loaded_mtime_ns = None
            return before != self.status_tuple()

        self.downloaded = True
        mtime_ns = self.module_path.stat().st_mtime_ns
        if self.instance is not None and self.applied and self._loaded_mtime_ns == mtime_ns:
            return before != self.status_tuple()

        try:
            self._dispose_instance()
            module_name = f"vehicle_feature_{self.feature_id.lower()}_{mtime_ns}"
            spec = importlib.util.spec_from_file_location(module_name, self.module_path)
            if spec is None or spec.loader is None:
                raise RuntimeError("feature module spec could not be created")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            feature_class = getattr(module, self.class_name)
            raw_version = getattr(module, "VERSION", getattr(module, "__version__", None))
            self.instance = feature_class(**self.factory_kwargs)
            self.applied = True
            self.error = None
            self.version = str(raw_version) if raw_version else None
            self._loaded_mtime_ns = mtime_ns
            logger.info("%s feature module applied: %s", self.feature_id, self.module_path)
        except Exception as exc:
            self.instance = None
            self.applied = False
            self.error = f"{type(exc).__name__}: {exc}"
            self.version = None
            logger.exception("%s feature module failed to apply", self.feature_id)

        return before != self.status_tuple()

    def _dispose_instance(self) -> None:
        if self.instance is None:
            return
        for method_name in ("close", "shutdown", "stop"):
            method = getattr(self.instance, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception as exc:
                    logger.warning("%s feature %s failed: %s", self.feature_id, method_name, exc)
                break

    def update(
        self,
        joystick: JoystickSignal,
        *,
        enabled: bool,
        context: dict[str, Any],
    ) -> FeatureState:
        self.refresh()
        if not self.downloaded:
            return FeatureState(
                enabled=False,
                mode="not downloaded",
                value={},
                downloaded=False,
                applied=False,
            )
        if not self.applied or self.instance is None:
            return FeatureState(
                enabled=False,
                mode="not applied",
                value={},
                downloaded=True,
                applied=False,
                error=self.error,
            )

        try:
            result = self.instance.update(joystick=joystick, enabled=enabled, context=context)
            if not isinstance(result, dict):
                raise RuntimeError("feature update must return a dict")
        except Exception as exc:
            self._dispose_instance()
            self.applied = False
            self.error = f"{type(exc).__name__}: {exc}"
            logger.exception("%s feature update failed", self.feature_id)
            return FeatureState(
                enabled=False,
                mode="error",
                value={},
                downloaded=True,
                applied=False,
                error=self.error,
            )

        return FeatureState(
            enabled=bool(result.get("enabled", enabled and self.applied)),
            mode=str(result.get("mode", "active" if enabled else "standby")),
            value=dict(result.get("value", {})),
            downloaded=True,
            applied=True,
            error=None,
        )

    def status_tuple(self) -> tuple[bool, bool, str | None, str | None]:
        return (self.downloaded, self.applied, self.error, self.version)


class VehicleControl:
    def __init__(
        self,
        *,
        on_gear_change: Callable[[str], None] | None = None,
        sensor_provider: Callable[[], dict[str, Any]] | None = None,
        packet_sender: Callable[[bytes], None] | None = None,
        feature_state_store: Any | None = None,
        features_dir: str | os.PathLike[str] | None = None,
        interval_seconds: float = 0.02,
        joystick_button_p: int = JOYSTICK_BUTTON_P,
        joystick_button_d: int = JOYSTICK_BUTTON_D,
        joystick_axis_speed: int = JOYSTICK_AXIS_SPEED,
        joystick_axis_steer: int = JOYSTICK_AXIS_STEER,
        someip_service_id: int,
        someip_method_id: int,
        someip_client_id: int,
        aeb_service_id: int,
        aeb_control_method_id: int,
        aeb_trigger_event_id: int,
    ) -> None:
        self._lock = threading.RLock()
        self._joystick = JoystickReader(
            button_p=joystick_button_p,
            button_d=joystick_button_d,
            axis_speed=joystick_axis_speed,
            axis_steer=joystick_axis_steer,
        )
        self._feature_state_store = feature_state_store
        self._desired_enabled = {"LKAS": False, "FVSA": False, "AEB": False}
        self._sync_desired_enabled_from_store()
        self._features_dir = Path(features_dir) if features_dir else Path(__file__).resolve().parent / "features"
        self._reported_runtime_status: dict[str, tuple[bool, bool, str | None, str | None]] = {}
        self._lkas = DownloadedPythonFeature(
            feature_id="LKAS",
            module_path=self._features_dir / "LKAS.py",
            class_name="LKASFeature",
            factory_kwargs={
                "max_angle": 20.0,
                "angle_deadzone": 3.0,
                "smooth_frames": 5,
                "lane_smooth_frames": 10,
                "speed_byte_threshold": 117,
                "sensitivity": 2.5,
            },
        )
        self._fvsa = DownloadedPythonFeature(
            feature_id="FVSA",
            module_path=self._features_dir / "FVSA.py",
            class_name="FVSAFeature",
            factory_kwargs={
                "stop_time_threshold": 2.0,
                "distance_diff_threshold": 700.0,
                "tof_smooth_frames": 5,
            },
        )
        self._aeb = DownloadedPythonFeature(
            feature_id="AEB",
            module_path=self._features_dir / "AEB.py",
            class_name="AEBFeature",
            factory_kwargs={"alarm_seconds": 4.0},
        )
        self._on_gear_change = on_gear_change
        self._sensor_provider = sensor_provider
        self._packet_sender = packet_sender
        self._interval_seconds = interval_seconds
        self._last_gear = GEAR_P
        self._session_id = 1
        self._someip_service_id = someip_service_id
        self._someip_method_id = someip_method_id
        self._someip_client_id = someip_client_id
        self._aeb_service_id = aeb_service_id
        self._aeb_control_method_id = aeb_control_method_id
        self._aeb_trigger_event_id = aeb_trigger_event_id
        self._last_aeb_enabled_sent: bool | None = None

        neutral = JoystickSignal(False, GEAR_P, 0.0, 0.0, SPEED_CENTER_BYTE, STEER_CENTER_BYTE, "initial")
        neutral_lkas = self._update_lkas(neutral)
        neutral_aeb = self._update_aeb(neutral)
        neutral_payload = self.build_control_payload(neutral, neutral_lkas)
        self._snapshot = VehicleControlSnapshot(
            joystick=neutral,
            lkas=neutral_lkas,
            fvsa=self._update_fvsa(neutral),
            aeb=neutral_aeb,
            control_payload_hex=neutral_payload.hex(" "),
            control_packet_hex=self._build_someip_packet(neutral_payload).hex(" "),
            updated_at=utc_now(),
        )

    def _feature_context(self) -> dict[str, Any]:
        sensor_context = self._sensor_provider() if self._sensor_provider is not None else {}
        return {
            "gear_d": GEAR_D,
            "tof_distance_mm": sensor_context.get("tof_mm"),
            "speed_raw": sensor_context.get("speed_raw"),
            "speed_kmh": sensor_context.get("speed_kmh"),
            "steer_center_byte": STEER_CENTER_BYTE,
            "axis_to_byte": axis_to_byte,
            "angle_to_byte": angle_to_byte,
            "is_manual_steering": is_manual_steering,
        }

    def _sync_desired_enabled_from_store(self) -> None:
        if self._feature_state_store is None:
            return
        for feature_id in self._desired_enabled:
            try:
                self._desired_enabled[feature_id] = bool(
                    self._feature_state_store.is_feature_enabled(feature_id)
                )
            except Exception as exc:
                logger.warning("feature state load failed for %s: %s", feature_id, exc)
                self._desired_enabled[feature_id] = False

    def _feature_enabled(self, feature_id: str) -> bool:
        return bool(self._desired_enabled.get(feature_id, False))

    def reload_feature_state(self) -> None:
        with self._lock:
            self._sync_desired_enabled_from_store()

    def _set_feature_enabled(self, feature_id: str, enabled: bool) -> None:
        if self._feature_state_store is not None:
            self._feature_state_store.set_feature_enabled(feature_id, enabled)
        self._desired_enabled[feature_id] = bool(enabled)
        logger.info("%s %s", feature_id, "enabled" if enabled else "disabled")

    def _feature_can_run(self, feature: DownloadedPythonFeature) -> bool:
        feature.refresh()
        self._report_runtime_status(feature)
        return feature.downloaded and feature.applied

    def _report_runtime_status(self, feature: DownloadedPythonFeature) -> None:
        status = feature.status_tuple()
        if self._reported_runtime_status.get(feature.feature_id) == status:
            return
        self._reported_runtime_status[feature.feature_id] = status
        if self._feature_state_store is not None:
            self._feature_state_store.mark_feature_runtime(
                feature.feature_id,
                downloaded=feature.downloaded,
                applied=feature.applied,
                error=feature.error,
                version=feature.version,
            )

    def _update_lkas(self, joystick: JoystickSignal, context: dict[str, Any] | None = None) -> FeatureState:
        enabled = self._feature_enabled("LKAS")
        state = self._lkas.update(
            joystick,
            enabled=enabled,
            context=context if context is not None else self._feature_context(),
        )
        self._report_runtime_status(self._lkas)
        if enabled and (not state.downloaded or not state.applied):
            self._set_feature_enabled("LKAS", False)
        if state.value.get("disable_requested"):
            self._set_feature_enabled("LKAS", False)
            state = FeatureState(
                enabled=False,
                mode=state.mode,
                value={key: value for key, value in state.value.items() if key != "disable_requested"},
                downloaded=state.downloaded,
                applied=state.applied,
                error=state.error,
            )
        return state

    def _update_fvsa(self, joystick: JoystickSignal, context: dict[str, Any] | None = None) -> FeatureState:
        enabled = self._feature_enabled("FVSA")
        state = self._fvsa.update(
            joystick,
            enabled=enabled,
            context=context if context is not None else self._feature_context(),
        )
        self._report_runtime_status(self._fvsa)
        if enabled and (not state.downloaded or not state.applied):
            self._set_feature_enabled("FVSA", False)
        return state

    def _update_aeb(self, joystick: JoystickSignal, context: dict[str, Any] | None = None) -> FeatureState:
        enabled = self._feature_enabled("AEB")
        state = self._aeb.update(
            joystick,
            enabled=enabled,
            context=context if context is not None else self._feature_context(),
        )
        self._report_runtime_status(self._aeb)
        if enabled and (not state.downloaded or not state.applied):
            self._set_feature_enabled("AEB", False)
            enabled = False
        if self._last_aeb_enabled_sent is None:
            self._last_aeb_enabled_sent = enabled
            if enabled:
                self._send_aeb_enabled(enabled)
            return state
        if self._last_aeb_enabled_sent != enabled:
            self._send_aeb_enabled(enabled)
            self._last_aeb_enabled_sent = enabled
        return state

    def poll_once(self) -> VehicleControlSnapshot:
        joystick = self._joystick.read()
        with self._lock:
            if joystick.gear != self._last_gear:
                self._last_gear = joystick.gear
                if self._on_gear_change:
                    self._on_gear_change(joystick.gear)

            feature_context = self._feature_context()
            lkas = self._update_lkas(joystick, feature_context)
            fvsa = self._update_fvsa(joystick, feature_context)
            aeb = self._update_aeb(joystick, feature_context)
            control_payload, control_packet = self._build_next_control_packet(joystick, lkas)
            self._snapshot = VehicleControlSnapshot(
                joystick=joystick,
                lkas=lkas,
                fvsa=fvsa,
                aeb=aeb,
                control_payload_hex=control_payload.hex(" "),
                control_packet_hex=control_packet.hex(" "),
                updated_at=utc_now(),
            )
            return self._snapshot

    def set_lkas_enabled(self, enabled: bool) -> dict:
        with self._lock:
            if enabled and not self._feature_can_run(self._lkas):
                enabled = False
            self._set_feature_enabled("LKAS", enabled)
            lkas = self._update_lkas(self._snapshot.joystick)
            control_payload = self.build_control_payload(self._snapshot.joystick, lkas)
            self._snapshot = VehicleControlSnapshot(
                joystick=self._snapshot.joystick,
                lkas=lkas,
                fvsa=self._snapshot.fvsa,
                aeb=self._snapshot.aeb,
                control_payload_hex=control_payload.hex(" "),
                control_packet_hex=self._build_someip_packet(control_payload).hex(" "),
                updated_at=utc_now(),
            )
            return self.snapshot()

    def set_fvsa_enabled(self, enabled: bool) -> dict:
        with self._lock:
            if enabled and not self._feature_can_run(self._fvsa):
                enabled = False
            self._set_feature_enabled("FVSA", enabled)
            self._snapshot = VehicleControlSnapshot(
                joystick=self._snapshot.joystick,
                lkas=self._snapshot.lkas,
                fvsa=self._update_fvsa(self._snapshot.joystick),
                aeb=self._snapshot.aeb,
                control_payload_hex=self._snapshot.control_payload_hex,
                control_packet_hex=self._snapshot.control_packet_hex,
                updated_at=utc_now(),
            )
            return self.snapshot()

    def trigger_fvsa_buzzer(self) -> dict:
        with self._lock:
            if not self._feature_can_run(self._fvsa) or self._fvsa.instance is None:
                return {"ok": False, "control": self.snapshot(), "message": "FVSA is not applied"}

            if hasattr(self._fvsa.instance, "ring_buzzer"):
                self._fvsa.instance.ring_buzzer()
            elif hasattr(self._fvsa.instance, "_beep"):
                self._fvsa.instance._beep()
            else:
                return {"ok": False, "control": self.snapshot(), "message": "FVSA buzzer hook is unavailable"}

            self._snapshot = VehicleControlSnapshot(
                joystick=self._snapshot.joystick,
                lkas=self._snapshot.lkas,
                fvsa=self._update_fvsa(self._snapshot.joystick),
                aeb=self._snapshot.aeb,
                control_payload_hex=self._snapshot.control_payload_hex,
                control_packet_hex=self._snapshot.control_packet_hex,
                updated_at=utc_now(),
            )
            return {"ok": True, "control": self.snapshot(), "message": "FVSA buzzer triggered"}

    def set_aeb_enabled(self, enabled: bool) -> dict:
        with self._lock:
            if enabled and not self._feature_can_run(self._aeb):
                enabled = False
            self._set_feature_enabled("AEB", enabled)
            self._snapshot = VehicleControlSnapshot(
                joystick=self._snapshot.joystick,
                lkas=self._snapshot.lkas,
                fvsa=self._snapshot.fvsa,
                aeb=self._update_aeb(self._snapshot.joystick),
                control_payload_hex=self._snapshot.control_payload_hex,
                control_packet_hex=self._snapshot.control_packet_hex,
                updated_at=utc_now(),
            )
            return self.snapshot()

    def trigger_aeb(self) -> dict:
        with self._lock:
            if not self._feature_can_run(self._aeb) or not self._feature_enabled("AEB"):
                return self.snapshot()
            if self._aeb.instance is not None and hasattr(self._aeb.instance, "trigger"):
                self._aeb.instance.trigger()
            self._snapshot = VehicleControlSnapshot(
                joystick=self._snapshot.joystick,
                lkas=self._snapshot.lkas,
                fvsa=self._snapshot.fvsa,
                aeb=self._update_aeb(self._snapshot.joystick),
                control_payload_hex=self._snapshot.control_payload_hex,
                control_packet_hex=self._snapshot.control_packet_hex,
                updated_at=utc_now(),
            )
            return self.snapshot()

    def control_packet(self) -> bytes:
        with self._lock:
            return bytes.fromhex(self._snapshot.control_packet_hex)

    def snapshot(self) -> dict:
        with self._lock:
            return self._snapshot.to_dict()

    def run(self, stop_event: threading.Event, heartbeat: Callable[[], None]) -> None:
        logger.debug("ready")
        while not stop_event.is_set():
            snapshot = self.poll_once()
            packet = bytes.fromhex(snapshot.control_packet_hex)
            if self._packet_sender:
                self._packet_sender(packet)
            heartbeat()
            stop_event.wait(self._interval_seconds)
        logger.debug("stop requested")

    @staticmethod
    def build_control_payload(joystick: JoystickSignal, lkas: FeatureState | None = None) -> bytes:
        if joystick.gear == GEAR_P:
            return bytes([SPEED_CENTER_BYTE, STEER_CENTER_BYTE])

        steer_byte = joystick.steer_byte
        if lkas is not None and lkas.enabled:
            steer_byte = int(lkas.value.get("steer_byte", steer_byte))

        return bytes([
            clamp_byte(joystick.speed_byte),
            clamp_byte(steer_byte),
        ])

    @staticmethod
    def build_control_packet(
        joystick: JoystickSignal,
        *,
        service_id: int,
        method_id: int,
        client_id: int,
        session_id: int = 1,
        lkas: FeatureState | None = None,
    ) -> bytes:
        return build_someip_packet(
            VehicleControl.build_control_payload(joystick, lkas),
            service_id=service_id,
            method_id=method_id,
            client_id=client_id,
            session_id=session_id,
        )

    def _build_next_control_packet(
        self,
        joystick: JoystickSignal,
        lkas: FeatureState,
    ) -> tuple[bytes, bytes]:
        control_payload = self.build_control_payload(joystick, lkas)
        control_packet = self._build_someip_packet(control_payload)
        self._session_id += 1
        if self._session_id > 0xFFFF:
            self._session_id = 1
        return control_payload, control_packet

    def _build_someip_packet(self, payload: bytes) -> bytes:
        return build_someip_packet(
            payload,
            service_id=self._someip_service_id,
            method_id=self._someip_method_id,
            client_id=self._someip_client_id,
            session_id=self._session_id,
        )

    def _send_aeb_enabled(self, enabled: bool) -> None:
        self._send_aeb_packet(
            method_id=self._aeb_control_method_id,
            payload=bytes([0x01 if enabled else 0x00]),
        )

    def _send_aeb_packet(self, *, method_id: int, payload: bytes) -> None:
        packet = build_someip_packet(
            payload,
            service_id=self._aeb_service_id,
            method_id=method_id,
            client_id=self._someip_client_id,
            session_id=self._session_id,
        )
        self._session_id += 1
        if self._session_id > 0xFFFF:
            self._session_id = 1
        if self._packet_sender:
            self._packet_sender(packet)


def clamp_byte(value: int) -> int:
    return max(0, min(255, int(value)))


def is_manual_steering(joystick: JoystickSignal) -> bool:
    if joystick.gear != GEAR_D:
        return False
    if abs(joystick.axis_steer) > MANUAL_STEER_DEADZONE:
        return True
    return abs(joystick.steer_byte - STEER_CENTER_BYTE) > MANUAL_STEER_BYTE_DEADZONE
