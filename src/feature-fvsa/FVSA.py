from __future__ import annotations

import importlib
import os
import threading
import time
from collections import deque
from typing import Any


class FVSAFeature:
    def __init__(
        self,
        *,
        stop_time_threshold: float,
        distance_diff_threshold: float,
        tof_smooth_frames: int = 5,
        min_valid_distance_mm: int = 50,
        max_valid_distance_mm: int = 10000,
        stopped_speed_kmh: float = 0.5,
        buzzer_pin: int | None = None,
        buzzer_enabled: bool | None = None,
    ) -> None:
        self.stop_time_threshold = stop_time_threshold
        self.distance_diff_threshold = distance_diff_threshold
        self.min_valid_distance_mm = min_valid_distance_mm
        self.max_valid_distance_mm = max_valid_distance_mm
        self.stopped_speed_kmh = stopped_speed_kmh
        self.buzzer_pin = int(buzzer_pin or os.getenv("VEHICLE_FVSA_BUZZER_PIN", "18"))
        self.buzzer_enabled = (
            os.getenv("VEHICLE_FVSA_BUZZER_ENABLED", "1") != "0"
            if buzzer_enabled is None
            else bool(buzzer_enabled)
        )
        self._tof_history: deque[int] = deque(maxlen=tof_smooth_frames)
        self._tof_distance_mm: int | None = None
        self._stopped_time: float | None = None
        self._stopped_distance_mm: int | None = None
        self._alert = False
        self._gpio: Any | None = None
        self._buzzer: Any | None = None
        self._buzzer_error: str | None = None
        self._beep_running = False

    def update(self, *, joystick: Any, enabled: bool, context: dict[str, Any]) -> dict:
        self._apply_tof(context.get("tof_distance_mm"))

        if not enabled:
            self._reset_stop_state()
            return self._state(enabled=False, mode="standby")

        if joystick.gear != context["gear_d"]:
            self._reset_stop_state()
            return self._state(enabled=True, mode="standby")

        if self._is_moving(joystick, context):
            self._reset_stop_state()
            return self._state(enabled=True, mode="moving")

        if self._tof_distance_mm is None:
            return self._state(enabled=True, mode="waiting for tof")

        if self._stopped_time is None:
            self._stopped_time = time.time()
        if self._stopped_distance_mm is None:
            self._stopped_distance_mm = self._tof_distance_mm

        stopped_duration = time.time() - self._stopped_time
        distance_diff = self._tof_distance_mm - self._stopped_distance_mm
        if (
            stopped_duration > self.stop_time_threshold
            and distance_diff > self.distance_diff_threshold
        ):
            if not self._alert:
                self._beep()
                self._alert = True

        return self._state(enabled=True, mode="alert" if self._alert else "watching")

    def _apply_tof(self, distance: Any) -> None:
        if distance is None:
            return
        try:
            distance_mm = int(distance)
        except (TypeError, ValueError):
            return
        if not self.min_valid_distance_mm <= distance_mm <= self.max_valid_distance_mm:
            return

        self._tof_history.append(distance_mm)
        self._tof_distance_mm = int(sum(self._tof_history) / len(self._tof_history))

    def _is_moving(self, joystick: Any, context: dict[str, Any]) -> bool:
        speed_kmh = context.get("speed_kmh")
        if speed_kmh is not None:
            try:
                return float(speed_kmh) > self.stopped_speed_kmh
            except (TypeError, ValueError):
                pass

        speed_raw = context.get("speed_raw")
        if speed_raw is not None:
            try:
                return int(speed_raw) > 0
            except (TypeError, ValueError):
                pass

        return abs(getattr(joystick, "axis_speed", 0.0)) > 0.05

    def _state(self, *, enabled: bool, mode: str) -> dict:
        distance_diff = None
        if self._tof_distance_mm is not None and self._stopped_distance_mm is not None:
            distance_diff = self._tof_distance_mm - self._stopped_distance_mm

        return {
            "enabled": enabled,
            "mode": mode,
            "value": {
                "alert": self._alert,
                "tof_distance_mm": self._tof_distance_mm,
                "stopped_distance_mm": self._stopped_distance_mm,
                "distance_diff_mm": distance_diff,
                "stop_duration_seconds": (
                    round(time.time() - self._stopped_time, 3)
                    if self._stopped_time is not None
                    else 0.0
                ),
                "buzzer_error": self._buzzer_error,
            },
        }

    def _reset_stop_state(self) -> None:
        self._stopped_time = None
        self._stopped_distance_mm = None
        self._alert = False

    def _beep(self) -> None:
        if not self.buzzer_enabled or self._beep_running:
            return

        buzzer = self._buzzer_pwm()
        if buzzer is None:
            return

        def run() -> None:
            self._beep_running = True
            try:
                buzzer.ChangeDutyCycle(50)
                time.sleep(0.15)
                buzzer.ChangeDutyCycle(0)
                time.sleep(0.1)
                buzzer.ChangeDutyCycle(50)
                time.sleep(0.15)
                buzzer.ChangeDutyCycle(0)
            finally:
                self._beep_running = False

        threading.Thread(target=run, daemon=True).start()

    def _buzzer_pwm(self) -> Any | None:
        if self._buzzer is not None:
            return self._buzzer
        try:
            gpio = importlib.import_module("RPi.GPIO")
            gpio.setmode(gpio.BCM)
            gpio.setup(self.buzzer_pin, gpio.OUT)
            self._buzzer = gpio.PWM(self.buzzer_pin, 2000)
            self._buzzer.start(0)
            self._gpio = gpio
            self._buzzer_error = None
        except Exception as exc:
            self._buzzer_error = f"{type(exc).__name__}: {exc}"
            self._buzzer = None
        return self._buzzer
