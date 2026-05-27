from __future__ import annotations

import time
from typing import Any


class FVSAFeature:
    def __init__(
        self,
        *,
        stop_time_threshold: float,
        distance_diff_threshold: float,
    ) -> None:
        self.stop_time_threshold = stop_time_threshold
        self.distance_diff_threshold = distance_diff_threshold
        self._stopped_time: float | None = None
        self._stopped_distance_mm: float | None = None
        self._alert = False

    def update(self, *, joystick: Any, enabled: bool, context: dict[str, Any]) -> dict:
        tof_distance_mm = context.get("tof_distance_mm")
        if not enabled:
            self._reset()
            return self._state(enabled=False, mode="standby")

        if joystick.gear != context["gear_d"]:
            self._reset()
            return self._state(enabled=True, mode="standby")

        if abs(joystick.axis_speed) > 0.05:
            self._reset()
            return self._state(enabled=True, mode="moving")

        if self._stopped_time is None:
            self._stopped_time = time.time()
        if self._stopped_distance_mm is None and tof_distance_mm is not None:
            self._stopped_distance_mm = tof_distance_mm

        stopped_duration = time.time() - self._stopped_time
        if (
            stopped_duration >= self.stop_time_threshold
            and self._stopped_distance_mm is not None
            and tof_distance_mm is not None
            and tof_distance_mm - self._stopped_distance_mm >= self.distance_diff_threshold
        ):
            self._alert = True

        return self._state(enabled=True, mode="alert" if self._alert else "watching")

    def _state(self, *, enabled: bool, mode: str) -> dict:
        return {
            "enabled": enabled,
            "mode": mode,
            "value": {
                "alert": self._alert,
                "stopped_distance_mm": self._stopped_distance_mm,
            },
        }

    def _reset(self) -> None:
        self._stopped_time = None
        self._stopped_distance_mm = None
        self._alert = False
