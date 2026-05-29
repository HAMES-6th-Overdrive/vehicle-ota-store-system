from __future__ import annotations

import importlib
import os
import socket
from collections import deque
from typing import Any


VERSION = "1.0.1"
__version__ = VERSION


class LKASFeature:
    def __init__(
        self,
        *,
        max_angle: float,
        angle_deadzone: float,
        smooth_frames: int,
        image_size: tuple[int, int] = (640, 360),
        roi_start_ratio: float = 0.5,
        calibration_offset: int = 0,
        lane_smooth_frames: int = 10,
        steer_deadzone: float = 0.1,
        speed_byte_threshold: int = 117,
        sensitivity: float = 3.0,
        video_bind_host: str | None = None,
        video_port: int | None = None,
        camera_source_host: str | None = None,
    ) -> None:
        self.max_angle = max_angle
        self.angle_deadzone = angle_deadzone
        self.image_size = image_size
        self.roi_start_ratio = roi_start_ratio
        self.calibration_offset = calibration_offset
        self.steer_deadzone = steer_deadzone
        self.speed_byte_threshold = speed_byte_threshold
        self.sensitivity = sensitivity
        self.video_bind_host = video_bind_host or os.getenv("VEHICLE_CAMERA_BIND_HOST", "0.0.0.0")
        self.video_port = int(video_port or os.getenv("VEHICLE_CAMERA_VIDEO_PORT", "30501"))
        self.camera_source_host = camera_source_host or os.getenv("VEHICLE_CAMERA_ECU_IP") or None
        self._angle_history = deque(maxlen=smooth_frames)
        self._lane_center_history = deque(maxlen=lane_smooth_frames)
        self._last_valid_angle = 0.0
        self._last_frame: Any | None = None
        self._socket: socket.socket | None = None
        self._socket_error: str | None = None
        self._cv2: Any | None = None
        self._np: Any | None = None
        self._vision_error: str | None = None

    def update(self, *, joystick: Any, enabled: bool, context: dict[str, Any]) -> dict:
        if not enabled:
            self._reset_tracking()
            return self._manual_state(joystick, enabled=False, mode="manual")
        if joystick.gear != context["gear_d"]:
            self._reset_tracking()
            return self._state("parked", context["steer_center_byte"], 0.0, status="parked")
        if self._is_manual_steering(joystick, context):
            return self._manual_state(joystick, enabled=True, mode="manual override")
        if joystick.speed_byte >= self.speed_byte_threshold:
            return self._manual_state(joystick, enabled=True, mode="speed hold")

        frame = self._read_camera_frame()
        if frame is None:
            return self._state(
                "waiting for camera",
                joystick.steer_byte,
                joystick.axis_steer * self.max_angle,
                status="no frame",
                error=self._socket_error or self._vision_error,
            )

        result = self._detect_lane(frame)
        if result["status"] == "ok" and result["steer_angle"] is not None:
            steer_angle = max(-self.max_angle, min(self.max_angle, result["steer_angle"]))
            self._last_valid_angle = steer_angle
        else:
            steer_angle = self._last_valid_angle
        self._angle_history.append(steer_angle)

        smooth_angle = float(sum(self._angle_history) / len(self._angle_history)) if self._angle_history else 0.0
        if abs(smooth_angle) < self.angle_deadzone:
            smooth_angle = 0.0

        steer_byte = context["angle_to_byte"](smooth_angle, self.max_angle)
        return self._state(
            f"LKAS ({result['status']})",
            steer_byte,
            smooth_angle,
            status=result["status"],
            lane_center=result.get("lane_center"),
            offset_px=result.get("offset_px"),
        )

    def _read_camera_frame(self) -> Any | None:
        cv2 = self._cv2_module()
        np = self._np_module()
        sock = self._video_socket()
        if cv2 is None or np is None or sock is None:
            return None

        latest = None
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError as exc:
                self._socket_error = f"{type(exc).__name__}: {exc}"
                break
            if self.camera_source_host and addr[0] != self.camera_source_host:
                continue
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                latest = cv2.resize(frame, self.image_size)
        if latest is not None:
            self._last_frame = latest
            return latest
        return self._last_frame

    def _detect_lane(self, frame: Any) -> dict[str, Any]:
        cv2 = self._cv2_module()
        np = self._np_module()
        if cv2 is None or np is None:
            return {"status": "error", "steer_angle": None, "error": self._vision_error}

        frame_h, frame_w = frame.shape[:2]
        roi_y = int(frame_h * self.roi_start_ratio)
        roi = frame[roi_y:frame_h, :]
        ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
        y, cr, cb = cv2.split(ycrcb)
        y_eq = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(y)
        bgr_eq = cv2.cvtColor(cv2.merge([y_eq, cr, cb]), cv2.COLOR_YCrCb2BGR)
        blurred = cv2.GaussianBlur(bgr_eq, (3, 3), 0)

        ycrcb = cv2.cvtColor(blurred, cv2.COLOR_BGR2YCrCb)
        y, cr, cb = cv2.split(ycrcb)
        _, cr_mask = cv2.threshold(cr, 140, 255, cv2.THRESH_BINARY)
        _, cb_mask = cv2.threshold(cb, 120, 255, cv2.THRESH_BINARY_INV)
        red_mask = cv2.bitwise_and(cr_mask, cb_mask)
        _, y_mask = cv2.threshold(y, 60, 255, cv2.THRESH_BINARY)
        red_mask = cv2.bitwise_and(red_mask, y_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        left_xs: list[int] = []
        right_xs: list[int] = []
        mid_x = frame_w // 2
        for contour in contours:
            if cv2.arcLength(contour, False) < 10:
                continue
            for point in contour:
                x = int(point[0][0])
                (left_xs if x < mid_x else right_xs).append(x)

        lane_center = self._lane_center(left_xs, right_xs)
        status = "ok" if len(left_xs) >= 5 and len(right_xs) >= 5 else "fail"
        if lane_center is not None:
            self._lane_center_history.append(lane_center)
        if self._lane_center_history:
            lane_center = int(sum(self._lane_center_history) / len(self._lane_center_history))
        if lane_center is None or status != "ok":
            return {"status": status, "steer_angle": None, "lane_center": lane_center, "offset_px": None}

        offset = frame_w // 2 - lane_center - self.calibration_offset
        steer_angle = -(offset / (frame_w // 2)) * self.max_angle * self.sensitivity
        return {"status": status, "steer_angle": float(steer_angle), "lane_center": lane_center, "offset_px": offset}

    def _lane_center(self, left_xs: list[int], right_xs: list[int]) -> int | None:
        np = self._np_module()
        if np is None:
            return None
        left_mid = int(np.median(left_xs)) if len(left_xs) >= 5 else None
        right_mid = int(np.median(right_xs)) if len(right_xs) >= 5 else None
        if left_mid is not None and right_mid is not None:
            return (left_mid + right_mid) // 2
        return left_mid if left_mid is not None else right_mid

    def _is_manual_steering(self, joystick: Any, context: dict[str, Any]) -> bool:
        return abs(getattr(joystick, "axis_steer", 0.0)) > self.steer_deadzone

    def _manual_state(self, joystick: Any, *, enabled: bool, mode: str) -> dict:
        return {
            "enabled": enabled,
            "mode": mode,
            "value": {
                "steer_byte": joystick.steer_byte,
                "steer_angle": round(joystick.axis_steer * self.max_angle, 3),
            },
        }

    def _state(
        self,
        mode: str,
        steer_byte: int,
        steer_angle: float,
        *,
        status: str,
        lane_center: int | None = None,
        offset_px: int | None = None,
        error: str | None = None,
    ) -> dict:
        value = {
            "steer_byte": int(steer_byte),
            "steer_angle": round(float(steer_angle), 3),
            "lane_status": status,
            "lane_center": lane_center,
            "offset_px": offset_px,
            "camera_source": self.camera_source_host,
            "video_port": self.video_port,
        }
        if error:
            value["error"] = error
        return {"enabled": True, "mode": mode, "value": value}

    def _video_socket(self) -> socket.socket | None:
        if self._socket is not None:
            return self._socket
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65535)
            sock.bind((self.video_bind_host, self.video_port))
            sock.setblocking(False)
            self._socket = sock
            self._socket_error = None
        except OSError as exc:
            self._socket_error = f"{type(exc).__name__}: {exc}"
            self._socket = None
        return self._socket

    def _cv2_module(self) -> Any | None:
        if self._cv2 is not None:
            return self._cv2
        try:
            self._cv2 = importlib.import_module("cv2")
            return self._cv2
        except Exception as exc:
            self._vision_error = f"OpenCV unavailable: {type(exc).__name__}: {exc}"
            return None

    def _np_module(self) -> Any | None:
        if self._np is not None:
            return self._np
        try:
            self._np = importlib.import_module("numpy")
            return self._np
        except Exception as exc:
            self._vision_error = f"NumPy unavailable: {type(exc).__name__}: {exc}"
            return None

    def _reset_tracking(self) -> None:
        self._angle_history.clear()
        self._lane_center_history.clear()
        self._last_valid_angle = 0.0
