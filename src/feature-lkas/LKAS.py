import os
os.environ["SDL_VIDEODRIVER"] = "dummy"

import cv2
import numpy as np
import pygame
import serial
import time
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer


# ==========================================
# 설정값
# ==========================================
LKAS_ENABLED        = True
CAMERA_INDEX        = 0
IMAGE_SIZE          = (640, 360)
ROI_START_RATIO     = 0.55
CALIBRATION_OFFSET  = 0
MAX_ANGLE           = 20.0
SMOOTH_FRAMES       = 5
LANE_SMOOTH_FRAMES  = 10
STEER_DEADZONE      = 0.1
ANGLE_DEADZONE      = 3.0
SPEED_THRESHOLD     = 127
STREAM_PORT         = 8080
SENSITIVITY = 2.0  # 1.0이 기본, 높을수록 민감

# ==========================================
# 웹 스트리밍
# ==========================================
latest_frame = None
frame_lock   = threading.Lock()

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = b"""
            <html>
            <head>
                <title>LKAS Monitor</title>
                <style>
                    body { background: #111; color: #0f0;
                           font-family: monospace; text-align: center; }
                    img  { width: 640px; border: 2px solid #0f0; }
                </style>
                <meta http-equiv="refresh" content="0">
            </head>
            <body>
                <h2>LKAS Live Monitor</h2>
                <img src="/stream">
            </body>
            </html>
            """
            self.wfile.write(html)

        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        if latest_frame is None:
                            time.sleep(0.05)
                            continue
                        frame_bytes = latest_frame
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b"\r\n")
            except Exception:
                pass

    def log_message(self, format, *args):
        pass

def start_stream_server():
    server = HTTPServer(('0.0.0.0', STREAM_PORT), StreamHandler)
    print(f"Stream server: http://RaspberryPi_IP:{STREAM_PORT}")
    server.serve_forever()


# ==========================================
# UART 설정
# ==========================================
ser = serial.Serial(
    port='/dev/serial0',
    baudrate=9600,
    timeout=1
)


# ==========================================
# pygame 초기화
# ==========================================
pygame.init()
pygame.joystick.init()

while pygame.joystick.get_count() == 0:
    print("Waiting for gamepad...")
    time.sleep(1)
    pygame.joystick.quit()
    pygame.joystick.init()

js = pygame.joystick.Joystick(0)
js.init()
print("Gamepad connected:", js.get_name())


# ==========================================
# 카메라 초기화
# ==========================================
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IMAGE_SIZE[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_SIZE[1])
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Camera connection failed")
    exit()

print("Camera connected")


# ==========================================
# 변환 함수
# ==========================================
def axis_to_byte(axis_value):
    value = int((axis_value + 1.0) * 127.5)
    return max(0, min(255, value))

def angle_to_byte(angle_deg):
    normalized = angle_deg / MAX_ANGLE
    normalized = max(-1.0, min(1.0, normalized))
    return axis_to_byte(normalized)


# ==========================================
# Core 1: 영상 전처리
# ==========================================
def vision_preprocessing(frame):
    h, w  = frame.shape[:2]
    roi_y = int(h * ROI_START_RATIO)
    roi   = frame[roi_y:h, :]

    ycrcb     = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycrcb)

    clahe    = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    y_eq     = clahe.apply(y)

    ycrcb_eq = cv2.merge([y_eq, cr, cb])
    bgr_eq   = cv2.cvtColor(ycrcb_eq, cv2.COLOR_YCrCb2BGR)
    blurred  = cv2.GaussianBlur(bgr_eq, (3, 3), 0)

    return blurred, roi_y


# ==========================================
# Core 2: 마스크 생성
# ==========================================
def mask_generation(blurred):
    ycrcb     = cv2.cvtColor(blurred, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycrcb)

    _, cr_mask = cv2.threshold(cr, 140, 255, cv2.THRESH_BINARY)
    _, cb_mask = cv2.threshold(cb, 120, 255, cv2.THRESH_BINARY_INV)
    red_mask   = cv2.bitwise_and(cr_mask, cb_mask)

    _, y_mask  = cv2.threshold(y, 60, 255, cv2.THRESH_BINARY)
    red_mask   = cv2.bitwise_and(red_mask, y_mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    opened = cv2.morphologyEx(closed,   cv2.MORPH_OPEN,  kernel)

    return opened


# ==========================================
# Core 3: 윤곽선 검출 및 차선 중앙 계산
# ==========================================
def line_detection(mask, frame_w):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    left_xs,  left_ys  = [], []
    right_xs, right_ys = [], []
    mid_x = frame_w // 2

    for cnt in contours:
        if cv2.arcLength(cnt, False) < 10:
            continue
        for pt in cnt:
            x, y = pt[0]
            if x < mid_x:
                left_xs.append(x)
                left_ys.append(y)
            else:
                right_xs.append(x)
                right_ys.append(y)

    lane_center = _calc_lane_center(left_xs, right_xs)

    has_left  = len(left_xs)  >= 5
    has_right = len(right_xs) >= 5

    if has_left and has_right:
        status = "ok"
    else:
        status = "fail"

    if lane_center is not None and status == "ok":
        offset      = frame_w // 2 - lane_center - CALIBRATION_OFFSET
        steer_angle = -(offset / (frame_w // 2)) * MAX_ANGLE * SENSITIVITY
    else:
        offset      = 0
        steer_angle = None

    return steer_angle, status, offset, lane_center, left_xs, left_ys, right_xs, right_ys


def _calc_lane_center(left_xs, right_xs):
    left_x_mid  = int(np.median(left_xs))  if len(left_xs)  >= 5 else None
    right_x_mid = int(np.median(right_xs)) if len(right_xs) >= 5 else None

    if left_x_mid is not None and right_x_mid is not None:
        return (left_x_mid + right_x_mid) // 2
    elif left_x_mid is not None:
        return left_x_mid
    elif right_x_mid is not None:
        return right_x_mid
    return None


# ==========================================
# 시각화
# ==========================================
def make_vis_frame(frame, mask, roi_y, steer_angle, status, offset,
                   lane_center, left_xs, left_ys, right_xs, right_ys,
                   frame_w, frame_h, control_mode, fps):
    result     = frame.copy()
    car_center = frame_w // 2
    line_top   = roi_y
    line_bottom= frame_h

    # ROI 경계선
    cv2.line(result, (0, roi_y), (frame_w, roi_y), (255, 255, 0), 1)

    # ── 직선으로 차선 표시 (2차 곡선 제거) ──────────────
    for xs, ys in [(left_xs, left_ys), (right_xs, right_ys)]:
        if len(xs) >= 5:
            try:
                y_min  = min(ys)
                y_max  = max(ys)
                # y_min, y_max에 해당하는 x 평균
                x_top  = int(np.mean([x for x, y in zip(xs, ys) if y == y_min]))
                x_bot  = int(np.mean([x for x, y in zip(xs, ys) if y == y_max]))
                cv2.line(result,
                         (x_top, y_min + roi_y),
                         (x_bot, y_max + roi_y),
                         (0, 200, 255), 3)
            except Exception:
                pass

    # 차량 중앙 (파랑)
    cv2.line(result, (car_center, line_top), (car_center, line_bottom), (255, 0, 0), 2)
    cv2.putText(result, "CAR", (car_center + 5, line_top + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

    # 차선 중앙 (빨강)
    if lane_center is not None and status == "ok":
        cv2.line(result, (lane_center, line_top), (lane_center, line_bottom), (0, 0, 255), 2)
        cv2.putText(result, "LANE", (lane_center + 5, line_top + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        mid_y = (line_top + line_bottom) // 2
        cv2.arrowedLine(result, (car_center, mid_y), (lane_center, mid_y),
                        (0, 255, 255), 2, tipLength=0.2)
        cv2.putText(result, f"Offset: {offset:+d}px",
                    (min(car_center, lane_center) + 5, mid_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # 마스크 미리보기 (우상단)
    mh, mw       = 90, 120
    mask_preview = cv2.resize(mask, (mw, mh))
    mask_bgr     = cv2.cvtColor(mask_preview, cv2.COLOR_GRAY2BGR)
    result[10:10+mh, frame_w-mw-10:frame_w-10] = mask_bgr

    # 상태 텍스트
    color_map = {"ok": (0, 255, 0), "fail": (0, 0, 255)}
    cv2.putText(result, f"Angle: {steer_angle:+.1f} deg",
                (10, 25),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_map[status], 2)
    cv2.putText(result, f"Status: {status}",
                (10, 50),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_map[status], 1)
    cv2.putText(result, f"Mode: {control_mode}",
                (10, 72),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(result, f"FPS: {fps:.1f}",
                (10, 94),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return result


# ==========================================
# 스트림 서버 시작 (백그라운드)
# ==========================================
threading.Thread(target=start_stream_server, daemon=True).start()


# ==========================================
# 메인 루프
# ==========================================
angle_history       = deque(maxlen=SMOOTH_FRAMES)
lane_center_history = deque(maxlen=LANE_SMOOTH_FRAMES)
last_valid_angle    = 0.0
prev_time           = time.time()

while True:
    try:
        pygame.event.pump()

        axis_speed = js.get_axis(1)
        axis_steer = js.get_axis(2)
        speed_byte = axis_to_byte(axis_speed)

        is_moving       = speed_byte < SPEED_THRESHOLD
        manual_steering = abs(axis_steer) > STEER_DEADZONE

        # ── 카메라 + 차선 인식 (항상 실행) ──────────────
        ret, frame = cap.read()
        if ret:
            frame            = cv2.resize(frame, IMAGE_SIZE)
            frame_h, frame_w = frame.shape[:2]
            blurred, roi_y   = vision_preprocessing(frame)
            mask             = mask_generation(blurred)
            steer_angle, status, offset, lane_center, lxs, lys, rxs, rys = \
                line_detection(mask, frame_w)

            # 차선 중앙 smoothing
            if lane_center is not None and status == "ok":
                lane_center_history.append(lane_center)
            if lane_center_history:
                lane_center = int(np.mean(lane_center_history))
                offset      = frame_w // 2 - lane_center - CALIBRATION_OFFSET
                steer_angle = -(offset / (frame_w // 2)) * MAX_ANGLE * SENSITIVITY
        else:
            frame            = np.zeros((IMAGE_SIZE[1], IMAGE_SIZE[0], 3), dtype=np.uint8)
            frame_h, frame_w = IMAGE_SIZE[1], IMAGE_SIZE[0]
            roi_y            = frame_h // 2
            mask             = np.zeros((IMAGE_SIZE[1]//2, IMAGE_SIZE[0]), dtype=np.uint8)
            steer_angle      = None
            status           = "fail"
            offset           = 0
            lane_center      = None
            lxs, lys, rxs, rys = [], [], [], []

        # ── 조향값 결정 ──────────────────────────────────
        if manual_steering or not LKAS_ENABLED or not is_moving:
            steer_byte   = axis_to_byte(axis_steer)
            smooth_angle = axis_steer * MAX_ANGLE

            if manual_steering:
                control_mode = "MANUAL"
            elif not is_moving:
                control_mode = "STOPPED"
            else:
                control_mode = "MANUAL(LKAS OFF)"

        else:
            if status == "ok" and steer_angle is not None:
                steer_angle      = np.clip(steer_angle, -MAX_ANGLE, MAX_ANGLE)
                angle_history.append(steer_angle)
                last_valid_angle = steer_angle
            else:
                steer_angle = last_valid_angle

            smooth_angle = float(np.mean(angle_history)) if angle_history else 0.0

            if abs(smooth_angle) < ANGLE_DEADZONE:
                smooth_angle = 0.0

            steer_byte   = angle_to_byte(smooth_angle)
            control_mode = f"LKAS ({status})"

        # ── UART 패킷 전송 ────────────────────────────────
        packet = bytes([0x3B, speed_byte, steer_byte, 0x0D])
        ser.write(packet)

        # ── FPS 계산 ─────────────────────────────────────
        cur_time  = time.time()
        fps       = 1.0 / (cur_time - prev_time + 1e-9)
        prev_time = cur_time

        # ── 시각화 프레임 → 스트리밍 ─────────────────────
        vis = make_vis_frame(frame, mask, roi_y, smooth_angle, status, offset,
                             lane_center, lxs, lys, rxs, rys,
                             frame_w, frame_h, control_mode, fps)
        _, jpeg = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 50])
        with frame_lock:
            latest_frame = jpeg.tobytes()

        # ── 디버깅 출력 ───────────────────────────────────
        print(f"[{control_mode}] Angle: {smooth_angle:+.1f} | "
              f"Speed: {speed_byte} | Steer: {steer_byte} | FPS: {fps:.1f}")
        print(f"Packet: {packet.hex().upper()}")
        print("--------------------------------")

    except Exception as e:
        print("Error:", e)
        time.sleep(1)