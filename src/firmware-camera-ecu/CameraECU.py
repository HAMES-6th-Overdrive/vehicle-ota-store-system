import cv2
import socket
import time

# ==========================================
# 설정
# ==========================================

HPC_IP   = "192.168.10.1"   # HPC 라즈베리파이 IP로 변경
HPC_PORT = 30500

CAMERA_INDEX = 0

FRAME_WIDTH  = 640
FRAME_HEIGHT = 360

FPS = 30                     # 송신 주기 (20~30 추천)
JPEG_QUALITY = 50            # 40~60 추천

# ==========================================
# UDP Socket
# ==========================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==========================================
# Camera 초기화
# ==========================================

cap = cv2.VideoCapture(CAMERA_INDEX)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    print("Camera open failed")
    exit()

print("Camera ECU started")

# ==========================================
# Main Loop
# ==========================================

frame_interval = 1.0 / FPS
prev_time = time.time()

while True:

    try:
        ret, frame = cap.read()

        if not ret:
            continue

        # ----------------------------------
        # 1. Resize (안정성 확보)
        # ----------------------------------
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

        # ----------------------------------
        # 2. JPEG 압축
        # ----------------------------------
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)

        # ----------------------------------
        # 3. UDP 전송
        # ----------------------------------
        sock.sendto(buffer.tobytes(), (HPC_IP, HPC_PORT))

        # ----------------------------------
        # 4. FPS 제한
        # ----------------------------------
        elapsed = time.time() - prev_time
        sleep_time = frame_interval - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)

        prev_time = time.time()

        # ----------------------------------
        # Debug
        # ----------------------------------
        print(f"Sent frame | size: {len(buffer)} bytes")

    except Exception as e:
        print("Error:", e)
        time.sleep(0.5)