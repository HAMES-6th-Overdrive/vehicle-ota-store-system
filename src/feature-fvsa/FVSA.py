import os
os.environ["SDL_VIDEODRIVER"] = "dummy"

import pygame
import socket
import struct
import time
from collections import deque
import RPi.GPIO as GPIO 
import threading


# ==========================================
# SOME/IP Ethernet 설정
# ==========================================

# Raspberry Pi → ZCU
ZCU_IP   = "192.168.10.2"
ZCU_PORT = 30500

# ZCU → Raspberry Pi
RPI_IP   = "192.168.10.1"
RPI_PORT = 30500

SERVICE_ID_TX = 0x0001  
METHOD_ID_TX  = 0x1001

# ToF RX
SERVICE_ID_RX = 0x0002
METHOD_ID_RX  = 0x2002

# Vehicle Speed RX
SERVICE_ID_SPEED = 0x0002
METHOD_ID_SPEED  = 0x2003

CLIENT_ID = 0x0001

PROTOCOL_VERSION = 0x01
INTERFACE_VERSION = 0x01

MSG_TYPE_REQUEST  = 0x00
MSG_TYPE_RESPONSE = 0x00

RETURN_CODE_OK = 0x00

session_id = 1

# ==========================================
# UDP Socket
# ==========================================

tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rx_sock.bind((RPI_IP, RPI_PORT))

rx_sock.setblocking(False)

print(f"Listening SOME/IP : {RPI_PORT}")

# ==========================================
# 차량 상태 설정
# ==========================================

vehicle_speed = 0

# ==========================================
# 기어 상태
# ==========================================

GEAR_P = "P"
GEAR_D = "D"

gear_state    = GEAR_P
prev_button_p = False
prev_button_d = False

# ==========================================
# FVSA 설정
# ==========================================

FVSA_ENABLED = True  # ← 추가

STOP_TIME_THRESHOLD = 2.0

DISTANCE_DIFF_THRESHOLD = 500

TOF_SMOOTH_FRAMES = 5

# ==========================================
# 부저 설정
# ==========================================

BUZZER_PIN = 18

GPIO.setmode(GPIO.BCM)

GPIO.setup(BUZZER_PIN, GPIO.OUT)

buzzer = GPIO.PWM(BUZZER_PIN, 2000)

buzzer.start(0)

# ==========================================
# 삐빅 알림음
# ==========================================

def beep():
    def _beep():
        buzzer.ChangeDutyCycle(100)
        time.sleep(0.15)
        buzzer.ChangeDutyCycle(0)
        time.sleep(0.1)
        buzzer.ChangeDutyCycle(100)
        time.sleep(0.15)
        buzzer.ChangeDutyCycle(0)
    
    threading.Thread(target=_beep, daemon=True).start()

# ==========================================
# pygame 초기화
# ==========================================

pygame.init()

pygame.joystick.init()

# ==========================================
# 게임패드 연결 대기
# ==========================================

while pygame.joystick.get_count() == 0:

    print("게임패드 연결 대기중...")

    time.sleep(1)

    pygame.joystick.quit()

    pygame.joystick.init()

# ==========================================
# 게임패드 연결
# ==========================================

js = pygame.joystick.Joystick(0)

js.init()

print("게임패드 연결됨 :", js.get_name())

# ==========================================
# Axis → Byte 변환
# ==========================================

def axis_to_byte(axis_value):

    value = int((axis_value + 1.0) * 127.5)

    if value < 0:
        value = 0

    if value > 255:
        value = 255

    return value

# ==========================================
# SOME/IP 생성
# ==========================================

def build_someip(service_id,
                 method_id,
                 client_id,
                 session_id,
                 payload):

    message_id = (service_id << 16) | method_id

    request_id = (client_id << 16) | session_id

    length = 8 + len(payload)

    header = struct.pack(
        "!IIIBBBB",
        message_id,
        length,
        request_id,
        PROTOCOL_VERSION,
        INTERFACE_VERSION,
        MSG_TYPE_REQUEST,
        RETURN_CODE_OK
    )

    return header + payload

# ==========================================
# SOME/IP 파싱
# ==========================================

def parse_someip(data):

    if len(data) < 16:
        return None

    (
        message_id,
        length,
        request_id,
        pv,
        iv,
        msg_type,
        ret_code
    ) = struct.unpack(
        "!IIIBBBB",
        data[:16]
    )

    service_id = (message_id >> 16) & 0xFFFF

    method_id = message_id & 0xFFFF

    payload = data[16:]

    return {
        "service_id": service_id,
        "method_id": method_id,
        "payload": payload
    }

# ==========================================
# 상태 변수
# ==========================================

tof_history = deque(maxlen=TOF_SMOOTH_FRAMES)

stopped_time = None

stopped_distance = None

fvsa_triggered = False

tof_distance_mm = None

# ==========================================
# SOME/IP RX
#
# Payload:
# [LOW][HIGH]
# ==========================================

def read_someip():

    global tof_distance_mm
    global vehicle_speed

    try:

        while True:

            data, addr = rx_sock.recvfrom(1024)

            parsed = parse_someip(data)

            if parsed is None:
                continue

            service_id = parsed["service_id"]
            method_id  = parsed["method_id"]
            payload    = parsed["payload"]

            # =========================
            # ToF
            # =========================

            if (
                service_id == SERVICE_ID_RX
                and method_id == METHOD_ID_RX
            ):

                if len(payload) < 2:
                    continue

                low  = payload[0]
                high = payload[1]

                distance = low | (high << 8)

                if 50 <= distance <= 10000:

                    tof_history.append(distance)

                    tof_distance_mm = int(
                        sum(tof_history)
                        / len(tof_history)
                    )

                    print(
                        f"[RX ToF] "
                        f"{tof_distance_mm} mm"
                    )

            # =========================
            # Vehicle Speed
            # =========================

            elif (
                service_id == SERVICE_ID_SPEED
                and method_id == METHOD_ID_SPEED
            ):

                if len(payload) < 2:
                    continue

                low  = payload[0]
                high = payload[1]

                vehicle_speed = low | (high << 8)

                print(
                    f"[RX Speed] "
                    f"{vehicle_speed}"
                )

    except BlockingIOError:

        pass

    except Exception as e:

        print("RX Error :", e)

# ==========================================
# 메인 루프
# ==========================================

while True:

    try:

        pygame.event.pump()

        # ==================================
        # 게임패드 입력
        # ==================================

        axis_speed = js.get_axis(1)

        axis_steer = js.get_axis(2)

        speed_byte = axis_to_byte(axis_speed)

        steer_byte = axis_to_byte(axis_steer)

        # ==================================
        # 기어 버튼 입력
        # ==================================

        button_p = js.get_button(0)
        button_d = js.get_button(1)

        if button_p and not prev_button_p:
            gear_state = GEAR_P
            print("\n==========\nGEAR -> P\n==========\n")

        if button_d and not prev_button_d:
            gear_state = GEAR_D
            print("\n==========\nGEAR -> D\n==========\n")

        prev_button_p = button_p
        prev_button_d = button_d

        # ==================================
        # P단이면 강제 중립
        # ==================================

        if gear_state == GEAR_P:
            speed_byte = 127
            steer_byte = 127

        # ==================================
        # 차량 이동 상태
        # ==================================

        is_moving = vehicle_speed > 0

        # ==================================
        # SOME/IP 수신
        # ==================================

        read_someip()

        # ==================================
        # FVSA 로직
        # ==================================
        if FVSA_ENABLED and gear_state == GEAR_D:
            
            if not is_moving:

                if stopped_time is None:

                        stopped_time = time.time()

                stop_duration = (
                    time.time()
                    - stopped_time
                )

                if (
                    stopped_distance is None
                    and tof_distance_mm is not None
                ):

                    stopped_distance = tof_distance_mm

                if (
                    stop_duration
                    > STOP_TIME_THRESHOLD
                    and stopped_distance is not None
                    and tof_distance_mm is not None
                ):

                    dist_diff = (
                        tof_distance_mm
                        - stopped_distance
                    )

                    if (
                        dist_diff
                        > DISTANCE_DIFF_THRESHOLD
                    ):

                        if not fvsa_triggered:

                            print("")
                            print("================================")
                            print(">>> FRONT VEHICLE MOVING <<<")
                            print("================================")
                            print("")

                            beep()

                            fvsa_triggered = True

            # ==================================
            # 차량 움직이면 초기화
            # ==================================

            else:

                stopped_time = None

                stopped_distance = None

                fvsa_triggered = False

        # ==================================
        # SOME/IP Payload 생성
        #
        # [speed][steer]
        # ==================================

        payload = bytes([
            speed_byte,
            steer_byte
        ])

        # ==================================
        # SOME/IP 패킷 생성
        # ==================================

        packet = build_someip(
            SERVICE_ID_TX,
            METHOD_ID_TX,
            CLIENT_ID,
            session_id,
            payload
        )

        # ==================================
        # Ethernet 송신
        # ==================================

        try:

            tx_sock.sendto(
                packet,
                (ZCU_IP, ZCU_PORT)
            )

        except Exception as tx_error:

            print("TX Error :", tx_error)

        # ==================================
        # Session 증가
        # ==================================

        session_id += 1

        if session_id > 0xFFFF:
            session_id = 1

        # ==================================
        # 디버깅 출력
        # ==================================

        print(
            f"Speed Axis : "
            f"{axis_speed:.3f} -> {speed_byte}"
        )

        print(
            f"Steer Axis : "
            f"{axis_steer:.3f} -> {steer_byte}"
        )

        print(
            f"Moving State : "
            f"{'MOVING' if is_moving else 'STOPPED'} "
            f"(Vehicle Speed : {vehicle_speed})"
        )

        print(
            f"Vehicle Speed : "
            f"{vehicle_speed}"
        )

        print(
            f"GEAR : "
            f"{gear_state}"
        )

        if tof_distance_mm is not None:

            print(
                f"Front Distance : "
                f"{tof_distance_mm} mm"
            )

        if (
            stopped_distance is not None
            and tof_distance_mm is not None
        ):

            diff = (
                tof_distance_mm
                - stopped_distance
            )

            print(
                f"Distance Diff : "
                f"{diff} mm"
            )

        print(
            f"FVSA Triggered : "
            f"{fvsa_triggered}"
        )

        print(
            "Payload :",
            payload.hex().upper()
        )

        print(
            "Packet  :",
            packet.hex().upper()
        )

        print("--------------------------------")

        time.sleep(0.1)

    except KeyboardInterrupt:

        break

    except Exception as e:

        print("오류 발생 :", e)

        time.sleep(1)

# ==========================================
# 종료 처리
# ==========================================

buzzer.stop()

GPIO.cleanup()