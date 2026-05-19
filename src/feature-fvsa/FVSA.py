import os
os.environ["SDL_VIDEODRIVER"] = "dummy"

import pygame
import serial
import time
from collections import deque
import RPi.GPIO as GPIO


# ==========================================
# UART (Raspberry Pi ↔ ZCU)
# ==========================================
ser = serial.Serial(
    port='/dev/serial0',
    baudrate=9600,
    timeout=0.01
)


# ==========================================
# 차량 상태 설정
# ==========================================
NEUTRAL_SPEED = 127
STOP_DEADZONE = 10


# ==========================================
# FVSA 설정
# ==========================================
STOP_TIME_THRESHOLD = 2.0

# 앞차 거리 증가 기준(mm)
DISTANCE_DIFF_THRESHOLD = 700

# ToF smoothing
TOF_SMOOTH_FRAMES = 5


# ==========================================
# UART Frame
# ==========================================

# Raspberry Pi → ZCU
TX_START_BYTE = 0x3B
TX_END_BYTE   = 0x0D

# ZCU → Raspberry Pi
RX_START_BYTE = 0x3C
RX_END_BYTE   = 0x0D


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

    # 첫 번째 삐
    buzzer.ChangeDutyCycle(50)

    time.sleep(0.15)

    buzzer.ChangeDutyCycle(0)

    time.sleep(0.1)

    # 두 번째 삐
    buzzer.ChangeDutyCycle(50)

    time.sleep(0.15)

    buzzer.ChangeDutyCycle(0)


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
# 상태 변수
# ==========================================
tof_history = deque(maxlen=TOF_SMOOTH_FRAMES)

stopped_time = None

stopped_distance = None

fvsa_triggered = False

tof_distance_mm = None


# ==========================================
# UART RX Buffer
# ==========================================
rx_buffer = bytearray()


# ==========================================
# UART RX Parser
#
# ZCU → Raspberry Pi
#
# Frame:
# [0x3C][LOW][HIGH][0x0D]
# ==========================================
def read_tof_frame():

    global tof_distance_mm
    global rx_buffer

    try:

        waiting = ser.in_waiting

        if waiting > 0:

            data = ser.read(waiting)

            rx_buffer.extend(data)

        while len(rx_buffer) >= 4:

            # 시작 바이트 확인
            if rx_buffer[0] != RX_START_BYTE:

                rx_buffer.pop(0)

                continue

            # 종료 바이트 확인
            if rx_buffer[3] != RX_END_BYTE:

                rx_buffer.pop(0)

                continue

            # little endian
            low = rx_buffer[1]

            high = rx_buffer[2]

            distance = low | (high << 8)

            # 사용한 프레임 제거
            del rx_buffer[:4]

            # sanity check
            if 50 <= distance <= 10000:

                tof_history.append(distance)

                tof_distance_mm = int(
                    sum(tof_history)
                    / len(tof_history)
                )

                print(
                    f"[RX] ToF : "
                    f"{tof_distance_mm} mm"
                )

    except Exception as rx_error:

        print("RX Error :", rx_error)


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
        # 차량 이동 상태
        # ==================================
        is_moving = (
            abs(speed_byte - NEUTRAL_SPEED)
            > STOP_DEADZONE
        )

        # ==================================
        # ToF 수신
        # ==================================
        read_tof_frame()

        # ==================================
        # FVSA 로직
        # ==================================
        if not is_moving:

            # 정차 시작 시간 저장
            if stopped_time is None:

                stopped_time = time.time()

            stop_duration = (
                time.time()
                - stopped_time
            )

            # 정차 시작 거리 저장
            if (
                stopped_distance is None
                and tof_distance_mm is not None
            ):

                stopped_distance = tof_distance_mm

            # ==================================
            # 앞차 출발 감지
            # ==================================
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

                # 거리 증가 감지
                if (
                    dist_diff
                    > DISTANCE_DIFF_THRESHOLD
                ):

                    # 중복 알림 방지
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
        # Raspberry Pi → ZCU
        #
        # Frame:
        # [0x3B][speed][steer][0x0D]
        # ==================================
        packet = bytes([
            TX_START_BYTE,
            speed_byte,
            steer_byte,
            TX_END_BYTE
        ])

        try:

            ser.write(packet)

        except Exception as tx_error:

            print("TX Error :", tx_error)

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
            f"{'MOVING' if is_moving else 'STOPPED'}"
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
            "Packet :",
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