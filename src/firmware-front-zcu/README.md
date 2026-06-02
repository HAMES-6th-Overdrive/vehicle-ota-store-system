# Firmware Front ZCU

## 1. 프로젝트 개요

`firmware-front-zcu`는 TC375 기반 Front ZCU(Zone Control Unit) 펌웨어 프로젝트입니다.
Front ZCU는 Vehicle Computer(HPC)와 Ethernet/SOME-IP/DoIP로 통신하고, Motor ECU 및 Sensor ECU와 CAN/CAN FD로 통신합니다.

본 프로젝트에서 Front ZCU는 다음 역할을 수행합니다.

* HPC로부터 주행 제어 명령 수신
* Sensor ECU로부터 TOF 거리값 및 차량 속도 수신
* AEB 판단 및 stopCmd 생성
* Motor ECU로 차량 제어 명령 송신
* Sensor ECU CAN FD OTA Gateway 수행
* ZCU Local UDS/DoIP 및 SOTA 구조 지원
* ECU 버전 정보 요청/응답 관리

---

## 2. 전체 시스템 구조

```text
Vehicle Computer / HPC
        |
        | Ethernet
        | - SOME/IP
        | - DoIP / UDS
        v
Front ZCU
        |
        | CAN / CAN FD
        v
Motor ECU / Sensor ECU
```

Front ZCU는 차량 네트워크의 중앙 제어 역할을 수행합니다.
HPC와는 Ethernet 기반 서비스 통신을 수행하고, 하위 ECU들과는 CAN/CAN FD를 통해 제어 명령, 센서 데이터, OTA 메시지를 주고받습니다.

---

## 3. 주요 기능

### 3.1 Drive Service

Drive Service는 HPC에서 전달된 주행 명령을 수신하고, Motor ECU로 차량 제어 명령을 송신합니다.

주요 기능은 다음과 같습니다.

* HPC로부터 drive command 수신
* HPC로부터 steering command 수신
* AEB Service에서 계산한 stopCmd 반영
* Motor ECU로 `VehicleControlCmd` 송신

```text
ZCU → Motor ECU
CAN ID: 0x100
Message: VehicleControlCmd
Payload:
  B0: driveCmd
  B1: steeringCmd
  B2: stopCmd
```

Drive Service는 주기적으로 최신 drive/steering 명령과 AEB stopCmd를 조합하여 Motor ECU에 전달합니다.

---

### 3.2 AEB Service

AEB Service는 Sensor ECU에서 수신한 TOF 거리값과 차량 속도값을 기반으로 긴급 제동 여부를 판단합니다.

입력 데이터는 다음과 같습니다.

```text
Sensor ECU → ZCU
CAN ID: 0x201
Message: TofDistanceData

Sensor ECU → ZCU
CAN ID: 0x202
Message: SpeedData
```

AEB Service는 거리와 속도 기반으로 stop distance를 계산하고, 위험 상황에서는 stopCmd를 `STOP` 상태로 변경합니다.
Drive Service는 이 stopCmd를 `0x100 VehicleControlCmd`에 포함하여 Motor ECU로 전달합니다.

---

### 3.3 Sensor Service

Sensor Service는 Sensor ECU에서 전달된 센서 데이터를 수신하고, 필요한 경우 SOME/IP Event 형태로 Vehicle Computer에 전달합니다.

주요 기능은 다음과 같습니다.

* `0x201 TofDistanceData` 수신
* `0x202 SpeedData` 수신
* Sensor Service SOME/IP Event 송신
* SpeedData는 일정 주기 기반으로 Vehicle Computer에 전달

---

### 3.4 Info Service

Info Service는 각 ECU의 버전 정보를 관리합니다.

CAN 기반 버전 요청/응답 구조는 다음과 같습니다.

| CAN ID  | 방향               | 메시지                | 설명               |
| ------- | ---------------- | ------------------ | ---------------- |
| `0x700` | ZCU → ECU        | Version Request    | ECU 버전 요청        |
| `0x701` | ZCU → Bus        | Front ZCU Version  | ZCU 버전 응답        |
| `0x702` | Drive ECU → ZCU  | Drive ECU Version  | Drive ECU 버전 응답  |
| `0x703` | Sensor ECU → ZCU | Sensor ECU Version | Sensor ECU 버전 응답 |

ZCU는 `0x700 Version Request`를 송신하고, 각 ECU의 버전 응답을 수신하여 Vehicle Computer의 Info 요청에 응답합니다.

---

## 4. CAN / CAN FD Interface

| CAN ID  | 방향               | 메시지명               | 프레임           | 설명                          |
| ------- | ---------------- | ------------------ | ------------- | --------------------------- |
| `0x100` | ZCU → Motor ECU  | VehicleControlCmd  | Classical CAN | 주행/조향/stopCmd 제어 명령         |
| `0x201` | Sensor ECU → ZCU | TofDistanceData    | Classical CAN | TOF 거리 데이터                  |
| `0x202` | Sensor ECU → ZCU | SpeedData          | Classical CAN | Hall Sensor 기반 차량 속도        |
| `0x600` | ZCU → Sensor ECU | OtaRequest         | CAN FD        | Sensor ECU OTA UDS Request  |
| `0x601` | Sensor ECU → ZCU | OtaResponse        | CAN FD        | Sensor ECU OTA UDS Response |
| `0x700` | ZCU → ECU        | Version Request    | Classical CAN | ECU 버전 요청                   |
| `0x701` | ZCU → Bus        | Front ZCU Version  | Classical CAN | ZCU 버전 응답                   |
| `0x702` | Drive ECU → ZCU  | Drive ECU Version  | Classical CAN | Drive ECU 버전 응답             |
| `0x703` | Sensor ECU → ZCU | Sensor ECU Version | Classical CAN | Sensor ECU 버전 응답            |

OTA 관련 메시지는 CAN FD를 사용하며, 일반 제어 및 센서 메시지는 Classical CAN을 사용합니다.

---

## 5. SOME/IP Service

Front ZCU는 Vehicle Computer와 SOME/IP 기반 서비스 통신을 수행합니다.

| Service ID | Service        | 역할            |
| ---------- | -------------- | ------------- |
| `0x0001`   | Drive Service  | 주행/조향 명령 수신   |
| `0x0002`   | Sensor Service | 센서 데이터 이벤트 전달 |
| `0x0006`   | AEB Service    | AEB 상태/제어     |
| `0x0007`   | Info Service   | ECU 버전 정보 제공  |

기본 SOME/IP 포트는 `30500`이며, Front ZCU의 Client ID는 `0x0002`입니다.

---

## 6. Sensor ECU CAN OTA Gateway

Front ZCU는 Sensor ECU OTA Gateway 역할을 수행합니다.

```text
HPC / Vehicle Computer
        |
        | TCP / DoIP / UDS
        v
Front ZCU
        |
        | CAN FD / UDS
        v
Sensor ECU
```

Sensor ECU OTA Gateway는 ZCU Local OTA가 아니라, Sensor ECU 업데이트를 중계하기 위한 입력 경로입니다.

### 6.1 HPC → ZCU

HPC는 TCP/DoIP로 ZCU에 접속합니다.

```text
HPC → ZCU
Port: 13401
Protocol: DoIP
Payload: UDS
```

ZCU는 다음 DoIP 메시지를 처리합니다.

| Payload Type | 설명                          |
| ------------ | --------------------------- |
| `0x0005`     | Routing Activation Request  |
| `0x0006`     | Routing Activation Response |
| `0x8001`     | Diagnostic Message          |
| `0x8002`     | Diagnostic Message ACK      |

DoIP Diagnostic Message 안의 UDS payload는 Sensor ECU OTA Gateway UDS adapter로 전달됩니다.

### 6.2 ZCU 내부 OTA Gateway 경로

```text
App_SensorOtaGateway_Doip
        ↓
App_SensorOtaGateway_Uds
        ↓
App_OtaReceiver
        ↓
App_OtaGateway
        ↓
UdsOtaClient
        ↓
App_Can
        ↓
Sensor ECU
```

ZCU는 전체 firmware binary를 저장하지 않습니다.
대신 현재 요청된 block만 받아서 Sensor ECU로 전달하는 streaming gateway 구조를 사용합니다.

---

## 7. Sensor ECU OTA UDS Flow

ZCU는 Sensor ECU에 대해 CAN FD 기반 UDS OTA Client로 동작합니다.

```text
ZCU → Sensor ECU : CAN FD 0x600 UDS Request
Sensor ECU → ZCU : CAN FD 0x601 UDS Response
```

지원하는 OTA 흐름은 다음과 같습니다.

```text
1. 0x10 DiagnosticSessionControl
2. 0x34 RequestDownload
3. 0x36 TransferData
4. 0x37 RequestTransferExit
5. 0x31 RoutineControl CRC32
6. 필요 시 0x11 ECUReset
```

Sparse OTA에서는 segment 단위 다운로드 흐름을 지원합니다.

```text
0x10 DiagnosticSessionControl

Segment 0:
  0x34 RequestDownload(offset / size)
  0x36 TransferData 반복
  0x37 RequestTransferExit

Segment 1:
  0x34 RequestDownload(offset / size)
  0x36 TransferData 반복
  0x37 RequestTransferExit

0x31 RoutineControl CRC32
필요 시 0x11 ECUReset
```

---

## 8. Sparse OTA

Sparse OTA는 전체 BIN 파일을 통째로 전송하지 않고, 실제 데이터가 존재하는 segment만 전송하는 방식입니다.

기존 방식은 Application Slot 전체를 하나의 BIN처럼 전송하기 때문에, 중간의 빈 Address Gap까지 함께 전송되어 OTA 시간이 길어질 수 있습니다.

Sparse OTA에서는 다음 정보를 Manifest로 관리합니다.

* Virtual Size
* Virtual CRC32
* Segment Count
* Gap Fill 값
* Segment Offset
* Segment Size

ZCU는 Sparse Manifest를 기반으로 Sensor ECU에 segment별 `RequestDownload`를 수행하고, 각 segment의 data block을 순서대로 전송합니다.

```text
Full BIN OTA:
[ 전체 Slot 이미지 전송 ]

Sparse OTA:
[ Segment 1 Data ] + [ Segment 2 Data ] + [ Metadata / Manifest ]
```

ZCU의 OTA Client는 sparse mode에서 실제 CAN으로 전송할 payload 총량을 segment size 합으로 관리하고, CRC는 virtual image CRC를 사용합니다.

---

## 9. ZCU Local DoIP / UDS

Front ZCU는 Sensor ECU OTA Gateway와 별개로 ZCU Local UDS/DoIP 경로도 가지고 있습니다.

```text
HPC
        |
        | DoIP / UDS
        v
Front ZCU
        |
        | Core1 Diagnostic Worker
        v
App_Uds
```

ZCU Local UDS 요청은 Core1 diagnostic worker로 전달됩니다.
Core1은 UDS 요청을 처리하고, 응답을 Core0 DoIP 경로를 통해 HPC로 반환합니다.

지원하는 기본 UDS 서비스는 다음과 같습니다.

| SID    | Service                  |
| ------ | ------------------------ |
| `0x10` | DiagnosticSessionControl |
| `0x34` | RequestDownload          |
| `0x36` | TransferData             |
| `0x37` | RequestTransferExit      |

---

## 10. Flash / SOTA

Front ZCU는 Flash write/erase 및 SOTA 관련 기능을 포함합니다.

### 10.1 Flash 처리

Flash erase/write는 PSPR로 복사한 wrapper 함수를 사용합니다.
erase/write command sequence 중 interrupt disable 구간을 짧게 유지하고, `waitUnbusy` 대기 구간에서는 interrupt를 복원하는 구조로 되어 있습니다.

이 구조는 Flash 작업 중 Ethernet/CAN interrupt가 장시간 막히는 문제를 줄이기 위한 설계입니다.

### 10.2 SOTA

SOTA 모듈은 UCB_SWAP 기반 A/B Bank 전환을 관리합니다.

주요 기능은 다음과 같습니다.

* SWAPEN 초기 설정
* Bank A marker 기록
* Bank B marker 기록
* UCB_SWAP ORIG/COPY 이중 관리
* Swap entry 검증
* Group A/B active 상태 확인
* reset을 통한 active bank 전환

---

## 11. 멀티코어 구조

본 프로젝트는 TC375의 멀티코어 구조를 활용합니다.

| Core  | 역할                                                                                        |
| ----- | ----------------------------------------------------------------------------------------- |
| Core0 | FreeRTOS 기반 메인 애플리케이션, CAN, SOME/IP, Ethernet, OTA Gateway, AEB/Drive/Sensor/Info Service |
| Core1 | Diagnostic Worker, ZCU Local UDS 처리, Vehicle Service                                      |
| Core2 | 현재 별도 기능 없음                                                                               |

Core0는 FreeRTOS scheduler를 실행하고, 대부분의 서비스 task를 관리합니다.
Core1은 UDS diagnostic 요청을 처리하는 worker 역할을 수행합니다.

---

## 12. 주요 Task 구성

| Task               | 주기/역할                                |
| ------------------ | ------------------------------------ |
| APP LED            | 500ms LED toggle                     |
| APP CAN            | CAN/CAN FD 송수신 처리                    |
| APP DRIVE SERVICE  | HPC 주행 명령 처리 및 Motor ECU 제어 명령 송신    |
| APP AEB SERVICE    | 센서값 기반 AEB 판단                        |
| APP SENSOR SERVICE | Sensor ECU 데이터 수신 및 SOME/IP Event 송신 |
| APP INFO SERVICE   | ECU 버전 정보 관리                         |
| APP SOMEIP         | SOME/IP 송수신                          |
| APP ETH            | lwIP/Ethernet 처리                     |
| APP OTA GATEWAY    | Sensor ECU OTA 상태머신 및 CAN FD UDS 중계  |
| SENSOR OTA DOIP    | Sensor ECU OTA DoIP server 초기화       |

---

## 13. OTA Package / Segment 생성

Repository에는 OTA segment 생성을 위한 `make_segments.ps1`과 OTA package 파일이 포함됩니다.

```text
make_segments.ps1
ota_segments/
firmware-front-zcu_ota_package.zip
```

Sparse OTA에서는 HEX/BIN 분석 결과를 기반으로 실제 데이터가 있는 segment와 metadata를 구성합니다.

---

## 14. 핵심 특징

* TC375 기반 Front ZCU 펌웨어
* FreeRTOS 기반 task 구조
* Ethernet / lwIP 기반 SOME/IP 통신
* DoIP / UDS 기반 진단 및 OTA 입력
* CAN / CAN FD 기반 ECU 통신
* ZCU 중심 AEB 판단 및 stopCmd 생성
* Sensor ECU CAN FD OTA Gateway
* Streaming OTA 구조
* Sparse Segment OTA 지원
* Flash PSPR 실행 wrapper 적용
* SOTA UCB_SWAP 기반 A/B Bank 전환 지원
* ECU version request/response 관리

---

## 15. 요약

Front ZCU는 HPC와 하위 ECU 사이에서 차량 제어, 센서 데이터 수집, AEB 판단, 버전 관리, OTA Gateway 기능을 수행하는 중앙 제어 ECU입니다.

일반 주행 중에는 HPC로부터 주행/조향 명령을 받아 Motor ECU로 전달하고, Sensor ECU의 TOF/Speed 데이터를 기반으로 AEB stopCmd를 판단합니다.

OTA 수행 시에는 HPC로부터 DoIP/UDS 기반 요청을 수신하고, Sensor ECU와 CAN FD UDS 통신을 수행하여 Sensor ECU OTA를 중계합니다. 또한 Sparse OTA와 SOTA 구조를 통해 전송량을 줄이고 안전한 펌웨어 전환을 지원합니다.
