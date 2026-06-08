# 스토어형 차량용 OTA 프로젝트 (Store-Type Vehicle OTA Project)

Vehicle Computer와 Front ZCU를 중심으로 SOME/IP 기반 Ethernet, CAN/CAN FD,  
DoIP/UDS OTA를 연동하여 차량 기능 스토어와 LKAS, FVSA, AEB를 통합한 SDV 프로젝트

---

## 팀원

<div align="center">
<table>
  <tr>
    <td align="center">
      <a href="https://github.com/Wangjaepil">
        <img src="https://github.com/Wangjaepil.png" width="100px;" alt="왕재필" />
        <br />
        <b>왕재필</b>
      </a>
      <br />
      팀장
      <br />
      CAN / CAN OTA / Sensor ECU / AEB
    </td>
    <td align="center">
      <a href="https://github.com/starryeev">
        <img src="https://github.com/starryeev.png" width="100px;" alt="김건우" />
        <br />
        <b>김건우</b>
      </a>
      <br />
      팀원
      <br />
      SOME/IP / Gateway / Vehicle Computer
    </td>
    <td align="center">
      <a href="https://github.com/Kim-Byunghyun">
        <img src="https://github.com/Kim-Byunghyun.png" width="100px;" alt="김병현" />
        <br />
        <b>김병현</b>
      </a>
      <br />
      팀원
      <br />
      Motor ECU / LKAS / FVSA
    </td>
    <td align="center">
      <a href="https://github.com/jaedong1">
        <img src="https://github.com/jaedong1.png" width="100px;" alt="김재동" />
        <br />
        <b>김재동</b>
      </a>
      <br />
      팀원
      <br />
      UDS / DoIP / Bootloader / SOTA
    </td>
    <td align="center">
      <a href="https://github.com/lsoyeon">
        <img src="https://github.com/lsoyeon.png" width="100px;" alt="이소연" />
        <br />
        <b>이소연</b>
      </a>
      <br />
      팀원
      <br />
      UDS Client / CAN OTA / Vehicle Computer
    </td>
  </tr>
</table>
</div>

---

## 기술 스택

<div align="center">

### 하드웨어

<p>
  <img src="https://img.shields.io/badge/TC375%20Lite%20Kit%20V2-005B95?style=for-the-badge" alt="TC375 Lite Kit V2" />
  <img src="https://img.shields.io/badge/Raspberry%20Pi%204-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white" alt="Raspberry Pi 4" />
  <img src="https://img.shields.io/badge/Camera%20Module-111827?style=for-the-badge" alt="Camera Module" />
  <img src="https://img.shields.io/badge/ToF%20Sensor-0F766E?style=for-the-badge" alt="ToF Sensor" />
  <img src="https://img.shields.io/badge/Hall%20Sensor-7C3AED?style=for-the-badge" alt="Hall Sensor" />
</p>

### 기술스택

<p>
  <img src="https://img.shields.io/badge/C-A8B9CC?style=for-the-badge&logo=c&logoColor=white" alt="C" />
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python" />
  <img src="https://img.shields.io/badge/HTML-E34F26?style=for-the-badge&logo=html5&logoColor=white" alt="HTML" />
  <img src="https://img.shields.io/badge/JavaScript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=111111" alt="JavaScript" />
  <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" alt="OpenCV" />
  <img src="https://img.shields.io/badge/FreeRTOS-111827?style=for-the-badge" alt="FreeRTOS" />
  <img src="https://img.shields.io/badge/Ethernet-0F766E?style=for-the-badge" alt="Ethernet" />
  <img src="https://img.shields.io/badge/SOME%2FIP-2563EB?style=for-the-badge" alt="SOME/IP" />
  <img src="https://img.shields.io/badge/DoIP-7C3AED?style=for-the-badge" alt="DoIP" />
  <img src="https://img.shields.io/badge/UDS-B91C1C?style=for-the-badge" alt="UDS" />
  <img src="https://img.shields.io/badge/CAN-374151?style=for-the-badge" alt="CAN" />
  <img src="https://img.shields.io/badge/CAN%20FD-111827?style=for-the-badge" alt="CAN FD" />
</p>

### 협업 & 도구

<p>
  <img src="https://img.shields.io/badge/GitHub%20Releases-181717?style=for-the-badge&logo=github&logoColor=white" alt="GitHub Releases" />
  <img src="https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=confluence&logoColor=white" alt="Confluence" />
  <img src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=jira&logoColor=white" alt="Jira" />
  <img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white" alt="GitHub" />
  <img src="https://img.shields.io/badge/VS%20Code-007ACC?style=for-the-badge&logo=visualstudiocode&logoColor=white" alt="VS Code" />
  <img src="https://img.shields.io/badge/AURIX%20Development%20Studio-005B95?style=for-the-badge" alt="AURIX Development Studio" />
  <img src="https://img.shields.io/badge/Codex-111827?style=for-the-badge&logo=openai&logoColor=white" alt="Codex" />
</p>

</div>

---

## 1. 프로젝트 소개

`스토어형 차량용 OTA 프로젝트`는 차량 기능을 소프트웨어 단위로 선택하고,
필요한 기능과 펌웨어를 OTA(Over-The-Air)로 업데이트할 수 있도록 구성한 SDV
기반 차량 소프트웨어 시스템입니다.

본 프로젝트는 차량 기능이 특정 하드웨어에 고정되지 않고, 기능 스토어와 OTA
흐름을 통해 확장되는 구조를 구현하는 데 초점을 두었습니다. Vehicle Computer는
웹 대시보드와 기능 스토어를 제공하고, Front ZCU는 Vehicle Computer와 하위 ECU
사이에서 SOME/IP 기반 Ethernet, DoIP/UDS, CAN, CAN FD 통신을 중계합니다.

사용자는 대시보드에서 LKAS, AEB, FVSA와 같은 기능의 구매/활성화 상태를 확인하고
필요한 기능을 적용할 수 있습니다. 시스템은 기능 상태, 차량 상태, ECU 버전 정보,
OTA 진행 상태를 UI에 표시하며, 업데이트가 필요한 경우 GitHub Releases 기반
패키지를 다운로드해 대상 ECU 업데이트 흐름으로 연결합니다.

<table>
  <tr>
    <td>
      핵심 흐름은 <strong>기능 선택 → 기능 활성화 → ECU 버전 확인 →
      OTA 패키지 확인 → UDS/DoIP 업데이트 → 차량 제어 연동</strong>입니다.
      <br /><br />
      차량 내부 통신은 SOME/IP 기반 Ethernet과 CAN/CAN FD를 함께 사용하며,
      OTA 진단 흐름은 UDS/DoIP, CAN FD, Sparse OTA, SOTA/Bootloader 구조를
      통해 처리합니다.
    </td>
  </tr>
</table>

## 2. 프로젝트 목표

- LKAS, AEB, FVSA 기능을 기능 스토어에서 선택하고 활성화할 수 있는 구조 구현
- Vehicle Computer에서 기능 상태, 차량 상태, ECU 버전, OTA 진행 상태를 통합 표시
- SOME/IP 기반 Ethernet과 CAN/CAN FD를 연계한 Vehicle Computer-Front ZCU-ECU 통신 구조 구현
- DoIP/UDS와 CAN FD 기반 UDS 요청/응답을 이용한 Sensor ECU OTA Gateway 흐름 구성
- Sparse OTA, CRC 검증, SOTA/Bootloader 전환을 고려한 안정적인 펌웨어 업데이트 구조 설계
- Joystick 입력을 주행/조향 명령으로 변환해 Motor ECU 및 차량 제어 흐름과 연동
- V-Model 흐름에 따라 요구사항 정의, 설계, 구현, 단위/통합/시스템/인수 테스트 수행

## 3. 주요 기능

| 기능 | 내용 |
| --- | --- |
| **차량 통신 시스템** | Vehicle Computer, Front ZCU, 하위 ECU 간 데이터 교환을 위해 SOME/IP 기반 Ethernet과 CAN/CAN FD를 함께 사용합니다.<br />제어 명령, 센서 데이터, ECU 버전 정보, OTA 요청/응답이 각 통신 계층을 통해 전달됩니다. |
| **차량 기능 스토어** | 대시보드 내부의 기능 스토어에서 LKAS, AEB, FVSA 기능의 구매/활성화 상태를 관리합니다.<br />각 기능은 Vehicle Computer의 상태 파일을 통해 활성화 여부와 패키지 적용 상태를 추적합니다. |
| **OTA 업데이트** | GitHub Releases 또는 로컬 패키지 정보를 기반으로 업데이트 대상을 확인하고 다운로드합니다.<br />UDS의 DiagnosticSessionControl, RequestDownload, TransferData, RequestTransferExit, ECUReset 흐름을 이용하며, Sensor ECU OTA는 Front ZCU를 거쳐 CAN FD 기반 UDS 요청/응답으로 중계됩니다. |
| **LKAS** | Camera ECU에서 전달된 영상 프레임을 OpenCV로 처리해 차선을 검출합니다.<br />차량 중심과 차선 중심의 오프셋을 계산해 조향 보정 값을 산출하고, 기어 상태, 속도 조건, 수동 조향 입력 등을 고려해 동작을 제한합니다. |
| **AEB** | Sensor ECU의 ToF 거리 데이터와 Hall Sensor 기반 속도 데이터를 활용해 전방 위험 상황을 판단합니다.<br />Front ZCU는 거리/속도 기반 판단 결과를 stopCmd에 반영하고, Motor ECU로 전달되는 차량 제어 명령에 포함합니다. |
| **FVSA** | 정차 상태에서 전방 차량과의 거리 변화를 ToF 데이터로 모니터링합니다.<br />정차 시간과 거리 변화 조건이 충족되면 전방 차량 출발 알림을 제공해 운전자 편의성을 높입니다. |
| **차량 제어 및 대시보드 UI** | Joystick 입력을 기어 P/D, 속도, 조향 값으로 해석하고 차량 제어 명령으로 변환합니다.<br />대시보드는 차량 속도, 네트워크 상태, 날씨, OTA 상태, 기능 활성화 상태를 시각화합니다. |

## 4. 시스템 소개

전체 시스템은 Vehicle Computer(HPC), Front ZCU, Sensor ECU, Motor ECU,
Camera ECU, OTA Bootloader, 그리고 기능 모듈(LKAS/AEB/FVSA)로 계층화됩니다.

Vehicle Computer는 사용자와 시스템이 만나는 진입점입니다. 웹 기반 Dashboard UI와
기능 스토어를 제공하고, OTA Manager를 통해 릴리즈 확인, 펌웨어 다운로드,
업데이트 진행 상태 표시를 수행합니다. 또한 차량 제어 및 기능 이벤트를 SOME/IP
메시지로 구성해 Front ZCU로 전달합니다.

Front ZCU는 Vehicle Computer와 하위 ECU 사이의 Gateway이자 실시간 제어 판단
계층입니다. Ethernet 구간에서는 SOME/IP 및 DoIP/UDS 통신을 수행하고, 하위
제어 버스에서는 CAN/CAN FD Gateway로 동작합니다. Sensor ECU로부터 수신한
거리/속도 데이터는 AEB 판단에 활용되며, Vehicle Computer의 OTA 요청은 Sensor
ECU로 중계됩니다.

Sensor ECU는 ToF 센서와 Hall Sensor를 통해 거리 및 속도 데이터를 수집합니다.
이 데이터는 Front ZCU의 AEB 판단과 FVSA 기능의 입력으로 사용됩니다. OTA 상황에서는
CAN FD 기반 UDS 메시지를 통해 펌웨어 데이터를 수신하고, Sparse OTA 및 SOTA/
Bootloader 구조를 이용해 신규 애플리케이션 전환을 수행합니다.

Motor ECU는 상위 계층에서 전달된 차량 제어 명령을 실제 차량 제어 보드가 이해할
수 있는 UART Serial Packet으로 변환합니다. 이를 통해 조이스틱 입력, 주행/정지,
조향 제어가 실제 차량 동작으로 이어집니다.

Camera ECU는 카메라 프레임을 Vehicle Computer로 전달하고, LKAS는 영상 전처리,
차선 마스크 생성, 차선 검출, 조향각 계산 순서로 동작합니다. FVSA와 AEB는 Sensor
ECU의 거리/속도 데이터를 기반으로 운전자 편의 및 안전 기능을 제공합니다.

<p align="center">
  <img src="./docs/system-architecture.png" width="90%" alt="전체 시스템 구조도" />
</p>

```text
Vehicle Computer / HPC
  |-- Dashboard UI / Feature Store
  |-- OTA Manager
  |-- UDS/DoIP Client
  |-- SOME/IP Vehicle Control
        |
        | Ethernet: SOME/IP, DoIP/UDS
        v
Front ZCU (TC375)
  |-- Ethernet-CAN Gateway
  |-- UDS/DoIP Server
  |-- CAN / CAN FD Gateway
  |-- AEB Decision Logic
        |
        | CAN / CAN FD
        v
Sensor ECU / Motor ECU / OTA Bootloader

Camera ECU
        |
        | Ethernet Video Stream
        v
Vehicle Computer / LKAS
```

## 5. 프로젝트 의의

이 프로젝트는 차량 기능을 단순히 개별 펌웨어로 구현하는 데서 끝나지 않고,
기능 스토어, 차량 네트워크, OTA 진단 흐름, Gateway, Bootloader, LKAS, FVSA,
AEB를 하나의 SDV 서비스 흐름으로 연결했다는 점에 의미가 있습니다.

특히 Vehicle Computer와 Front ZCU 사이의 역할을 분리하고, SOME/IP, DoIP, CAN,
CAN FD를 함께 사용하는 통신 구조를 구성함으로써 Zonal Architecture 환경에서
소프트웨어 기능 배포와 차량 제어가 어떻게 결합될 수 있는지 검증했습니다.

또한 LKAS, AEB, FVSA를 독립 기능 모듈로 구성하고, 기능 활성화 상태와 OTA 적용
흐름을 함께 관리하는 구조를 시도했습니다. 이는 향후 차량 기능을 앱처럼 배포하고
라이프사이클을 관리하는 SDV 플랫폼으로 확장할 수 있는 기반이 됩니다.
