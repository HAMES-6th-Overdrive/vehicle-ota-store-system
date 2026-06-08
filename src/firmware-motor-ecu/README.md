# Motor ECU

<div align="center">

# 🚗 Motor ECU System

CAN 기반 차량 제어 데이터 변환 및 UART 패킷 송신 시스템

</div>

---

# 📌 Overview

본 프로젝트는 차량 제어를 위한
Motor ECU(Electronic Control Unit)를 구현한 프로젝트입니다.

ZCU(Zonal Control Unit)로부터 CAN 통신을 통해
차량 제어 데이터를 수신한 뒤,
기존 차량 내부 보드에서 사용하는 UART 패킷 형식으로 변환하여 전송합니다.

기존 차량은 자체 컨트롤러와 보드 간 Bluetooth 통신을 사용하고 있었으며,
해당 통신에서 송수신되는 데이터 패킷을 분석하여
동일한 형식의 UART 패킷을 구성하였습니다.

이를 통해 기존 차량 시스템을 변경하지 않고도
외부 제어 시스템과 연동할 수 있도록 설계되었습니다.

SDV(Software Defined Vehicle) 및
Zonal Architecture 기반 차량 구조를 고려하여 구현되었습니다.

---

# 🎯 Main Features

* 🚌 CAN 기반 차량 제어 데이터 수신
* 🔄 CAN 데이터 → UART 패킷 변환
* 📡 기존 차량 프로토콜 분석 기반 패킷 구성
* ⚡ 실시간 제어 데이터 처리
* 🚗 기존 차량 보드 연동 지원
* 🔌 UART 기반 제어 데이터 송신
* 🐧 Linux 기반 ECU 시스템

---

# 🏗️ System Architecture

```plaintext
┌─────────────────┐
│       ZCU       │
│ Control Sender  │
└────────┬────────┘
         │
         │ CAN
         ▼
┌─────────────────┐
│    Motor ECU    │
│ Packet Convert  │
│ UART Transmit   │
└────────┬────────┘
         │
         │ UART
         ▼
┌─────────────────┐
│ Existing Vehicle│
│   Control Board │
└─────────────────┘
```

---

# ⚙️ Tech Stack

## Hardware

* Raspberry Pi
* Vehicle Control Board
* CAN Interface Module
* UART Interface

## Software

* Python
* Linux (Ubuntu)
* Socket Programming
* Serial Communication

## Communication

* CAN
* UART

---

# 📂 Project Structure

```bash
motor-ecu/
├── can/               # CAN communication
├── uart/              # UART transmission
├── packet/            # Packet conversion logic
├── communication/     # Communication utilities
├── utils/             # Utility functions
├── docs/              # Documents
└── README.md
```

---

# 🚀 How It Works

1. ZCU에서 CAN 기반 차량 제어 데이터 전송
2. Motor ECU에서 CAN 메시지 수신
3. 기존 차량 프로토콜 기반 패킷 생성
4. UART 형식으로 데이터 변환 수행
5. 기존 차량 내부 보드로 UART 데이터 전송
6. 차량 제어 기능 수행

---

# 📡 Packet Conversion Pipeline

```plaintext
CAN Message Input
        ↓
CAN Data Parsing
        ↓
Packet Conversion
        ↓
UART Packet Generate
        ↓
UART Transmission
        ↓
Vehicle Control Board
```

---

# 🖥️ Installation

## Clone Repository

```bash
git clone https://github.com/HAMES-6th-Overdrive/motor-ecu.git
cd motor-ecu
```

## Install Dependencies

```bash
pip install -r requirements.txt
```

---

# ▶️ Run

## Start Motor ECU

```bash
python main.py
```

---

# 📸 Demo

* CAN message reception
* UART packet conversion
* Existing vehicle board communication
* Real-time control data transmission

---

# 📈 Expected Results

* 안정적인 CAN 메시지 수신
* 기존 차량 프로토콜 호환 지원
* 실시간 UART 데이터 전송
* 기존 차량 보드 연동 성공
* 차량 제어 응답 속도 향상

---

# 🔥 Future Work

* CAN FD 지원
* 패킷 검증 기능 추가
* UART 통신 안정성 향상
* OTA 기반 업데이트 시스템
* 차량 상태 피드백 기능 추가

---

# 👨‍💻 Team

HAMES 6th Overdrive Team

---

# 📄 License

This project is for educational and research purposes.
