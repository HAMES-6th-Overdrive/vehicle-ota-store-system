# FVSA (Forward Vehicle Start Alarm)

<div align="center">

# 🚘 Forward Vehicle Start Alarm System

ToF Sensor ECU · ZCU · HPC 기반 전방 차량 출발 알림 시스템

</div>

---

# 📌 Overview

본 프로젝트는 차량 정차 상황에서
전방 차량과의 거리 변화를 감지하여
운전자에게 출발 알림을 제공하는
FVSA(Forward Vehicle Start Alarm) 시스템 구현 프로젝트입니다.

Sensor ECU에서는 ToF(Time of Flight) 센서를 통해
전방 차량과의 거리 데이터를 측정합니다.

측정된 데이터는 CAN 통신을 통해
ZCU(Zonal Control Unit)로 전달되며,
ZCU는 수신한 데이터를 Ethernet 통신을 통해
HPC(High Performance Computer)로 전달합니다.

HPC에서는 거리 변화 데이터를 분석하여
전방 차량의 출발 여부를 판단하고
운전자에게 경고를 제공합니다.

SDV(Software Defined Vehicle) 및
Zonal Architecture 구조를 기반으로 설계되었습니다.

---

# 🎯 Main Features

* 📡 ToF 센서 기반 거리 측정
* 🚌 CAN 기반 ECU 통신
* 🌐 Ethernet 기반 ZCU-HPC 데이터 전송
* 🚗 전방 차량 거리 변화 감지
* 📍 실시간 거리 분석
* 🚨 전방 차량 출발 알림
* 🧠 HPC 기반 판단 로직 수행
* ⚡ 실시간 데이터 처리
* 🔄 Zonal Architecture 기반 통신 구조

---

# 🏗️ System Architecture

```plaintext
┌─────────────────┐
│   Sensor ECU    │
│   ToF Sensor    │
└────────┬────────┘
         │
         │ CAN
         ▼
┌─────────────────┐
│       ZCU       │
│ CAN ↔ Ethernet  │
└────────┬────────┘
         │
         │ Ethernet
         ▼
┌─────────────────┐
│       HPC       │
│ Distance Analyze│
│  FVSA Control   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Driver Warning  │
│ Alarm / Display │
└─────────────────┘
```

---

# ⚙️ Tech Stack

## Hardware

* Raspberry Pi
* ToF Sensor
* ZCU Platform
* Vehicle Platform

## Software

* Python
* Linux (Ubuntu)
* Socket Programming

## Communication

* CAN
* Ethernet TCP/IP

---

# 📂 Project Structure

```bash
FVSA/
├── sensor_ecu/        # Sensor ECU source
├── zcu/               # ZCU communication source
├── hpc/               # HPC processing source
├── distance_analysis/ # Distance analysis logic
├── communication/     # CAN & Ethernet communication
├── utils/             # Utility functions
├── docs/              # Documents
└── README.md
```

---

# 🚀 How It Works

1. Sensor ECU에서 ToF 센서를 통해 거리 데이터 측정
2. CAN 통신을 통해 ZCU로 데이터 전송
3. ZCU에서 Ethernet 기반으로 HPC에 데이터 전달
4. HPC에서 거리 변화 분석 수행
5. 전방 차량 출발 여부 판단
6. 운전자에게 경고 및 알림 제공

---

# 📡 Distance Analysis Pipeline

```plaintext
ToF Distance Input
        ↓
CAN Transmission
        ↓
ZCU Data Routing
        ↓
Ethernet Streaming
        ↓
Distance Analysis
        ↓
Vehicle Movement Decision
        ↓
Driver Alert
```

---

# 🖥️ Installation

## Clone Repository

```bash
git clone https://github.com/HAMES-6th-Overdrive/FVSA.git
cd FVSA
```

## Install Dependencies

```bash
pip install -r requirements.txt
```

---

# ▶️ Run

## Sensor ECU

```bash
python sensor_sender.py
```

## ZCU

```bash
python zcu_gateway.py
```

## HPC

```bash
python main.py
```

---

# 📸 Demo

* Real-time ToF distance measurement
* CAN-based ECU communication
* Ethernet data streaming
* Distance change analysis
* Driver warning visualization

---

# 📈 Expected Results

* 안정적인 거리 측정
* 실시간 거리 변화 분석
* 신속한 차량 출발 감지
* 운전자 반응 시간 향상
* CAN/Ethernet 기반 안정적인 데이터 전송

---

# 🔥 Future Work

* CAN FD 적용
* 센서 융합 시스템 적용
* OTA 기반 업데이트 시스템
* SDV 플랫폼 확장
* AI 기반 상황 판단 기능 추가

---

# 👨‍💻 Team

HAMES 6th Overdrive Team

---

# 📄 License

This project is for educational and research purposes.
