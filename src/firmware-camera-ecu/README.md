# Camera ECU

<div align="center">

# 📷 Camera ECU System

SDV 기반 실시간 차량 영상 수집 및 스트리밍 ECU

</div>

---

# 📌 Overview

본 프로젝트는 차량 내 Camera ECU(Camera Electronic Control Unit)를 구현한 프로젝트입니다.

카메라 센서로부터 실시간 영상을 수집하고,
수집된 프레임 데이터를 Ethernet 통신을 통해
HPC(High Performance Computer)로 전송합니다.

실시간 영상 스트리밍 및 저지연 데이터 전송을 목표로 하며,
ADAS 시스템에서 활용 가능한 영상 데이터를 안정적으로 전달하도록 설계되었습니다.

SDV(Software Defined Vehicle) 환경을 고려한
차량 네트워크 구조 기반으로 구현되었습니다.

---

# 🎯 Main Features

* 📷 실시간 카메라 영상 수집
* 🌐 Ethernet 기반 영상 스트리밍
* ⚡ 저지연 프레임 전송
* 🚘 ADAS 시스템 연동 지원
* 🐧 Linux 기반 ECU 시스템
* 🔄 실시간 네트워크 데이터 송신

---

# 🏗️ System Architecture

```plaintext
┌─────────────────┐
│ Camera Sensor   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│   Camera ECU    │
│ Frame Capture   │
│ Image Streaming │
└────────┬────────┘
         │
         │ Ethernet
         ▼
┌─────────────────┐
│       HPC       │
│ Video Receiver  │
└─────────────────┘
```

---

# ⚙️ Tech Stack

## Hardware

* Raspberry Pi
* Camera Module
* Vehicle Platform

## Software

* Python
* OpenCV
* Linux (Ubuntu)
* Socket Programming

## Communication

* Ethernet TCP/IP

---

# 📂 Project Structure

```bash
camera-ecu/
├── camera/            # Camera interface source
├── streaming/         # Frame streaming logic
├── communication/     # Ethernet communication
├── utils/             # Utility functions
├── docs/              # Documents
└── README.md
```

---

# 🚀 How It Works

1. 카메라 센서에서 실시간 프레임 수집
2. Camera ECU에서 프레임 처리 수행
3. Ethernet 기반 네트워크 전송 수행
4. HPC로 영상 데이터 전달
5. HPC에서 영상 데이터 수신 및 활용

---

# 📡 Streaming Pipeline

```plaintext
Camera Sensor
      ↓
Frame Capture
      ↓
Image Processing
      ↓
Frame Encoding
      ↓
Ethernet Streaming
      ↓
HPC Receiver
```

---

# 🖥️ Installation

## Clone Repository

```bash
git clone https://github.com/HAMES-6th-Overdrive/camera-ecu.git
cd camera-ecu
```

## Install Dependencies

```bash
pip install -r requirements.txt
```

---

# ▶️ Run

## Start Camera ECU

```bash
python main.py
```

---

# 📸 Demo

* Real-time camera streaming
* Ethernet frame transmission
* Low-latency image transfer
* Video receiver communication

---

# 📈 Expected Results

* 안정적인 영상 수집
* 저지연 프레임 전송
* 실시간 네트워크 스트리밍
* Ethernet 기반 안정적인 데이터 통신

---

# 🔥 Future Work

* H.264 영상 압축 적용
* 멀티 카메라 지원
* OTA 기반 업데이트 시스템
* 고해상도 스트리밍 지원
* 영상 전송 최적화

---

# 👨‍💻 Team

HAMES 6th Overdrive Team

---

# 📄 License

This project is for educational and research purposes.
