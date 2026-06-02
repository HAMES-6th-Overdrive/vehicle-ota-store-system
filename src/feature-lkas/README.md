# LKAS (Lane Keeping Assist System)

<div align="center">

# 🚗 Lane Keeping Assist System

Camera ECU & HPC 기반 실시간 차선 유지 보조 시스템

</div>

---

# 📌 Overview

본 프로젝트는 차량용 Camera ECU와 HPC(High Performance Computer)를 활용하여
실시간 차선 인식 및 차량 조향 제어를 수행하는
LKAS(Lane Keeping Assist System) 구현 프로젝트입니다.

카메라 ECU에서 촬영한 영상을 Ethernet 통신을 통해 HPC로 전송하고,
HPC에서는 OpenCV 기반 차선 인식을 수행하여
차량이 차선을 유지할 수 있도록 조향 값을 계산합니다.

SDV(Software Defined Vehicle) 및
Zonal Architecture 환경을 고려한 구조로 설계되었습니다.

---

# 🎯 Main Features

* 📷 Camera ECU 기반 영상 수집
* 🌐 Ethernet 기반 프레임 전송
* 🛣️ 실시간 차선 인식
* 🚘 차선 유지 보조 제어
* 🧠 HPC 기반 영상 처리
* ⚡ 실시간 조향 값 계산
* 🔄 ECU 간 네트워크 통신

---

# 🏗️ System Architecture

```plaintext
┌─────────────────┐
│   Camera ECU    │
│  (Raspberry Pi) │
└────────┬────────┘
         │
         │ Ethernet
         ▼
┌─────────────────┐
│       HPC       │
│ Lane Detection  │
│  LKAS Control   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Vehicle Control │
│ Steering Output │
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
LKAS/
├── camera_ecu/        # Camera ECU source
├── hpc/               # HPC processing source
├── lane_detection/    # Lane detection algorithms
├── communication/     # Ethernet communication
├── utils/             # Utility functions
├── docs/              # Documents
└── README.md
```

---

# 🚀 How It Works

1. Camera ECU에서 실시간 영상 촬영
2. Ethernet 통신을 통해 HPC로 프레임 전송
3. HPC에서 차선 검출 수행
4. 차량 중심과 차선 중심 오차 계산
5. 조향 값을 생성하여 차량 제어 수행

---

# 🛣️ Lane Detection Pipeline

```plaintext
Camera Input
      ↓
Grayscale Conversion
      ↓
Gaussian Blur
      ↓
Canny Edge Detection
      ↓
ROI Extraction
      ↓
Hough Transform
      ↓
Lane Detection
      ↓
Steering Calculation
```

---

# 🖥️ Installation

## Clone Repository

```bash
git clone https://github.com/HAMES-6th-Overdrive/LKAS.git
cd LKAS
```

## Install Dependencies

```bash
pip install -r requirements.txt
```

---

# ▶️ Run

## Camera ECU

```bash
python camera_sender.py
```

## HPC

```bash
python main.py
```

---

# 📸 Demo

* Real-time lane detection
* Ethernet frame streaming
* Steering control visualization
* Vehicle lane tracking

---

# 📈 Expected Results

* 안정적인 차선 인식
* 실시간 영상 처리
* 차량 중앙 유지 성능 향상
* ECU 간 안정적인 데이터 전송

---

# 🔥 Future Work

* 객체 인식 기능 추가
* CAN 통신 연동
* 딥러닝 기반 차선 인식 적용
* OTA 기반 업데이트 시스템
* SDV 플랫폼 확장

---

# 👨‍💻 Team

HAMES 6th Overdrive Team

---

# 📄 License

This project is for educational and research purposes.
