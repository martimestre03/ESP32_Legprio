# 🦿 LegPrio — Proprioception Evaluation System

**LegPrio** is a wearable, real-time system for assessing proprioception during gait using digital distance sensors and an IMU.  
Designed for rehabilitation research, the system detects **leg-crossing events** and evaluates **step phase timing** using 9 edge-triggered lasers and a 6-DOF MPU6050 accelerometer.  
It communicates with a host device via **Bluetooth Low Energy (BLE)**.

---

## 📦 Features

- Step phase detection (toe-off and heel-strike) using smoothed gyro data  
- Detection of leg-crossing events via 9 digital laser sensors  
- Real-time BLE data transmission for timestamps and positions  
- Polynomial fitting of triggered points for estimating gait curves  
- External button trigger with relative time and position logging  
- Compact, wearable integration on an **ESP32** using **PlatformIO**

---

## ⚙️ Hardware

- ESP32 (Dev Module)  
- 9x Pololu Digital Distance Sensors (`#4067`)  
- MPU6050 IMU (I2C)  
- Push button for user marking  
- BLE-connected smartphone or computer for receiving data  

---

## 📁 Folder Structure

```bash
LegPrio/
├── lib/                    # Custom modules
│   ├── AccelerometerHW123/
│   ├── BluetoothManager/
│   ├── ButtonHandler/
│   ├── PolynomialFit/
│   └── SensorManager/
├── src/
│   ├── main.cpp            # Main firmware
│   └── main-prova-IMU.cpp  # Optional IMU test
├── platformio.ini          # PlatformIO configuration
└── README.md               # Project documentation

---

## 🚀 Getting Started

1. **Install [PlatformIO](https://platformio.org/)** (VSCode recommended)  
2. Clone this repository:

```bash
git clone https://github.com/your-username/LegPrio.git
cd LegPrio
