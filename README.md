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

3.Connect your ESP32 via USB
4.Upload the firmware:

```bash
pio run --target upload
---

## 📡 BLE Output Format
Sensor data is sent over BLE in JSON format like this:

{
  "t": 12345678,
  "p": -36.4,
  "b": 0.253,
  "s": 0.230,
  "e": 0.310
}
Key	Meaning
-t	Absolute timestamp in microseconds
-p	Position at button press
-b	Button time relative to central sensor
-s	Step start time (relative)
-e	Step end time (relative)
---

## 📈 Example Use Case
Strap the device around the legs with beam sensors aligned across the gait path

Start walking naturally

Press the button when you feel your legs are crossing

Data is streamed via BLE to the host

Use the collected timestamps and fitted positions to analyze proprioception delay

---

## 👨‍🔬 Contributors
Martí Mestre — Concept, hardware, firmware, validation

Supervisors: Prof. Ramon Bragós (UPC), Prof. David Reinkensmeyer (UCI)
---

## 📄 License
This project is licensed under the MIT License.
See the LICENSE file for more details.
