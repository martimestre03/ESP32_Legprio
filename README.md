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
- 9x Pololu Digital Distance Sensors (`#4067`)  A
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
```
---

## 🚀 Getting Started

1. **Install VSCode (https://code.visualstudio.com/download)**
2. **Install de PlatformIO in VSCode extensions (https://platformio.org/)**
3. **Install git (https://git-scm.com/downloads/win) and run the download**
4. Open VSCode and a terminal whetever you want the project to be
6. Clone this repository:

```bash
git clone https://github.com/martimestre03/ESP32_Legprio.git
cd ESP32_Legprio
```
7. Go to File > Open Folder > Select "ESP32_Legprio"
8.Connect your ESP32 via USB
9.Upload the firmware:

```bash
pio run --target upload
```
---

## 📡 BLE Output Format
Sensor data is sent over BLE in JSON format like this:
- t	Absolute timestamp in microseconds 
- p	Position at button press (position_error)
- b	Button time relative to central sensor (time_error)
- s	Step start time 
- e	Step end time

Example:
```bash
{
  "t": 12345678,
  "p": -36.4,
  "b": -0.01,
  "s": -0.230,
  "e": 0.310
}
```


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
