🦿 LegPrio — Proprioception Evaluation System
LegPrio is a wearable real-time system for assessing proprioception during gait using a set of digital distance sensors and an IMU. Designed for rehabilitation research, the system detects leg-crossing events and evaluates step phase timing using edge-triggered laser sensors and a 6-DOF MPU6050 accelerometer. It communicates with a host device via Bluetooth Low Energy (BLE).

📦 Features
Step phase detection (toe-off and heel-strike) using smoothed gyro data

Detection of leg-crossing events via 9 digital laser sensors

Real-time BLE data transmission for timestamps and positions

Polynomial fitting of triggered points for estimating gait curves

External button trigger with relative time and position logging

Compact integration on an ESP32 using PlatformIO

⚙️ Hardware
ESP32 (Dev Module)

9x Pololu Digital Distance Sensors (#4067)

MPU6050 IMU (I2C)

Push button for validation

Optional: BLE-connected smartphone or laptop

📁 Folder Structure
bash
Copiar
Editar
📦LegPrio/
 ┣ 📂lib/                        # Custom libraries/modules
 ┃ ┣ 📂AccelerometerHW123/
 ┃ ┃ ┣ 📜AccelerometerHW123.cpp
 ┃ ┃ ┗ 📜AccelerometerHW123.h
 ┃ ┣ 📂BluetoothManager/
 ┃ ┃ ┣ 📜BluetoothManager.cpp
 ┃ ┃ ┗ 📜BluetoothManager.h
 ┃ ┣ 📂ButtonHandler/
 ┃ ┃ ┣ 📜ButtonHandler.cpp
 ┃ ┃ ┗ 📜ButtonHandler.h
 ┃ ┣ 📂PolynomialFit/
 ┃ ┃ ┣ 📜PolynomialFit.cpp
 ┃ ┃ ┗ 📜PolynomialFit.h
 ┃ ┣ 📂SensorManager/
 ┃ ┃ ┣ 📜SensorManager.cpp
 ┃ ┃ ┗ 📜SensorManager.h
 ┃ ┗ 📜README                   # (optional module readme)
 ┣ 📂src/
 ┃ ┣ 📜main.cpp                # Main firmware file
 ┃ ┗ 📜main-prova-IMU.cpp      # Optional IMU testing file
 ┣ 📂include/                  # Unused in this project
 ┣ 📜platformio.ini            # PlatformIO build configuration
 ┗ 📜README.md                 # Project overview

Install PlatformIO (VSCode recommended)

Clone this repository:

bash
Copiar
Editar
git clone https://github.com/your-username/LegPrio.git
cd LegPrio
Connect your ESP32 device via USB.

Upload the firmware:

bash
Copiar
Editar
pio run --target upload
📡 BLE Output Format
Sensor data is sent in JSON format via BLE:

json
Copiar
Editar
{
  "t": 12345678,
  "p": -36.4,
  "b": 0.253,
  "s": 0.230,
  "e": 0.310
}
t: Absolute microsecond timestamp

p: Position at button press

b: Button time relative to central sensor

s: Step start time relative to central sensor

e: Step end time relative to central sensor

📈 Example Use Case
Attach the device around the leg so that the beam sensors are aligned across the gait path.

Start walking naturally.

Press the button when you feel your legs are crossing.

Data is streamed via BLE to the connected host.

Use the collected timestamps and positions to evaluate proprioception delay.

🧠 Contributors
Martí Mestre – Concept, hardware design, firmware, and validation

Supervisor(s): Prof. Bragós (UPC), Prof. Reinkensmeyer (UCI)

📄 License
This project is under the MIT License. See LICENSE for more info.
