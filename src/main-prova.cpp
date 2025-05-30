// #include <Wire.h>
// #include "AccelerometerHW123.h"

// AccelerometerHW123 imu(0x68); // MPU6050 default I2C address
// const int buttonPin = 34;     // Ajusta al pin real que uses

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   pinMode(buttonPin, INPUT_PULLUP);

//   if (!imu.begin()) {
//     Serial.println("❌ MPU6050 not found!");
//     while (1);
//   }

//   Serial.println("millis,gx,gy,gz,ax,ay,az,stepStarted,endDetected,footRelaxed,button");
// }

// void loop() {
//   float ax, ay, az;
//   float gx, gy, gz;
//   int buttonPressed = digitalRead(buttonPin) == LOW ? 1 : 0;

//   if (imu.isDataReady()) {
//     if (imu.readAcceleration(ax, ay, az) && imu.readGyro(gx, gy, gz)) {

//       imu.updateShockState(ax, ay, az, gx);

//       Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\n",
//         millis(),
//         gx, gy, gz,
//         ax, ay, az,
//         imu.isStepStarted() ? 1 : 0,
//         imu.hasEndTime() ? 1 : 0,
//         imu.isFootRelaxed() ? 1 : 0,
//         buttonPressed
//       );
//     }
//   }

//   delay(10); // 100Hz
// }
