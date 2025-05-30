#include <Arduino.h>
#include <Wire.h>
#include "SensorManager.h"
#include "PolynomialFit.h"
#include "ButtonHandler.h"
#include "AccelerometerHW123.h"
#include "BluetoothManager.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_wifi.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

SensorManager sensorManager;
PolynomialFit polyFit;
ButtonHandler buttonHandler;
AccelerometerHW123 accel(0x68);
BluetoothManager bleManager;
long lastTime = 0;
bool accelWorking = true; // Flag to check if accelerometer is working properly
static bool lastShouldProcess = false;
static bool lastHasEndTime = false;
bool shouldProcess = false;
bool hasEndTime = false;

static bool sensorsEnd = false;
static bool accelMode = true;                        // Flag to check if accelerometer is in motion mode
static const unsigned long CHECK_INTERVAL = 1000;    // 5ms in microseconds
static const unsigned long SENSOR_TIMEOUT = 1000000; // 1s timeout for sensor readings
static unsigned long lastCheckTime = 0;

double x2_rel_before;
double pos2_before;

double x2_rel_after;
double pos2_after;
char logBuffer[100];

void setup()
{
  Serial.begin(115200);
  delay(100); // Give some time for serial to initialize

  // Initialize GPIO
  pinMode(22, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  // Initialize BLE after CPU frequency is set
  bleManager.init("LegPrio");

  if (!accel.begin())
  {
    Serial.println("Failed to initialize AccelerometerHW123!");
    while (1)
      ; // Halt if sensor initialization fails
  }
  Serial.println("AccelerometerHW123 initialized successfully.");
  sensorManager.init();
  buttonHandler.init();
  buttonHandler.setPolynomialFit(polyFit);       // Pass the polynomial fit instance
  buttonHandler.setSensorManager(sensorManager); // Pass the sensor manager instance
  buttonHandler.setBluetoothManager(bleManager); // Pass the Bluetooth manager instance
  sensorManager.setBluetoothManager(bleManager); // Pass the Bluetooth manager instance
  accel.setBluetoothManager(bleManager);         // Pass the Bluetooth manager instance to accelerometer

  Serial.println("ESP32 Motion Tracker Ready!");
  delay(1000);
}

void loop()
{
  unsigned long currentTime = micros();
  if (currentTime - lastCheckTime >= CHECK_INTERVAL)
  {
    lastCheckTime = currentTime;

    // Always check button regardless of state
    buttonHandler.checkButton();

    if (accelMode)
    {
      // State machine logic
      if (accel.isDataReady())
      {
        // Read accelerometer to detect end
        float ax, ay, az;
        float gx, gy, gz;

        if (accel.readAcceleration(ax, ay, az) && accel.readGyro(gx, gy, gz))
        {
          accel.updateShockState(ax, ay, az, gx);
          accelWorking = true;

          // Check if end was detected
          if (accel.getStartTime() != 0 && accel.getEndTime() == 0 && !sensorsEnd)
          {
            accelMode = false;
          }
          if (accel.getEndTime() != 0)
          {
            // Continue with the state machine

            // // Process all the data with the end time from previous cycle
            // snprintf(logBuffer, sizeof(logBuffer), "accel has end time");
            // Serial.println(logBuffer);
            // bleManager.sendLog(logBuffer);

            if (sensorManager.getImportantSensorsTriggered())
            {
              std::vector<double> timeData;
              std::vector<double> positionData;
              std::vector<double> relTimeData;

              auto [times, positions] = sensorManager.collectData(timeData, positionData);

              // Create relative times array from the full times array
              double relTimes[9];
              for (int i = 0; i < 9; i++)
              {
                relTimes[i] = ((double)times[i] - (double)times[4]) / 1.0e6;
              }

              x2_rel_before = relTimes[2];
              pos2_before = positions[2];

              if (relTimes[2] == 0)
              {
                // relTimes[6] no es válido, usamos relTimes[7]
                snprintf(logBuffer, sizeof(logBuffer), "  ⚠️ relTimes[2] inválido. Usando relTimes[1] en su lugar.");
                Serial.println(logBuffer);
                bleManager.sendLog(logBuffer);

                x2_rel_before = relTimes[1];
                pos2_before = positions[1];
              }

              x2_rel_after = relTimes[6];
              pos2_after = positions[6];

              if (relTimes[6] == 0)
              {
                // relTimes[6] no es válido, usamos relTimes[7]

                snprintf(logBuffer, sizeof(logBuffer), "  ⚠️ relTimes[6] inválido. Usando relTimes[7] en su lugar.");
                Serial.println(logBuffer);
                bleManager.sendLog(logBuffer);
                x2_rel_after = relTimes[7];
                pos2_after = positions[7];
              }
              snprintf(logBuffer, sizeof(logBuffer), "📦 Collected sensor points: %d", (int)timeData.size());
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);

              snprintf(logBuffer, sizeof(logBuffer), "📊 Sensor Timing & Positions:");
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);
              sensorManager.plotResults();

              snprintf(logBuffer, sizeof(logBuffer), "  🟢 Step Started at: %.6f s", ((double)accel.getStartTime()) / 1e6);
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);

              snprintf(logBuffer, sizeof(logBuffer), "  🛑 Step Ended at:   %.6f s", ((double)accel.getEndTime()) / 1e6);
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);

              snprintf(logBuffer, sizeof(logBuffer), "  🕒 Central Sensor Time: %.6f s", (double)times[4] / 1e6);
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);

              snprintf(logBuffer, sizeof(logBuffer), "\n🚶 Step Event:");
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);

              snprintf(logBuffer, sizeof(logBuffer), "  🟢 Step Started Rel at: %.6f s", ((double)accel.getStartTime() - (double)times[4]) / 1e6);
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);

              snprintf(logBuffer, sizeof(logBuffer), "  🛑 Step Ended Rel at:   %.6f s", ((double)accel.getEndTime() - (double)times[4]) / 1e6);
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);

              if (buttonHandler.wasPressed())
              {
                double relStartTime = ((double)accel.getStartTime() - (double)times[4]) / 1.0e6;
                double relEndTime = ((double)accel.getEndTime() - (double)times[4]) / 1.0e6;
                double buttonRelTime = buttonHandler.getButtonTime();

                for (double t : timeData)
                {
                  relTimeData.push_back(((double)t - (double)times[4]) / 1.0e6);
                }

                if (buttonRelTime < relTimeData[0])
                {
                  Serial.println("  🕹️ Button pressed before the first sensor.");
                  double x1_rel = relTimes[0]; // Use relTimes instead of relTimeData

                  double start_rel = ((double)accel.getStartTime() - (double)times[4]) / 1.0e6;
                  auto [Al, Bl] = polyFit.calculateAandB(x1_rel, positions[0], x2_rel_before, pos2_before, start_rel);
                  Serial.printf("  🧮 Left A: %.6f | Left B: %.6f\n", Al, Bl);
                  double position = Al * pow(buttonRelTime, 3) + Bl * buttonRelTime;
                  buttonHandler.setPosition(position);
                  Serial.printf("  🎯 Estimated Position at Press: %.6f mm ❗\n", position);

                  // Send data over Bluetooth
                  bleManager.sendData(position, buttonRelTime, relStartTime, relEndTime);
                }
                else if (buttonRelTime > relTimes[8])
                {
                  Serial.println("  🕹️ Button pressed after last sensor was triggered.");

                  // Obtener x1 y x2 usando relTimes[8] y una comprobación de validez para relTimes[6]
                  double x1_rel = relTimes[8];

                  double end_rel = ((double)accel.getEndTime() - (double)times[4]) / 1.0e6;

                  // Calcular A y B
                  auto [Ar, Br] = polyFit.calculateAandB(x1_rel, positions[8], x2_rel_after, pos2_after, end_rel);
                  Serial.printf("  🧮 Right A: %.6f | Right B: %.6f\n", Ar, Br);

                  // Estimar la posición
                  double position = Ar * pow(buttonRelTime, 3) + Br * buttonRelTime;
                  buttonHandler.setPosition(position);
                  Serial.printf("  🎯 Estimated Position at Press: %.6f mm ❗\n", position);

                  // Enviar datos por Bluetooth
                  bleManager.sendData(position, buttonRelTime, relStartTime, relEndTime);
                }
                else if (buttonRelTime > x2_rel_before && buttonRelTime < x2_rel_after)
                {
                  Serial.println("  🕹️ Button pressed during sensor triggering window. (Middle)");

                  Serial.println("\n🔍 Debug PolynomialFit inputs (excluding first & last):");

                  // Create new vectors excluding first and last elements
                  std::vector<double> fitTimeData(relTimeData.begin() + 1, relTimeData.end() - 1);
                  std::vector<double> fitPositionData(positionData.begin() + 1, positionData.end() - 1);

                  Serial.println("Time data (relative):");
                  for (size_t i = 0; i < fitTimeData.size(); i++)
                  {
                    Serial.printf("[%d]: %.6f\n", (int)i + 1, fitTimeData[i]); // +1 to match original indexing
                  }

                  Serial.println("\nPosition data:");
                  for (size_t i = 0; i < fitPositionData.size(); i++)
                  {
                    Serial.printf("[%d]: %.6f\n", (int)i + 1, fitPositionData[i]);
                  }

                  Serial.printf("\nDegree: %d\n", sensorManager.getDegree());

                  // Use filtered vectors for fitting
                  polyFit.fit(fitTimeData, fitPositionData, sensorManager.getDegree());
                  double position = polyFit.evaluate(buttonRelTime);
                  buttonHandler.setPosition(position);
                  Serial.printf("  🎯 Estimated Position at Press: %.6f mm ❗\n", position);
                  polyFit.printCoefficients();

                  // Send data over Bluetooth

                  bleManager.sendData(position, buttonRelTime, relStartTime, relEndTime);
                }
                else if ((buttonRelTime > ((double)accel.getStartTime() - (double)times[4]) / 1.0e6) && (buttonRelTime < ((double)accel.getEndTime() - (double)times[4]) / 1.0e6))
                {
                  Serial.println("  🕹️ Button pressed during sensor triggering window. (Extremes)");

                  double x1, y1, x2, y2;

                  if (buttonRelTime < x2_rel_before)
                  {
                    // Interpolar entre punto inicial y x2_rel_before
                    x1 = relTimes[0];
                    y1 = positions[0];

                    if (x2_rel_before == 0)
                    {
                      Serial.println("  ⚠️ x2_rel_before inválido. Usando relTimes[1] y positions[1] en su lugar.");
                      x2_rel_before = relTimes[1];
                      pos2_before = positions[1];
                    }

                    x2 = x2_rel_before;
                    y2 = pos2_before;

                    Serial.println("  📏 Using LEFT segment for linear interpolation.");
                  }
                  else
                  {
                    // Interpolar entre x2_rel_after y relTimes[8]
                    x1 = x2_rel_after;
                    y1 = pos2_after;

                    if (x2_rel_after == 0)
                    {
                      Serial.println("  ⚠️ x2_rel_after inválido. Usando relTimes[7] y positions[7] en su lugar.");
                      x2_rel_after = relTimes[7];
                      pos2_after = positions[7];
                    }

                    x2 = relTimes[8];
                    y2 = positions[8];

                    Serial.println("  📏 Using RIGHT segment for linear interpolation.");
                  }

                  // Evita división por 0
                  if (x2 - x1 == 0)
                  {
                    Serial.println("  ❌ Error: Tiempo duplicado, no se puede interpolar.");
                  }
                  else
                  {
                    double slope = (y2 - y1) / (x2 - x1);
                    double position = y1 + slope * (buttonRelTime - x1);

                    buttonHandler.setPosition(position);
                    Serial.printf("  📈 Linear interpolation between [%.6f, %.6f] and [%.6f, %.6f]\n", x1, y1, x2, y2);
                    Serial.printf("  🎯 Estimated Position at Press: %.6f mm ❗\n", position);

                    // Enviar datos por Bluetooth
                    bleManager.sendData(position, buttonRelTime, relStartTime, relEndTime);
                  }
                }
                else
                {
                  snprintf(logBuffer, sizeof(logBuffer), "  ❌ Button pressed outside of start and end window.");
                  Serial.println(logBuffer);
                  bleManager.sendLog(logBuffer);
                }
              }
              snprintf(logBuffer, sizeof(logBuffer), "\n📍 Final Sensor Data (Recap):");
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);
              sensorManager.plotResults();
              buttonHandler.plotResults();

            }
            else
            {
              snprintf(logBuffer, sizeof(logBuffer), "  ❌ No important sensors triggered.");
              Serial.println(logBuffer);
              bleManager.sendLog(logBuffer);
            }
            

            // Reset for next cycle if we timeout
            snprintf(logBuffer, sizeof(logBuffer), "\n🔁 System Reset");
            Serial.println(logBuffer);
            bleManager.sendLog(logBuffer);
            sensorManager.reset();
            buttonHandler.reset();
            accel.reset();
            sensorsEnd = false;
          }
        }
      }
    }

    else
    {
      // Reading sensors phase
      sensorManager.checkSensors();

      // Check for sensor timeout
      if (micros() - sensorManager.getLastTriggerTime() > SENSOR_TIMEOUT && sensorManager.getLastTriggerTime() > 0)
      {
        snprintf(logBuffer, sizeof(logBuffer), "\n⚠️ Sensor reading timed out");
        Serial.println(logBuffer);
        bleManager.sendLog(logBuffer);
        sensorManager.plotResults();
        accelMode = true; // Switch back to accelerometer mode
        sensorsEnd = true;
      }
      else if (sensorManager.getImportantSensorsTriggered())
      {
        snprintf(logBuffer, sizeof(logBuffer), "📊 Sensor Timing & Positions:");
        Serial.println(logBuffer);
        bleManager.sendLog(logBuffer);
        sensorManager.plotResults();
        accelMode = true; // Switch back to accelerometer mode
        sensorsEnd = true;
      }
    }
    if (!bleManager.isConnected() && !bleManager.isCurrentlyAdvertising())
    {
      Serial.println("🔁 Restarting BLE advertising...");
      bleManager.startAdvertising();
    }
  }
}
