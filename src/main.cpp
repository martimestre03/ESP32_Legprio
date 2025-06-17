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

bool accelWorking = true; // Flag to check if accelerometer is working properly

static bool sensorsEnd = true;
static bool accelMode = true;         // Flag to check if accelerometer is in motion mode
static bool accelModeChanged = false; // Flag to check if accelerometer is in motion mode

static const unsigned long CHECK_INTERVAL = 500;        // 0.5ms in microseconds
static const unsigned long CHECK_STATE_INTERVAL = 300000; // 0.3s in microseconds
static const unsigned long SENSOR_TIMEOUT = 500000;      // 0.1s timeout for sensor readings

static unsigned long lastGeneralCheckTime = 0;
static unsigned long lastAccelCheckTime = 0;
static unsigned long lastButtonCheckTime = 0;

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
    // Serial.println("Failed to initialize AccelerometerHW123!");
    while (1)
      ; // Halt if sensor initialization fails
  }
//   Serial.println("AccelerometerHW123 initialized successfully.");
  sensorManager.init();
  buttonHandler.init();
  buttonHandler.setPolynomialFit(polyFit);       // Pass the polynomial fit instance
  buttonHandler.setSensorManager(sensorManager); // Pass the sensor manager instance
  buttonHandler.setBluetoothManager(bleManager); // Pass the Bluetooth manager instance
  sensorManager.setBluetoothManager(bleManager); // Pass the Bluetooth manager instance
  accel.setBluetoothManager(bleManager);         // Pass the Bluetooth manager instance to accelerometer
  accel.setSensorManager(sensorManager);         // Pass the sensor manager instance to accelerometer

//   Serial.println("ESP32 Motion Tracker Ready!");
  delay(1000);
}

unsigned long t0, t1;

void loop()
{
  unsigned long loopStart = micros();

  unsigned long currentTime = micros();

  if (accelMode)
  {
    // State machine logic
    if (accel.isDataReady())
    {
      // Read accelerometer to detect end
      float gx, gy, gz;

      if (accel.readGyro(gx, gy, gz))
      {
        accel.updateShockState(gx);
        accelWorking = true;

        // Check if end was detected
        if (accel.getStartTime() != 0 && accel.getEndTime() == 0 && sensorsEnd && !accelModeChanged)
        {
          accelMode = false;
          accelModeChanged = true; // Set the flag to indicate mode change
          sensorsEnd = false;      // Reset the sensors end flag
        }
      }
    }
  }

  else
  {
    // Reading sensors phase
    t1 = micros();
    sensorManager.checkSensors();
    // Serial.printf("⏱️ sensorManager.checkSensors() at %lu lasted %lu us\n", micros() - loopStart , micros() - t1);

    if (currentTime - lastAccelCheckTime >= CHECK_INTERVAL)
    {
      lastAccelCheckTime = currentTime;
      if (accel.isDataReady())
      {
        // Read accelerometer to detect end
        float gx, gy, gz;

        if (accel.readGyro(gx, gy, gz))
        {
          accel.updateShockState(gx);
          accelWorking = true;
        }
      }
    }

    // Check for sensor timeout
    if (micros() - sensorManager.getLastTriggerTime() > SENSOR_TIMEOUT && sensorManager.getLastTriggerTime() > 0)
    {
      // snprintf(logBuffer, sizeof(logBuffer), "\n⚠️ Sensor reading timed out");
    //   Serial.println(logBuffer);
      // snprintf(logBuffer, sizeof(logBuffer), "📊 Sensor Timing & Positions:");
      // Serial.println(logBuffer);
      // bleManager.sendLog(logBuffer);
      sensorManager.plotResults();
      accelMode = true; // Switch back to accelerometer mode
      sensorsEnd = true;
    }
  }

  if (currentTime - lastGeneralCheckTime >= CHECK_STATE_INTERVAL)
  {

    lastGeneralCheckTime = currentTime;
    // Always check button regardless of state
    if (accel.getEndTime() != 0)
    {
      // Continue with the state machine

      // // Process all the data with the end time from previous cycle
      // snprintf(logBuffer, sizeof(logBuffer), "accel has end time");
      // Serial.println(logBuffer);
      // bleManager.sendLog(logBuffer);

      if (sensorManager.getImportantSensorsTriggered())
      {
        std::vector<double> timeVector;
        std::vector<double> positionVector;
        std::vector<double> relTimeVector;

        auto [times, positions] = sensorManager.collectData(timeVector, positionVector);
        size_t lastIndex = timeVector.size() - 1;

        // Create relative times array from the full times array
        double relTimes[9];
        for (int i = 0; i < 9; i++)
        {
          relTimes[i] = ((double)times[i] - (double)times[4]) / 1.0e6;
        }

        // snprintf(logBuffer, sizeof(logBuffer), "📦 Collected sensor points: %d", (int)timeVector.size());
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);

        // snprintf(logBuffer, sizeof(logBuffer), "📊 Sensor Timing & Positions:");
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);
        // sensorManager.plotResults();

        // snprintf(logBuffer, sizeof(logBuffer), "  🟢 Step Started at: %.6f s", ((double)accel.getStartTime()) / 1e6);
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);

        // snprintf(logBuffer, sizeof(logBuffer), "  🛑 Step Ended at:   %.6f s", ((double)accel.getEndTime()) / 1e6);
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);

        // snprintf(logBuffer, sizeof(logBuffer), "  🕒 Central Sensor Time: %.6f s", (double)times[4] / 1e6);
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);

        // snprintf(logBuffer, sizeof(logBuffer), "\n🚶 Step Event:");
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);

        // snprintf(logBuffer, sizeof(logBuffer), "  🟢 Step Started Rel at: %.6f s", ((double)accel.getStartTime() - (double)times[4]) / 1e6);
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);

        // snprintf(logBuffer, sizeof(logBuffer), "  🛑 Step Ended Rel at:   %.6f s", ((double)accel.getEndTime() - (double)times[4]) / 1e6);
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);

        if (buttonHandler.wasPressed())
        {
        //   Serial.println("accel.getStaTime: " + String(accel.getStartTime()));
        //   Serial.println("accel.getEndTime: " + String(accel.getEndTime()));
        //   Serial.println("times[4]: " + String(times[4]));

          double relStartTime = ((double)accel.getStartTime() - (double)times[4]) / 1.0e6;
          double relEndTime = ((double)accel.getEndTime() - (double)times[4]) / 1.0e6;
          double buttonRelTime = buttonHandler.getButtonTime();

          for (double t : timeVector)
          {
            relTimeVector.push_back(((double)t - (double)times[4]) / 1.0e6);
          }
          if (buttonRelTime < relStartTime || buttonRelTime > relEndTime)
          {
            // snprintf(logBuffer, sizeof(logBuffer), "  ❌ Button pressed outside of start and end window.");
            // Serial.println(logBuffer);
            // bleManager.sendLog(logBuffer);
          }
          else if (buttonRelTime < relTimeVector[0])
          {
            // Serial.println("  🕹️ Button pressed before the first sensor.");

            auto [Al, Bl] = polyFit.calculateAandB(relTimeVector[0], positionVector[0], relStartTime);
            double position = Al * pow(buttonRelTime, 3) + Bl * buttonRelTime;
            buttonHandler.setPosition(position);
            bleManager.sendData(position, buttonRelTime, relStartTime, relEndTime);

            // Serial.printf("  🧮 Left A: %.6f | Left B: %.6f\n", Al, Bl);
            // Serial.printf("  🎯 Estimated Position at Press: %.6f mm ❗\n", position);
          }
          else if (buttonRelTime > relTimeVector[lastIndex])
          {
            // Serial.println("  🕹️ Button pressed after last sensor was triggered.");

            auto [Ar, Br] = polyFit.calculateAandB(relTimeVector[lastIndex], positionVector[lastIndex], relEndTime);
            double position = Ar * pow(buttonRelTime, 3) + Br * buttonRelTime;
            buttonHandler.setPosition(position);
            bleManager.sendData(position, buttonRelTime, relStartTime, relEndTime);

            // Serial.printf("  🧮 Right A: %.6f | Right B: %.6f\n", Ar, Br);
            // Serial.printf("  🎯 Estimated Position at Press: %.6f mm ❗\n", position);
          }
          else
          {
            double x1, y1, x2, y2;
            for (int i = 0; i < 8; i++)
            {
              if (buttonRelTime >= relTimeVector[i] && buttonRelTime <= relTimeVector[i + 1])
              {
                x1 = relTimeVector[i];
                y1 = positionVector[i];
                x2 = relTimeVector[i + 1];
                y2 = positionVector[i + 1];
                break;
              }
            }
            // Serial.println("  🕹️ Button pressed during sensor triggering window. (Middle)");

            double a = (y2 - y1) / (x2 - x1);
            double b = y1 - a * x1;
            double position = a * buttonRelTime + b;
            // Serial.printf("  📏 Linear interpolation between [%.6f, %.6f] and [%.6f, %.6f]\n", x1, y1, x2, y2);
            buttonHandler.setPosition(position);
            bleManager.sendData(position, buttonRelTime, relStartTime, relEndTime);
          }
        }

        sensorManager.plotResults();
        buttonHandler.plotResults();
        
      }
      else
      {
        // snprintf(logBuffer, sizeof(logBuffer), "  ❌ No important sensors triggered.");
        // Serial.println(logBuffer);
        // bleManager.sendLog(logBuffer);
      }

      // Reset for next cycle if we timeout
      // snprintf(logBuffer, sizeof(logBuffer), "\n🔁 System Reset");
      // Serial.println(logBuffer);
      // bleManager.sendLog(logBuffer);
      accelModeChanged = false; // Reset the mode change flag
      sensorManager.reset();
      buttonHandler.reset();
      accel.reset();
    }
    if (!bleManager.isConnected() && !bleManager.isCurrentlyAdvertising())
    {
    //   Serial.println("🔁 Restarting BLE advertising...");
      bleManager.startAdvertising();
    }
  }

  if (currentTime - lastButtonCheckTime >= CHECK_INTERVAL)
  {
    t0 = micros();
    buttonHandler.checkButton();
    // Serial.printf("⏱️ buttonHandler.checkButton() at %lu laster %lu us\n", micros() - loopStart, micros() - t0);
  }

}
