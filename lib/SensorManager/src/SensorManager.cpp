#include "SensorManager.h"

const int SensorManager::sensorPins[numSensors] = {13, 12, 14, 26, 27, 25, 33, 35, 32};
const double positions[9] = {-218.5, -109.2, -72.8, -36.4, 0, 36.4, 72.8, 109.2, 218.5};
const double SensorManager::sensorSpacing = 3.6; // centimeters between sensors

/**
 * @brief Initializes all sensor pins as input with pulldown.
 */
void SensorManager::init()
{
    for (int i = 0; i < numSensors; i++)
    {
        pinMode(sensorPins[i], INPUT_PULLDOWN);
    }
}

/**
 * @brief Monitors sensors and timestamps when each one is triggered.
 */
void SensorManager::checkSensors()
{
    if (bleManager == nullptr)
        return;

    unsigned long currentTime = micros();
    static bool lastSensorStates[numSensors] = {HIGH};

    for (int i = expectedSensor; i < numSensors; i++)
    {
        bool currentSensorState = digitalRead(sensorPins[i]);

        if (lastSensorStates[i] == HIGH && currentSensorState == LOW)
        {
            timestamps[i] = currentTime;
            triggered[i] = true;


            if (i == 4)
            {
                centralSensorTime = currentTime;
            }
          

            lastTriggerTime = currentTime;
            expectedSensor = i + 1;

            // char logBuffer[100];
            // snprintf(logBuffer, sizeof(logBuffer), "🔍 Sensor %d triggered at: %.6f s",
            //          i, (currentTime) / 1.0e6);
            // Serial.println(logBuffer);

            // bleManager->sendLog(logBuffer);
        }
        lastSensorStates[i] = currentSensorState;
    }
}

/**
 * @brief Returns true if enough conditions are met to process the step data.
 */
bool SensorManager::shouldProcessData()
{
    if (bleManager == nullptr)
        return false;

    if ((micros() - lastTriggerTime) > 1000000 && expectedSensor > 0)
    {
        // bleManager->sendLog("✅ Condition 1: Timeout exceeded and partial trigger detected.");
        return true;
    }
    else if (getImportantSensorsTriggered())
    {
        // bleManager->sendLog("✅ Condition 2: All required sensors triggered.");
        return true;
    }
    return false;
}

/**
 * @brief Collects time and position data from triggered sensors.
 */
std::tuple<unsigned long *, double *> SensorManager::collectData(std::vector<double> &timeData, std::vector<double> &positionData)
{
    // Fill the vectors with triggered sensor data
    for (int i = 0; i < numSensors; i++)
    {
        if (triggered[i])
        {
            timeData.push_back(timestamps[i]);
            positionData.push_back(positions[i]);
        }
    }

    // Return pointers to the arrays
    return std::make_tuple(timestamps, (double *)positions);
}

/**
 * @brief Displays a summary of triggered sensor data.
 */
void SensorManager::plotResults()
{
    if (bleManager == nullptr)
        return;

    String output;
    output += "\n📍 Final Sensor Data (Recap):\n";
    output += "📍 Sens.\t| Time(s)\t| Position(mm)\n";

    for (int i = 0; i < numSensors; i++)
    {
        if (triggered[i])
        {
            char line[100];
            snprintf(line, sizeof(line), " %d\t| %+9.3f\t| %+7.2f\n",
                     i, ((double)timestamps[i] - (double)centralSensorTime) / 1.0e6, positions[i]);
            output += line;
        }
    }

    // Send to Serial
    // Serial.print(output);

    // Send to BLE
    // bleManager->sendLog(output.c_str());
}

/**
 * @brief Resets internal sensor state and timestamps.
 */
void SensorManager::reset()
{
    for (int i = 0; i < numSensors; i++)
    {
        timestamps[i] = 0;
        triggered[i] = false;
    }
    expectedSensor = 0;
    lastTriggerTime = 0;
}

/**
 * @brief Returns timestamp of the central sensor (index 4).
 */
unsigned long SensorManager::getCentralSensorTime()
{
    return centralSensorTime;
}

/**
 * @brief Returns the timestamp of the most recently triggered sensor.
 */
unsigned long SensorManager::getLastTriggerTime()
{
    return lastTriggerTime;
}

/**
 * @brief Checks if the first, central, and last sensor groups were triggered.
 */
bool SensorManager::getImportantSensorsTriggered()
{
    return triggered[4] && (triggered[5] || triggered[6] || triggered[7] || triggered[8]) && (triggered[0] || triggered[1] || triggered[2] || triggered[3]); // Check if the first, middle, and last sensors are triggered
}

/**
 * @brief Links the BluetoothManager for optional logging.
 */
void SensorManager::setBluetoothManager(BluetoothManager &btMgr)
{
    this->bleManager = &btMgr;
}