#include "SensorManager.h"

const int SensorManager::sensorPins[numSensors] = {13, 12, 14, 26, 27, 25, 33, 35, 32};
const double positions[9] = {-218.5, -109.2, -72.8, -36.4, 0, 36.4, 72.8, 109.2, 218.5};
const double SensorManager::sensorSpacing = 3.6; // centimeters between sensors

void SensorManager::init()
{
    for (int i = 0; i < numSensors; i++)
    {
        pinMode(sensorPins[i], INPUT_PULLDOWN);
    }
}

void SensorManager::setBluetoothManager(BluetoothManager &btMgr)
{
    this->bleManager = &btMgr;
}

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

            if (i == 0)
            {
                firstSensorTime = currentTime;
            }

            else if (i == 4)
            {
                centralSensorTime = currentTime;
            }
            else if (i == 8)
            {
                lastSensorTime = currentTime;
            }

            lastTriggerTime = currentTime;
            expectedSensor = i + 1;

            // char logBuffer[100];
            // snprintf(logBuffer, sizeof(logBuffer), "🔍 Sensor %d triggered at: %.6f s",
            //         i, (currentTime - firstSensorTime) / 1.0e6);
            // bleManager->sendLog(logBuffer);
        }
        lastSensorStates[i] = currentSensorState;
    }
}

bool SensorManager::shouldProcessData()
{
    if (bleManager == nullptr)
        return false;

    if ((micros() - lastTriggerTime) > 1000000 && expectedSensor > 0)
    {
        bleManager->sendLog("✅ Condition 1: Timeout exceeded and partial trigger detected.");
        return true;
    }
    else if (getImportantSensorsTriggered())
    {
        bleManager->sendLog("✅ Condition 2: All required sensors triggered.");
        return true;
    }
    return false;
}

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

void SensorManager::plotResults()
{
    if (bleManager == nullptr)
        return;

    bleManager->sendLog("\n📍 Final Sensor Data (Recap):");
    bleManager->sendLog("────────────────────────────────────────────");
    bleManager->sendLog("📍 Sensor |   Time (s)   |  Position (cm)");
    bleManager->sendLog("────────────────────────────────────────────");

    Serial.println("\n📍 Final Sensor Data (Recap):");
    Serial.println("────────────────────────────────────────────");
    Serial.println("📍 Sensor |   Time (s)   |  Position (cm)");
    Serial.println("────────────────────────────────────────────");

    char logBuffer[100];
    for (int i = 0; i < numSensors; i++)
    {
        if (triggered[i])
        {
            snprintf(logBuffer, sizeof(logBuffer), "   %d      |   %+9.6f  |   %+7.2f",
                     i, ((double)timestamps[i] - (double)centralSensorTime) / 1.0e6, positions[i]);
            Serial.println(logBuffer);
            bleManager->sendLog(logBuffer);
        }
    }
}

void SensorManager::reset()
{
    for (int i = 0; i < numSensors; i++)
    {
        timestamps[i] = 0;
        triggered[i] = false;
    }
    expectedSensor = 0;
    firstSensorTime = 0;
    lastTriggerTime = 0;
    lastSensorTime = 0;
}

int SensorManager::getDegree()
{
    return 4;
}

unsigned long SensorManager::getFirstSensorTime()
{
    return firstSensorTime;
}

unsigned long SensorManager::getCentralSensorTime()
{
    return centralSensorTime;
}

unsigned long SensorManager::getLastSensorTime()
{
    return lastSensorTime;
}

unsigned long SensorManager::getLastTriggerTime()
{
    return lastTriggerTime;
}

bool SensorManager::getImportantSensorsTriggered()
{
    return triggered[4] && (triggered[5] || triggered[6] || triggered[7] || triggered[8]) && (triggered[0] || triggered[1] || triggered[2] || triggered[3] ); // Check if the first, middle, and last sensors are triggered
}
