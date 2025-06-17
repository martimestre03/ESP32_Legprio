#include "ButtonHandler.h"

/**
 * @brief Initializes the button pin as input.
 */
void ButtonHandler::init()
{
    pinMode(buttonPin, INPUT);
}

/**
 * @brief Returns the time difference between button press and central sensor in seconds.
 * 
 * @return Relative time in seconds.
 */
double ButtonHandler::getButtonTime()
{
    return ((double)testTimestamp - (double)sensorManager->getCentralSensorTime()) / 1.0e6;
}

/**
 * @brief Monitors the digital button state and records press events.
 */
void ButtonHandler::checkButton()
{
    if (sensorManager == nullptr || bleManager == nullptr)
    {
        // Serial.println("SensorManager or BluetoothManager not set!");
        return;
    }

    bool currentButtonState = digitalRead(buttonPin);
    Serial.printf("%lu,%d\n", micros(), currentButtonState);

    // Detect initial press
    if (currentButtonState == HIGH && lastState == LOW)
    {
        testTimestamp = micros();   // your original logic
        testPressed = true;         // your original logic

        // holdStartTime = micros();   // new logic for hold tracking
        // holdTracking = true;
    }

    lastState = currentButtonState;
}

/**
 * @brief Returns whether the button was pressed since the last reset.
 * 
 * @return true if pressed, false otherwise.
 */
bool ButtonHandler::wasPressed()
{
    return testPressed;
}

/**
 * @brief Resets internal button state and timestamp.
 */
void ButtonHandler::reset()
{
    testTimestamp = 0;
    testPressed = false;
}

/**
 * @brief Logs and formats the result if a button press was detected.
 */
void ButtonHandler::plotResults()
{
    if (testPressed && polyFit != nullptr && sensorManager != nullptr && bleManager != nullptr)
    {
        double buttonRelTime = ((double)testTimestamp - (double)sensorManager->getCentralSensorTime()) / 1.0e6;
        char logBuffer[100];
        // snprintf(logBuffer, sizeof(logBuffer), "🟢B\t| %+9.3f\t| %+7.2f", buttonRelTime, position);
        // Serial.println(logBuffer);
        // bleManager->sendLog(logBuffer);
    }
}

/**
 * @brief Sets the PolynomialFit object used for result estimation.
 * 
 * @param poly Reference to PolynomialFit instance.
 */
void ButtonHandler::setPosition(double pos)
{
    this->position = pos;
}

/**
 * @brief Sets the PolynomialFit object used for result estimation.
 * 
 * @param poly Reference to PolynomialFit instance.
 */
void ButtonHandler::setPolynomialFit(PolynomialFit &poly)
{
    this->polyFit = &poly;
}

/**
 * @brief Sets the SensorManager instance used for time alignment.
 * 
 * @param sensorMgr Reference to SensorManager.
 */
void ButtonHandler::setSensorManager(SensorManager &sensorMgr)
{
    this->sensorManager = &sensorMgr;
}

/**
 * @brief Sets the Bluetooth manager reference.
 * 
 * @param btMgr Reference to BluetoothManager.
 */
void ButtonHandler::setBluetoothManager(BluetoothManager &btMgr)
{
    this->bleManager = &btMgr;
}