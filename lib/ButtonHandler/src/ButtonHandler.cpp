#include "ButtonHandler.h"

void ButtonHandler::init()
{
    pinMode(buttonPin, INPUT);
}

void ButtonHandler::setBluetoothManager(BluetoothManager &btMgr)
{
    this->bleManager = &btMgr;
}

double ButtonHandler::getButtonTime()
{
    return ((double)testTimestamp - (double)sensorManager->getCentralSensorTime()) / 1.0e6;
}

void ButtonHandler::checkButton()
{
    if (sensorManager == nullptr || bleManager == nullptr)
    {
        Serial.println("SensorManager or BluetoothManager not set!");
        return;
    }
    bool currentButtonState = digitalRead(buttonPin);

    if (currentButtonState == HIGH && lastState == LOW)
    // if (currentButtonState == LOW && !testPressed)

    {
        testTimestamp = micros();
        testPressed = true;

        double buttonTime = ((double)testTimestamp - (double)sensorManager->getCentralSensorTime()) / 1.0e6;
        char logBuffer[100];
        snprintf(logBuffer, sizeof(logBuffer), "🔘 Button Pressed at: %.2f s", buttonTime);
        Serial.println(logBuffer);
        bleManager->sendLog(logBuffer);
    }
    lastState = currentButtonState;
}

bool ButtonHandler::wasPressed()
{
    return testPressed;
}

void ButtonHandler::reset()
{
    testTimestamp = 0;
    testPressed = false;
}

void ButtonHandler::plotResults()
{
    if (testPressed && polyFit != nullptr && sensorManager != nullptr && bleManager != nullptr)
    {
        double buttonRelTime = ((double)testTimestamp - (double)sensorManager->getCentralSensorTime()) / 1.0e6;
        char logBuffer[100];
        snprintf(logBuffer, sizeof(logBuffer), "🟢 Button | %+9.6f s | %+7.2f cm", buttonRelTime, position);
        Serial.println(logBuffer);
        bleManager->sendLog(logBuffer);
    }
}

void ButtonHandler::setPolynomialFit(PolynomialFit &poly)
{
    this->polyFit = &poly;
}

void ButtonHandler::setSensorManager(SensorManager &sensorMgr)
{
    this->sensorManager = &sensorMgr;
}

void ButtonHandler::setPosition(double pos)
{
    this->position = pos;
}
