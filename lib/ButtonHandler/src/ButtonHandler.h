#ifndef BUTTONHANDLER_H
#define BUTTONHANDLER_H

#include <Arduino.h>
#include "PolynomialFit.h"
#include "SensorManager.h"
#include "BluetoothManager.h"

class ButtonHandler {
public:
    void init();
    void checkButton();
    bool wasPressed();
    void plotResults();
    void reset();
    void setPolynomialFit(PolynomialFit &poly);
    void setSensorManager(SensorManager &sensorMgr);
    void setBluetoothManager(BluetoothManager &btMgr);
    double getButtonTime();
    void setPosition(double pos);
    double getPosition() const { return position; }

private:
    const int buttonPin = 34;
    volatile unsigned long testTimestamp = 0;
    volatile bool lastState = true;
    volatile double position = 0.0;
    volatile bool testPressed = false;
    PolynomialFit *polyFit = nullptr;
    SensorManager *sensorManager = nullptr;
    BluetoothManager *bleManager = nullptr;
};

#endif
