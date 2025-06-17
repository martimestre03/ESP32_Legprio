#ifndef BUTTONHANDLER_H
#define BUTTONHANDLER_H

#include <Arduino.h>
#include "PolynomialFit.h"
#include "SensorManager.h"
#include "BluetoothManager.h"

class ButtonHandler
{
public:
    // --- Init & Runtime ---
    void init();
    void checkButton();
    bool wasPressed();
    void reset();
    
    // --- Data Access ---
    double getButtonTime();
    void plotResults();
    void setPosition(double pos);
    
    // --- External Modules --
    void setSensorManager(SensorManager &sensorMgr);
    void setPolynomialFit(PolynomialFit &poly);
    void setBluetoothManager(BluetoothManager &btMgr);

private:
    // --- Pin & State ---
    const int buttonPin = 34;
    volatile unsigned long testTimestamp = 0;
    volatile bool lastState = true;
    volatile double position = 0.0;
    volatile bool testPressed = false;

    // --- External Modules ---
    PolynomialFit *polyFit = nullptr;
    SensorManager *sensorManager = nullptr;
    BluetoothManager *bleManager = nullptr;
};

#endif
