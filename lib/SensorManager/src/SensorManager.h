#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include <vector>
#include "BluetoothManager.h"

class SensorManager {
public:
    // --- Core Logic ---
    void init();
    void checkSensors();
    bool shouldProcessData();
    std::tuple<unsigned long*, double*>  collectData(std::vector<double> &timeData, std::vector<double> &positionData);
    void reset();

    // --- Output ---
    void plotResults();

    // --- State Access ---
    unsigned long getCentralSensorTime();
    unsigned long getLastTriggerTime();
    bool getImportantSensorsTriggered();

    // --- External Modules --
    void setBluetoothManager(BluetoothManager &btMgr);

private:
    // --- Sensor Config ---
    static const int numSensors = 9;
    static const int sensorPins[numSensors];
    static const double sensorSpacing;
    
    // --- Internal State ---
    unsigned long timestamps[numSensors] = {0};
    bool triggered[numSensors] = {false};
    volatile unsigned long centralSensorTime = 0;
    volatile unsigned long lastTriggerTime = 0;
    int expectedSensor = 0;

    // --- External Modules --
    BluetoothManager* bleManager = nullptr;
};

#endif
