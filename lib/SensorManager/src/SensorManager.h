#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include <vector>
#include "BluetoothManager.h"

class SensorManager {
public:
    void init();
    void checkSensors();
    bool shouldProcessData();
    std::tuple<unsigned long*, double*>  collectData(std::vector<double> &timeData, std::vector<double> &positionData);
    void reset();
    void plotResults();
    int getDegree();
    unsigned long getFirstSensorTime();
    unsigned long getCentralSensorTime();
    unsigned long getLastSensorTime();
    unsigned long getLastTriggerTime();
    bool getImportantSensorsTriggered();
    void setBluetoothManager(BluetoothManager &btMgr);

private:
    static const int numSensors = 9;
    static const int sensorPins[numSensors];
    static const double sensorSpacing;
    
    unsigned long timestamps[numSensors] = {0};
    bool triggered[numSensors] = {false};
    volatile unsigned long firstSensorTime = 0;
    volatile unsigned long centralSensorTime = 0;
    volatile unsigned long lastSensorTime = 0;
    volatile unsigned long lastTriggerTime = 0;
    int expectedSensor = 0;
    BluetoothManager* bleManager = nullptr;
};

#endif
