// --- AccelerometerHW123.h ---
#ifndef ACCELEROMETER_HW123_H
#define ACCELEROMETER_HW123_H

#include <Wire.h>
#include <Arduino.h>
#include "BluetoothManager.h"
#include "SensorManager.h"

// --- Detection thresholds ---
#define MIN_THRESHOLD -30.0f      // Toe-off threshold (min gyroZ)
#define POS_SWING_THRES +100.0f // Heel-strike zero crossing

class AccelerometerHW123 {
public:
    // --- Constructor & Initialization ---
    AccelerometerHW123(uint8_t addr);
    bool begin();
    bool isDataReady();

    // --- Sensor Reading ---
    bool readAcceleration(float &ax, float &ay, float &az);
    bool readGyro(float &gyroX, float &gyroY, float &gyroZ);
    
    // --- Step Detection ---
    void updateShockState(float gyroY);
    void reset();

    // --- State Getters ---
    bool hasEndTime();
    unsigned long getStartTime();
    unsigned long getEndTime();

    // --- Integration ---
    void setBluetoothManager(BluetoothManager &btMgr);
    void setSensorManager(SensorManager &snsMgr);

private:
    // --- Low-level Register I/O ---
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

    // --- Device Address ---
    uint8_t _addr;

    // --- State Variables ---
    static unsigned long startTime;
    static unsigned long endTime;

    // --- External Managers ---
    BluetoothManager* bleManager = nullptr;
    SensorManager* sensorManager = nullptr;
};
#endif
