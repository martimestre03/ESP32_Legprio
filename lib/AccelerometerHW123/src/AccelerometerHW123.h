// --- AccelerometerHW123.h ---
#ifndef ACCELEROMETER_HW123_H
#define ACCELEROMETER_HW123_H

#include <Wire.h>
#include <Arduino.h>
#include "BluetoothManager.h"

class AccelerometerHW123 {
public:
    AccelerometerHW123(uint8_t addr);
    bool begin();
    bool readAcceleration(float &ax, float &ay, float &az);
    bool readGyro(float &gyroX, float &gyroY, float &gyroZ);
    bool isDataReady();
    void printAcceleration();

    // Step event detection logic
    void updateShockState(float ax, float ay, float az, float gyroY);
    void resetStates();
    void reset();
    void resetEndTime();

    // State getters
    bool isFootRelaxed();
    bool isStepStarted();
    bool hasEndTime();
    unsigned long getStartTime();
    unsigned long getEndTime();
    unsigned long getDefStartTime();

    void setBluetoothManager(BluetoothManager &btMgr);


private:
    uint8_t _addr;

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

    void checkShock(float ax, float ay, float az, float &totalAccel, float &avgAccel);

    // Step detection state
    unsigned long startTime = 0;
    unsigned long endTime = 0;
    unsigned long defStartTime = 0;
    bool stepStarted = false;
    bool endDetected = false;
    bool footRelaxed = false;

    // Gyro-based detection state
    float lastGyroY = 0.0f;
    float minGyroY = 0.0f;
    unsigned long minGyroYTime = 0;
    BluetoothManager* bleManager = nullptr;

};

#endif
