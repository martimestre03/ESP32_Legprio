#include "AccelerometerHW123.h"

#define STEP_START_THRESHOLD_GZ -40.0f      // Toe-off threshold (min gyroZ)
#define STEP_END_THRESHOLD_GZ_CROSSING 0.0f // Heel-strike zero crossing

AccelerometerHW123::AccelerometerHW123(uint8_t addr) : _addr(addr) {}

bool AccelerometerHW123::begin()
{
    Wire.begin();
    Wire.setClock(100000);
    writeRegister(0x6B, 0x00); // Wake up
    delay(100);
    writeRegister(0x38, 0x01); // Enable Data Ready interrupt
    uint8_t id = readRegister(0x75);
    return id == 0x68;
}

bool AccelerometerHW123::readAcceleration(float &ax, float &ay, float &az)
{
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0)
        return false;
    Wire.requestFrom((int)_addr, 6);
    if (Wire.available() < 6)
        return false;
    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;
    return true;
}

bool AccelerometerHW123::readGyro(float &gx, float &gy, float &gz)
{
    Wire.beginTransmission(_addr);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0)
        return false;
    Wire.requestFrom((int)_addr, 6);
    if (Wire.available() < 6)
        return false;
    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    gx = rawX / 131.0;
    gy = rawY / 131.0;
    gz = rawZ / 131.0;
    return true;
}

bool AccelerometerHW123::isDataReady()
{
    return (readRegister(0x3A) & 0x01);
}

void AccelerometerHW123::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

uint8_t AccelerometerHW123::readRegister(uint8_t reg)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((int)_addr, 1);
    if (Wire.available())
        return Wire.read();
    return 0;
}

void AccelerometerHW123::checkShock(float ax, float ay, float az, float &totalAccel, float &avgAccel)
{
    static const int windowSize = 4;
    static float window[windowSize] = {0};
    static int index = 0;
    static bool windowFilled = false;
    totalAccel = sqrt(ax * ax + ay * ay + (az + 1) * (az + 1));
    window[index] = totalAccel;
    index = (index + 1) % windowSize;
    if (index == 0)
        windowFilled = true;
    avgAccel = windowFilled ? (window[0] + window[1] + window[2] + window[3]) / 4.0 : totalAccel;
}

static int callCounter = 0;

void AccelerometerHW123::updateShockState(float ax, float ay, float az, float gyroX)
{
    static float accelBuffer[4] = {0};
    static int accelIndex = 0;
    static float lastGyroX = 0;
    static bool waitingForNextMin = true;

    static float lastMinGyro = 0;
    static unsigned long minGyroTime = 0;

    static bool lastWasEnd = true;

    static unsigned long lastEventTime = 0; // Time of last start or end

    float totalAccel = sqrt(ax * ax + ay * ay + az * az);

    accelBuffer[accelIndex] = totalAccel;
    accelIndex = (accelIndex + 1) % 4;

    float avgAccel = 0;
    for (int i = 0; i < 4; i++)
        avgAccel += accelBuffer[i];
    avgAccel /= 4.0;

    unsigned long now = micros();

    // --- Detect minimum in gyroX ---
    if (gyroX < -150 && waitingForNextMin && gyroX < lastGyroX)
    {
        lastMinGyro = gyroX;
        minGyroTime = now;
    }

    // --- Step detection on upward slope ---
    if (waitingForNextMin && lastGyroX < gyroX && lastMinGyro < -150)
    {
        char logBuffer[100];

        Serial.println("Min slope detected, checking for the event");
        // --- Timeout force start ---
        if (now - lastEventTime > 2e6)
        {
            startTime = now;
            lastEventTime = now;
            lastWasEnd = false;
            waitingForNextMin = false; // Prevent overlap with normal detection
            snprintf(logBuffer, sizeof(logBuffer), "⏱️🟢 Step Start (timeout) at %.6f s (%lu µs)\n", startTime / 1e6, startTime);
            Serial.println(logBuffer);
            // bleManager->sendLog(logBuffer);
        }

        else if (avgAccel > 1.75)
        {
            endTime = minGyroTime;
            endDetected = true;
            lastWasEnd = true;
            lastEventTime = endTime;
            snprintf(logBuffer, sizeof(logBuffer), "🛑 Step End (forced) at %.6f s (%lu µs), AccelAvg: %.2f\n", endTime / 1e6, endTime, avgAccel);
            Serial.println(logBuffer);
            // bleManager->sendLog(logBuffer);
        }
        else if (lastWasEnd)
        {
            startTime = minGyroTime;
            lastWasEnd = false;
            lastEventTime = startTime;
            Serial.printf("🟢 Step Start at %.6f s (%lu µs), AccelAvg: %.2f\n", startTime / 1e6, startTime, avgAccel);
        }
        else
        {
            endTime = minGyroTime;
            endDetected = true;
            lastWasEnd = true;
            lastEventTime = endTime;
            snprintf(logBuffer, sizeof(logBuffer), "🛑 Step End at %.6f s (%lu µs), AccelAvg: %.2f\n", endTime / 1e6, endTime, avgAccel);
            Serial.println(logBuffer);
            // bleManager->sendLog(logBuffer);
        }

        waitingForNextMin = false;
    }

    if (gyroX > -150)
    {
        waitingForNextMin = true;
        lastMinGyro = 0;
    }

    lastGyroX = gyroX;
}

bool AccelerometerHW123::isFootRelaxed() { return footRelaxed; }
bool AccelerometerHW123::isStepStarted() { return stepStarted; }
bool AccelerometerHW123::hasEndTime() { return endDetected; }
unsigned long AccelerometerHW123::getStartTime() { return startTime; }
unsigned long AccelerometerHW123::getEndTime() { return endTime; }
unsigned long AccelerometerHW123::getDefStartTime() { return defStartTime; }

void AccelerometerHW123::resetStates()
{
    stepStarted = false;
    endDetected = false;
    footRelaxed = false;
    minGyroY = 0;
    lastGyroY = 0;
}

void AccelerometerHW123::reset()
{
    startTime = 0;
    endTime = 0;
    resetStates();
}

void AccelerometerHW123::resetEndTime()
{
    endDetected = false;
}

void AccelerometerHW123::setBluetoothManager(BluetoothManager &btMgr)
{
    bleManager = &btMgr;
}