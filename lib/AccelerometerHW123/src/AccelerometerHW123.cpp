#include "AccelerometerHW123.h"



unsigned long AccelerometerHW123::startTime = 0;
unsigned long AccelerometerHW123::endTime = 0;
/**
 * @brief Constructor that sets the I2C address of the MPU6050.
 *
 * @param addr I2C address of the accelerometer/gyroscope.
 */
AccelerometerHW123::AccelerometerHW123(uint8_t addr) : _addr(addr) {}

/**
 * @brief Initializes the MPU6050 sensor.
 *
 * This function sets up the I2C communication, wakes up the sensor, and checks if the sensor is connected.
 *
 * @return true if the sensor is initialized successfully, false otherwise.
 */
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

/**
 * @brief Reads acceleration data from the MPU6050.
 *
 * This function reads the raw acceleration values from the sensor and converts them to g's.
 *
 * @param ax Reference to store the X-axis acceleration in g's.
 * @param ay Reference to store the Y-axis acceleration in g's.
 * @param az Reference to store the Z-axis acceleration in g's.
 * @return true if the data is read successfully, false otherwise.
 */
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

/**
 * @brief Reads gyroscope data from the MPU6050.
 *
 * This function reads the raw gyroscope values from the sensor and converts them to degrees per second.
 *
 * @param gx Reference to store the X-axis gyro value in degrees per second.
 * @param gy Reference to store the Y-axis gyro value in degrees per second.
 * @param gz Reference to store the Z-axis gyro value in degrees per second.
 * @return true if the data is read successfully, false otherwise.
 */
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

/**
 * @brief Checks if new data is available from the MPU6050.
 *
 * This function checks the Data Ready bit in the MPU6050 status register.
 *
 * @return true if new data is ready, false otherwise.
 */
bool AccelerometerHW123::isDataReady()
{
    return (readRegister(0x3A) & 0x01);
}

/**
 * @brief Writes a value to a specific register on the MPU6050.
 *
 * @param reg Register address to write to.
 * @param value Byte value to write.
 */
void AccelerometerHW123::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

/**
 * @brief Reads a value from a specific register on the MPU6050.
 *
 * @param reg Register address to read from.
 * @return Byte value read from the register.
 */
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

/**
 * @brief Updates the shock state based on the gyroscope X-axis data.
 *
 * This function implements a moving average smoother and detects step start and end events based on gyroscope data.
 *
 * @param gyroX The X-axis gyroscope value in degrees per second.
 */
void AccelerometerHW123::updateShockState(float gyroX)
{
    /* ---------- 0.  Moving‑average smoother -------------------- */
    const int SMOOTH_WINDOW = 10; // tune as needed
    static float smoothBuf[SMOOTH_WINDOW] = {0};
    static int smoothIdx = 0;
    static float smoothSum = 0;

    smoothSum -= smoothBuf[smoothIdx];
    smoothBuf[smoothIdx] = gyroX;
    smoothSum += gyroX;
    smoothIdx = (smoothIdx + 1) % SMOOTH_WINDOW;
    float smoothedGyro = smoothSum / SMOOTH_WINDOW;

    /* ---------- 1.  Persistent state & thresholds -------------- */
    static float lastGyroX = 0.0f;     // last *smoothed* value
    static bool wasFalling = false;    // slope tracker
    static bool positiveSwing = false; // > +50 °/s flag

    char logBuffer[100];
    unsigned long now = micros();

    /* ---------- 2.  Positive‑swing detection (raw) ------------- */
    if (gyroX > POS_SWING_THRES)
        positiveSwing = true;

    /* ---------- 3.  Minimum detection (smoothed) --------------- */
    bool isRising = smoothedGyro > lastGyroX;
    if (wasFalling && isRising && lastGyroX < MIN_THRESHOLD)
    {
        if (positiveSwing) // → Step End
        {
            endTime = now;

            snprintf(logBuffer, sizeof(logBuffer),
                     "🛑 Step End at %.6f s, smoothed valley %.1f °/s",
                     endTime / 1e6, lastGyroX);
        }
        else if (now - endTime > 500000) // → Step Start
        {
            startTime = now;
            if (sensorManager)
                sensorManager->reset();

            snprintf(logBuffer, sizeof(logBuffer),
                     "🟢 Step Start at %.6f s, smoothed valley %.1f °/s",
                     startTime / 1e6, lastGyroX);
        }

        // Serial.println(logBuffer);
        // if (bleManager) bleManager->sendLog(logBuffer);

        /* reset for next cycle */
        positiveSwing = false;
    }

    /* ---------- 4.  Prepare for next sample -------------------- */
    wasFalling = (smoothedGyro < lastGyroX); // still descending?
    lastGyroX = smoothedGyro;
}


/**
 * @brief Resets the step detection states and timestamps.
 *
 * This function resets the start and end times, and clears the step detection states.
 */
void AccelerometerHW123::reset()
{
    startTime = 0;
    endTime = 0;
}

/**
 * @brief Sets the Bluetooth manager for sending data.
 *
 * This function allows the accelerometer to send data through the Bluetooth manager.
 *
 * @param btMgr Reference to the BluetoothManager instance.
 */
void AccelerometerHW123::setBluetoothManager(BluetoothManager &btMgr)
{
    bleManager = &btMgr;
}

/**
 * @brief Sets the sensor manager for managing sensor data.
 *
 * This function allows the accelerometer to interact with the SensorManager instance.
 *
 * @param snsMgr Reference to the SensorManager instance.
 */
void AccelerometerHW123::setSensorManager(SensorManager &snsMgr)
{
    sensorManager = &snsMgr;
}

unsigned long AccelerometerHW123::getStartTime() { return startTime; }
unsigned long AccelerometerHW123::getEndTime() { return endTime; }
