#include "BluetoothManager.h"
#include "esp_bt.h"
#include "driver/rtc_io.h"

/**
 * @brief Configures and starts the BLE server, service, characteristics, and advertising.
 * 
 * Called internally by init(). Creates the BLE service and two characteristics: one for data and one for logs.
 * Sets up notification descriptors and defines advertising parameters.
 */
void BluetoothManager::setupServer()
{
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(*this));

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create data characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);

    // Create log characteristic
    pLogCharacteristic = pService->createCharacteristic(
        LOG_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);

    // Add descriptors for notifications
    pCharacteristic->addDescriptor(new BLE2902());
    pLogCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Configure advertising with longer intervals
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinInterval(0x20); // Faster advertising interval for better discovery
    pAdvertising->setMaxInterval(0x40);
    startAdvertising();
}

/**
 * @brief Initializes the BLE device and sets up the server.
 * 
 * @param deviceName The BLE name to be advertised.
 */
void BluetoothManager::init(const char *deviceName)
{

    // Initialize BLE with reduced power
    BLEDevice::init(deviceName);

    // Create the BLE Server with retry logic
    setupServer();
}

/**
 * @brief Starts advertising the BLE service if not already advertising.
 * 
 * Ensures the ESP32 is discoverable by BLE clients when needed.
 */
void BluetoothManager::startAdvertising()
{
    if (!isAdvertising)
    {
        BLEDevice::startAdvertising();
        isAdvertising = true;
        // Serial.println("📡 Advertising started.");
    }
    else
    {
        // Serial.println("ℹ️ Already advertising.");
    }
}

/**
 * @brief Sends formatted sensor data over BLE as a JSON string.
 * 
 * @param positionError Distance value or inferred position.
 * @param timeError Timestamp of button press. As time 0 is the crossing, buttonTime is timing error.
 * @param startTime Timestamp when IMU detected start of the step.
 * @param endTime Timestamp when IMU detected end of the step.
 * 
 * If no device is connected, the function will trigger advertising instead.
 */
void BluetoothManager::sendData(double positionError, double timeError, double startTime, double endTime)
{
    if (!deviceConnected)
    {
        // If not connected, ensure we're advertising
        startAdvertising();
        return;
    }

    // Format data as JSON with minimal precision to reduce packet size
    char jsonData[150]; // aumenta un poco el tamaño para evitar overflow
    snprintf(jsonData, sizeof(jsonData),
             "{\"t\":%lu,\"p\":%.1f,\"b\":%.3f,\"s\":%.3f,\"e\":%.3f}",
             micros(), positionError, timeError, startTime, endTime);

    pCharacteristic->setValue(jsonData);
    pCharacteristic->notify();
}

/**
 * @brief Sends a timestamped log message over BLE. This was used as a debugging tool to have a terminal-like output in the app.
 * 
 * @param message The message to send.
 * 
 * If no device is connected, the message is discarded.
 */
void BluetoothManager::sendLog(const char *message)
{
    if (!deviceConnected)
    {
        return;
    }

    // Add timestamp to the log message
    char timestampedLog[256];
    snprintf(timestampedLog, sizeof(timestampedLog),
             "[%.3f] %s",
             millis() / 1000.0,
             message);

    // Add delay between transmissions

    pLogCharacteristic->setValue(timestampedLog);
    pLogCharacteristic->notify();
}

/**
 * @brief Callback triggered when a device connects to the BLE server.
 * 
 * Increments the connection count and stops advertising if the maximum number of connections is reached.
 */
void BluetoothManager::onConnect()
{
    deviceConnected = true;
    connectedCount++;
    // Serial.printf("🔗 Device connected. Total connections: %d\n", connectedCount);

    if (connectedCount >= MAX_CONNECTIONS)
    {
        BLEDevice::stopAdvertising();
        isAdvertising = false;
        // Serial.println("🛑 Stopped advertising (max connections reached).");
    }
}

/**
 * @brief Callback triggered when a device disconnects from the BLE server.
 * 
 * Decrements the connection count and restarts advertising if below the maximum allowed connections.
 */
void BluetoothManager::onDisconnect()
{
    // Serial.printf("❌ Device disconnected. Connections before decrement: %d\n", connectedCount);

    if (connectedCount > 0)
    {
        connectedCount--;
    }

    deviceConnected = connectedCount > 0;
    isAdvertising = false;

    // Serial.printf("🔁 Remaining connections: %d | Advertising status: %s\n", connectedCount, isAdvertising ? "YES" : "NO");

    if (connectedCount < MAX_CONNECTIONS)
    {
        delay(500);
        // Serial.println("🔃 Restarting advertising from onDisconnect...");
        startAdvertising();
    }
}

/**
 * @brief Checks if at least one device is currently connected.
 * 
 * @return true if connected, false otherwise.
 */
bool BluetoothManager::isConnected() const
{
    return deviceConnected;
}

/**
 * @brief Checks if the ESP32 is currently advertising its BLE service.
 * 
 * @return true if advertising, false otherwise.
 */
bool BluetoothManager::isCurrentlyAdvertising() const
{
    return isAdvertising;
}
