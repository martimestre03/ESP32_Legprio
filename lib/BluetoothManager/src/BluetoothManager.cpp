#include "BluetoothManager.h"
#include "esp_bt.h"
#include "driver/rtc_io.h"

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

void BluetoothManager::init(const char *deviceName)
{

    // Initialize BLE with reduced power
    BLEDevice::init(deviceName);

    // Create the BLE Server with retry logic
    setupServer();
}

void BluetoothManager::startAdvertising()
{
    if (!isAdvertising)
    {
        BLEDevice::startAdvertising();
        isAdvertising = true;
        Serial.println("📡 Advertising started.");
    }
    else
    {
        Serial.println("ℹ️ Already advertising.");
    }
}

void BluetoothManager::sendData(double position, double buttonTime, double startTime, double endTime)
{
    if (!deviceConnected)
    {
        // If not connected, ensure we're advertising
        startAdvertising();
        return;
    }

    // Format data as JSON with minimal precision to reduce packet size
    char jsonData[100];
    snprintf(jsonData, sizeof(jsonData),
             "{\"p\":%.1f,\"b\":%.3f,\"s\":%.3f,\"e\":%.3f}",
             position, buttonTime, startTime, endTime);

    // Add delay between transmissions
    pCharacteristic->setValue(jsonData);
    pCharacteristic->notify();
}

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

void BluetoothManager::onConnect()
{
    deviceConnected = true;
    connectedCount++;
    Serial.printf("🔗 Device connected. Total connections: %d\n", connectedCount);

    if (connectedCount >= MAX_CONNECTIONS)
    {
        BLEDevice::stopAdvertising();
        isAdvertising = false;
        Serial.println("🛑 Stopped advertising (max connections reached).");
    }
}

void BluetoothManager::onDisconnect()
{
    Serial.printf("❌ Device disconnected. Connections before decrement: %d\n", connectedCount);

    if (connectedCount > 0)
    {
        connectedCount--;
    }

    deviceConnected = connectedCount > 0;
    isAdvertising = false;

    Serial.printf("🔁 Remaining connections: %d | Advertising status: %s\n", connectedCount, isAdvertising ? "YES" : "NO");

    if (connectedCount < MAX_CONNECTIONS)
    {
        delay(500);
        Serial.println("🔃 Restarting advertising from onDisconnect...");
        startAdvertising();
    }
}

bool BluetoothManager::isConnected() const
{
    return deviceConnected;
}

bool BluetoothManager::isCurrentlyAdvertising() const
{
    return isAdvertising;
}
