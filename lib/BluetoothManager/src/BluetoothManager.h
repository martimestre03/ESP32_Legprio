#ifndef BLUETOOTHMANAGER_H
#define BLUETOOTHMANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class BluetoothManager {
public:
    void init(const char* deviceName = "LegPrio");
    void sendData(double position, double buttonTime, double startTime, double endTime);
    void sendLog(const char* message); // New method to send logs
    bool isConnected() { return deviceConnected; }
    void onConnect();
    void onDisconnect();
    bool isConnected() const;
    bool isCurrentlyAdvertising() const;
    void startAdvertising();

private:
    void setupServer();

    BLEServer* pServer = nullptr;
    BLECharacteristic* pCharacteristic = nullptr;
    BLECharacteristic* pLogCharacteristic = nullptr; // New characteristic for logs
    bool deviceConnected = false;
    bool isAdvertising = false;
    uint8_t connectedCount = 0;
    static const uint8_t MAX_CONNECTIONS = 3;  // Maximum number of simultaneous connections
    
    // UUIDs for the BLE service and characteristic
    const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    const char* CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    const char* LOG_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9"; // New UUID for logs
    
    class ServerCallbacks: public BLEServerCallbacks {
        private:
            BluetoothManager& manager;
        public:
            ServerCallbacks(BluetoothManager& mgr) : manager(mgr) {}
            void onConnect(BLEServer* pServer) override { manager.onConnect(); }
            void onDisconnect(BLEServer* pServer) override { manager.onDisconnect(); }
    };
};

#endif