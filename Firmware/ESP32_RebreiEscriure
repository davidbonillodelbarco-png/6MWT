#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID         "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID_TX "87654321-4321-4321-4321-210987654321"  // ESP32 → Móvil
#define CHARACTERISTIC_UUID_RX "11223344-5566-7788-9900-aabbccddeeff"  // Móvil → ESP32

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristicTX = nullptr;
BLECharacteristic* pCharacteristicRX = nullptr;
bool deviceConnected = false;

// --- Callback para recibir mensajes del móvil ---
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String valor = pCharacteristic->getValue().c_str();
    if (valor.length() > 0) {
      Serial.println("Mensaje recibido del móvil: " + valor);
    }
  }
};

// --- Callback para conexión/desconexión ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Cliente conectado");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Cliente desconectado");
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);

  BLEDevice::init("ESP32_Grup4");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Característica TX: ESP32 → Móvil (NOTIFY)
  pCharacteristicTX = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristicTX->addDescriptor(new BLE2902());

  // Característica RX: Móvil → ESP32 (WRITE)
  pCharacteristicRX = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristicRX->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("ESP32_Grup4 listo. Esperando conexión...");
}

void loop() {
  if (deviceConnected) {
    // Envía mensaje al móvil cada 2 segundos
    String mensaje = "Hola desde ESP32";
    pCharacteristicTX->setValue(mensaje.c_str());
    pCharacteristicTX->notify();
    Serial.println("Enviado al móvil: " + mensaje);
    delay(2000);
  }
}
