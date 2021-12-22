
#include <AiEsp32RotaryEncoder.h>
#include <AiEsp32RotaryEncoderNumberSelector.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define SERVICE_UUID "0fdb887e-6150-11ec-90d6-0242ac120003"
#define CHARACTERISTIC_UUID "223fbac6-6150-11ec-90d6-0242ac120003"

#define LED_PIN 4

#define PWM_FREQ 20000
#define PWM_CHANNEL_LED 0
#define PWM_CHANNEL_WIRE 1
#define PWM_RESOLUTION 16

#define LED_ROTARY_ENCODER_A_PIN 14
#define LED_ROTARY_ENCODER_B_PIN 2
#define LED_ROTARY_ENCODER_BUTTON_PIN 15

#define LED_ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder ledRotaryEncoder = AiEsp32RotaryEncoder (
    LED_ROTARY_ENCODER_A_PIN, LED_ROTARY_ENCODER_B_PIN,
    LED_ROTARY_ENCODER_BUTTON_PIN, -1, LED_ROTARY_ENCODER_STEPS);

BLECharacteristic *pLedBrightnessCharacteristic;

// clang-format off
void IRAM_ATTR ledReadEncoderISR ()
{
  ledRotaryEncoder.readEncoder_ISR ();
}
// clang-format on

class ServerCallbacks : public BLEServerCallbacks
{
  void
  onConnect (BLEServer *pServer)
  {
    Serial.println ("Client Connected...");
  }

  void
  onDisconnect (BLEServer *pServer)
  {
    Serial.println ("Client Disconnected...");
    BLEDevice::startAdvertising ();
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void
  onWrite (BLECharacteristic *pLedBrightnessCharacteristic)
  {
    uint8_t *value = pLedBrightnessCharacteristic->getData ();

    if (value != nullptr)
      {
        uint8_t lower = *value;
        value++;
        uint8_t upper = *value;
        uint16_t fullValue = ((uint16_t)upper << 8) | lower;
        ledcWrite (PWM_CHANNEL_LED, fullValue);
      }
  }
};

void
setup ()
{
  Serial.begin (115200);
  Serial.println ("Starting BLE work!");

  ledRotaryEncoder.begin ();
  ledRotaryEncoder.setup (ledReadEncoderISR);
  ledRotaryEncoder.setBoundaries (0, 0xffff, false);
  ledRotaryEncoder.setAcceleration (100000);

  pinMode (LED_PIN, OUTPUT);
  ledcSetup (PWM_CHANNEL_LED, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin (LED_PIN, PWM_CHANNEL_LED);

  BLEDevice::init ("Cutter");
  BLEServer *pServer = BLEDevice::createServer ();
  pServer->setCallbacks (new ServerCallbacks ());

  BLEService *pService = pServer->createService (SERVICE_UUID);

  pLedBrightnessCharacteristic = pService->createCharacteristic (
      CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ
                               | BLECharacteristic::PROPERTY_WRITE_NR
                               | BLECharacteristic::PROPERTY_NOTIFY);
  pLedBrightnessCharacteristic->setCallbacks (new CharacteristicCallbacks ());
  uint16_t startValue = 0;
  pLedBrightnessCharacteristic->setValue (startValue);
  BLEDescriptor *pClientCharacteristicConfigDescriptor
      = new BLEDescriptor ((uint16_t)0x2902);
  pLedBrightnessCharacteristic->addDescriptor (
      pClientCharacteristicConfigDescriptor);

  pService->start ();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising ();
  pAdvertising->addServiceUUID (SERVICE_UUID);
  pAdvertising->setScanResponse (true);
  pAdvertising->setMinPreferred (
      0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred (0x12);
  BLEDevice::startAdvertising ();

  Serial.println ("BLE Setup done!");
}

void
loop ()
{
  if (ledRotaryEncoder.encoderChanged ())
    {
      uint16_t val = ledRotaryEncoder.readEncoder ();
      ledcWrite (PWM_CHANNEL_LED, val);
      pLedBrightnessCharacteristic->setValue (val);
      pLedBrightnessCharacteristic->notify ();
    }
  delay (25);
}
