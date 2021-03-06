
#include <AiEsp32RotaryEncoder.h>
#include <AiEsp32RotaryEncoderNumberSelector.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "SSD1306Wire.h"
#include <Wire.h>

#include "font.h"

#define SERVICE_UUID "0fdb887e-6150-11ec-90d6-0242ac120003"
#define LED_BRIGHT_CHARACTERISTIC_UUID "223fbac6-6150-11ec-90d6-0242ac120003"

#define LED_PIN 4

#define PWM_FREQ 20000
#define PWM_CHANNEL_LED 0
#define PWM_CHANNEL_WIRE 1
#define PWM_RESOLUTION 16

#define LED_ROTARY_ENCODER_A_PIN 14
#define LED_ROTARY_ENCODER_B_PIN 15
#define LED_ROTARY_ENCODER_BUTTON_PIN 12

#define LED_ROTARY_ENCODER_STEPS 4

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define I2C_SDA 2
#define I2C_SCL 13

SSD1306Wire display (0x3C, I2C_SDA, I2C_SCL, GEOMETRY_128_32);

AiEsp32RotaryEncoder ledRotaryEncoder = AiEsp32RotaryEncoder (
    LED_ROTARY_ENCODER_A_PIN, LED_ROTARY_ENCODER_B_PIN,
    LED_ROTARY_ENCODER_BUTTON_PIN, -1, LED_ROTARY_ENCODER_STEPS);

BLECharacteristic *pLedBrightnessCharacteristic;

uint16_t ledDutyCycle = 0;
bool ledOn = true;
uint16_t wireDutyCycle = 0;
bool wireOn = true;

bool updateDisplay = false;

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
  onWrite (BLECharacteristic *characteristic)
  {
    uint8_t *value = characteristic->getData ();

    if (value != nullptr)
      {
        auto uuidStr = characteristic->getUUID ().toString ();
        if (uuidStr == LED_BRIGHT_CHARACTERISTIC_UUID)
          {
            uint8_t lower = *value;
            value++;
            uint8_t upper = *value;
            ledDutyCycle = ((uint16_t)upper << 8) | lower;
            ledcWrite (PWM_CHANNEL_LED, ledOn ? ledDutyCycle : 0);
            updateDisplay = true;
          }
      }
  }
};

void
refreshDisplay ()
{
  display.clear ();
  display.drawString (
      0, 0,
      "LED: " + String (ledOn ? "ON" : "OFF") + ", "
          + String (static_cast<int> ((ledDutyCycle / 65535.0) * 100))
          + "%\nWire: " + String (wireOn ? "ON" : "OFF") + ", "
          + String (static_cast<int> ((wireDutyCycle / 65535.0) * 100)) + "%");
  display.display ();
}

void
setup ()
{
  Serial.begin (115200);
  Serial.println ("Setup...");

  display.init ();
  display.setFont (Roboto_12);
  refreshDisplay ();

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
      LED_BRIGHT_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR
          | BLECharacteristic::PROPERTY_NOTIFY);
  pLedBrightnessCharacteristic->setCallbacks (new CharacteristicCallbacks ());
  pLedBrightnessCharacteristic->setValue (ledDutyCycle);
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
      ledDutyCycle = ledRotaryEncoder.readEncoder ();
      updateDisplay = true;
      ledcWrite (PWM_CHANNEL_LED, ledOn ? ledDutyCycle : 0);
      pLedBrightnessCharacteristic->setValue (ledDutyCycle);
      pLedBrightnessCharacteristic->notify ();
    }
  if (ledRotaryEncoder.isEncoderButtonClicked ())
    {
      ledOn = !ledOn;
      updateDisplay = true;
      ledcWrite (PWM_CHANNEL_LED, ledOn ? ledDutyCycle : 0);
    }
  if (updateDisplay)
    {
      refreshDisplay ();
      updateDisplay = false;
    }
  delay (25);
}
