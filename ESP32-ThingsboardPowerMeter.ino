//  ______ _       _               ______ _           _                   _          
// |  ____| |     | |             |  ____| |         | |                 (_)         
// | |__  | |_   _| | _____       | |__  | | ___  ___| |_ _ __ ___  _ __  _  ___ ___ 
// |  __| | | | | | |/ / _ \      |  __| | |/ _ \/ __| __| '__/ _ \| '_ \| |/ __/ __|
// | |    | | |_| |   <  __/      | |____| |  __/ (__| |_| | | (_) | | | | | (__\__ \
// |_|    |_|\__,_|_|\_\___|      |______|_|\___|\___|\__|_|  \___/|_| |_|_|\___|___/
//
// Youtube channel : Fluke Electronics 
// Facebook page : ฟลุ๊คการไฟฟ้า



// OTA and WiFi
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiClient espClient;
ThingsBoard tb(espClient);
int status = WL_IDLE_STATUS;
bool subscribed = false;
const char *ssid = "injectorhome_2.4GHz";
const char *password = "sirapols0422";

// Thingsboard
#include <ThingsBoard.h>
#define THINGSBOARD_SERVER "192.168.1.25"
#define TOKEN "vA60qMrhtEzIX2LoddSN"


// DHT
#include "DHT.h"
#define DHTPIN 5
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// PZEM-004T
#define RXD2 16
#define TXD2 17
int requestCount = 0;
unsigned long timeOut = millis();
uint8_t bufferModbus[25];

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!Serial2)
    ;

  dht.begin();
  Serial.println("-------------------------");
  Serial.println("ESP32-ThingsboardPowerMeter");
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (millis() - timeOut > 2000)
  {
    Serial2.write(0xF8);
    Serial2.write(0x04);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x0A);
    Serial2.write(0x64);
    Serial2.write(0x64);
    timeOut = millis();
  }
  if (Serial2.available())
  {
    for (int i = 0; i < 25; i++)
    {
      if (!Serial2.available())
      {
        Serial.println("data not match");
        break;
      }
      bufferModbus[i] = Serial2.read();
      delay(20);
    }

    uint32_t voltage = (uint32_t)bufferModbus[3] << 8 | (uint32_t)bufferModbus[4];
    uint32_t current = (uint32_t)bufferModbus[5] << 8 | (uint32_t)bufferModbus[6] | (uint32_t)bufferModbus[7] << 24 | (uint32_t)bufferModbus[8] << 16;
    uint32_t power = (uint32_t)bufferModbus[9] << 8 | (uint32_t)bufferModbus[10] | (uint32_t)bufferModbus[11] << 24 | (uint32_t)bufferModbus[12] << 16;
    uint32_t energy = (uint32_t)bufferModbus[13] << 8 | (uint32_t)bufferModbus[14] | (uint32_t)bufferModbus[15] << 24 | (uint32_t)bufferModbus[16] << 16;
    uint32_t frequecy = (uint32_t)bufferModbus[17] << 8 | (uint32_t)bufferModbus[18];

    float fVoltage = voltage * 0.1;
    float fCurrent = current * 0.001;
    float fPower = power * 0.1;
    float fEnergy = energy * 0.001;
    float fFrequency = frequecy * 0.1;

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    Serial.println("------------------------------");
    Serial.println("Voltage = " + String(fVoltage));
    Serial.println("Current = " + String(fCurrent));
    Serial.println("Power   = " + String(fPower));
    Serial.println("Energy  = " + String(fEnergy));
    Serial.println("Freq    = " + String(fFrequency));
    Serial.println("Humid   = " + String(h));
    Serial.println("Temp    = " + String(t));

    while (Serial2.available())
    {
      Serial2.read();
    }
  }
}
