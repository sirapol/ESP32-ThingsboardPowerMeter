//  ______ _       _          ______ _           _                   _
// |  ____| |     | |        |  ____| |         | |                 (_)
// | |__  | |_   _| | _____  | |__  | | ___  ___| |_ _ __ ___  _ __  _  ___ ___
// |  __| | | | | | |/ / _ \ |  __| | |/ _ \/ __| __| '__/ _ \| '_ \| |/ __/ __|
// | |    | | |_| |   <  __/ | |____| |  __/ (__| |_| | | (_) | | | | | (__\__ \.
// |_|    |_|\__,_|_|\_\___| |______|_|\___|\___|\__|_|  \___/|_| |_|_|\___|___/

// OTA and WiFi
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiClient espClient;
int status = WL_IDLE_STATUS;
bool subscribed = false;
const char *ssid = "injectorhome_2.4GHz";
const char *password = "sirapols0422";

// Thingsboard
#include <ThingsBoard.h>
#define THINGSBOARD_SERVER "192.168.1.25"
#define TOKEN "vA60qMrhtEzIX2LoddSN"
#define tbUpdateRate 33333
unsigned long tbUpdateMillis = millis();

ThingsBoard tb(espClient);

// DHT
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 5
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
float Temp, Humid;
unsigned long timeOut_DHT = millis();

// PZEM-004T
#define RXD2 16
#define TXD2 17
int requestCount = 0;
unsigned long timeOut_PZEM = millis();
uint8_t bufferModbus[25];
float fVoltage;
float fCurrent;
float fPower;
float fEnergy;
float fFrequency;

void initDHT()
{
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void getDHT()
{
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    // Serial.println(F("Error reading temperature!"));
    Temp = -1.0;
  }
  else
  {
    // Serial.print(F("Temperature: "));
    // Serial.print(event.temperature);
    // Serial.println(F("°C"));
    Temp = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    // Serial.println(F("Error reading humidity!"));
    Humid = -1.0;
  }
  else
  {
    // Serial.print(F("Humidity: "));
    // Serial.print(event.relative_humidity);
    // Serial.println(F("%"));
    Humid = event.relative_humidity;
  }
}

void initWiFi()
{
  Serial.print("Connecting to WiFi");
  Serial.println(ssid);
  // attempt to connect to WiFi network

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void initOTA()
{
  Serial.println("Booting");
  ArduinoOTA.setHostname("ESP32-Home-PowerMeter");
  ArduinoOTA
      .onStart([]()
               {
                 String type;
                 if (ArduinoOTA.getCommand() == U_FLASH)
                   type = "sketch";
                 else // U_SPIFFS
                   type = "filesystem";

                 // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                 Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
                 Serial.printf("Error[%u]: ", error);
                 if (error == OTA_AUTH_ERROR)
                   Serial.println("Auth Failed");
                 else if (error == OTA_BEGIN_ERROR)
                   Serial.println("Begin Failed");
                 else if (error == OTA_CONNECT_ERROR)
                   Serial.println("Connect Failed");
                 else if (error == OTA_RECEIVE_ERROR)
                   Serial.println("Receive Failed");
                 else if (error == OTA_END_ERROR)
                   Serial.println("End Failed"); });
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  // Loop until we're reconnected
  Serial.println("Reconnecting WiFi");
  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!Serial2)
    ;

  Serial.println("-------------------------");
  Serial.println("ESP32-ThingsboardPowerMeter");
  initWiFi();
  initDHT();
  initOTA();
}

void loop()
{
  ArduinoOTA.handle(); // OTA Handle

  if (WiFi.status() != WL_CONNECTED)
  {
    reconnect();
  }

  if (millis() - tbUpdateMillis > tbUpdateRate)
  {
    tbUpdateMillis = millis();
    // Reconnect to ThingsBoard, if needed
    if (!tb.connected())
    {
      subscribed = false;

      // Connect to the ThingsBoard
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with token ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, 1883))
      {
        Serial.println("Failed to connect");
        return;
      }
      Serial.println("Connected");
      subscribed = true;
    }
    else
    {
      tb.sendTelemetryFloat("temperature", Temp);
      tb.sendTelemetryFloat("humidity", Humid);
      tb.sendTelemetryFloat("voltage", fVoltage);
      tb.sendTelemetryFloat("current", fCurrent);
      tb.sendTelemetryFloat("power", fPower);
      tb.sendTelemetryFloat("energy", fEnergy);
      tb.sendTelemetryFloat("frequncy", fFrequency);
      Serial.println("Send completed");
    }
  }

  if (millis() - timeOut_PZEM > 2222)
  {
    Serial2.write(0xF8);
    Serial2.write(0x04);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x0A);
    Serial2.write(0x64);
    Serial2.write(0x64);
    timeOut_PZEM = millis();
  }
  if (millis() - timeOut_DHT > 3333)
  {
    timeOut_DHT = millis();
    getDHT();
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

    fVoltage = voltage * 0.1;
    fCurrent = current * 0.001;
    fPower = power * 0.1;
    fEnergy = energy * 0.001;
    fFrequency = frequecy * 0.1;

    Serial.println("------------------------------");
    Serial.println("Voltage = " + String(fVoltage));
    Serial.println("Current = " + String(fCurrent));
    Serial.println("Power   = " + String(fPower));
    Serial.println("Energy  = " + String(fEnergy));
    Serial.println("Freq    = " + String(fFrequency));
    Serial.println("Humid   = " + String(Humid));
    Serial.println("Temp    = " + String(Temp));

    while (Serial2.available())
    {
      Serial2.read();
    }
  }
}
