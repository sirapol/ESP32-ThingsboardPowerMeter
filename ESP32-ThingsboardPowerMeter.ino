#define RXD2 16
#define TXD2 17

#define SDA_OLED 5
#define SCL_OLED 4


int requestCount = 0;
unsigned long timeOut = millis();
uint8_t bufferModbus[25];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!Serial2);
  Serial.println("Fin setup");

}

void loop() {
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
    Serial.println("data");
    for (int i = 0; i < 25; i++)
    {
      if (!Serial2.available())
      {
        Serial.println("data not match");
        break;
      }
      bufferModbus[i] = Serial2.read();
    }


    uint32_t voltage = (uint32_t)bufferModbus[3] << 8 | (uint32_t)bufferModbus[4];
    uint32_t current = (uint32_t)bufferModbus[5] << 8 | (uint32_t)bufferModbus[6] | (uint32_t)bufferModbus[7] << 24 | (uint32_t)bufferModbus[8] << 16;
    uint32_t power =   (uint32_t)bufferModbus[9] << 8 | (uint32_t)bufferModbus[10] | (uint32_t)bufferModbus[11] << 24 | (uint32_t)bufferModbus[12] << 16;
    uint32_t energy =  (uint32_t)bufferModbus[13] << 8 | (uint32_t)bufferModbus[14] | (uint32_t)bufferModbus[15] << 24 | (uint32_t)bufferModbus[16] << 16;
    uint32_t frequecy = (uint32_t)bufferModbus[17] << 8 | (uint32_t)bufferModbus[18];

    float fVoltage = voltage * 0.1;
    float fCurrent = current * 0.001;
    float fPower = power * 0.1;
    float fEnergy = energy * 0.001;
    float fFrequency = frequecy * 0.1;
    
    Serial.println(fVoltage);
    Serial.println(fCurrent);
    Serial.println(fPower);
    Serial.println(fEnergy);
    Serial.println(fFrequency);
    
    while (Serial2.available())
    {
      Serial2.read();
    }
  }
}
