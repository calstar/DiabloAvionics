#include <Wire.h>

// Pin config
#define SDA_PIN 4
#define SCL_PIN 5

// AS5600
#define AS5600_ADDR 0x36
#define REG_STATUS 0x0B
#define REG_RAW_MSB 0x0C
#define STATUS_MH 0x08 // magnet too strong
#define STATUS_ML 0x10 // magnet too weak
#define STATUS_MD 0x20 // magnet detected
#define READ_ERROR 0xFFFF

float startAngle = 0.0f;
float degAngle = 0.0f;
float correctedAngle = 0.0f;

bool i2cReadBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return false;
  Wire.requestFrom(AS5600_ADDR, (uint8_t)len);
  if (Wire.available() < len)
    return false;
  for (uint8_t i = 0; i < len; i++)
    buf[i] = Wire.read();
  return true;
}

uint16_t readRawAngle()
{
  uint8_t buf[2];
  if (!i2cReadBytes(REG_RAW_MSB, buf, 2))
    return READ_ERROR;
  return ((uint16_t)(buf[0] << 8) | buf[1]) & 0x0FFF;
}

float rawToDegrees(uint16_t raw)
{
  return raw * 0.087890625f;
}

bool checkMagnetPresence()
{
  Serial.println("[AS5600] Checking magnet...");
  unsigned long start = millis();
  while (millis() - start < 3000)
  {
    uint8_t status;
    if (!i2cReadBytes(REG_STATUS, &status, 1))
    {
      Serial.println("[AS5600] ERROR: could not read STATUS - check wiring");
      delay(500);
      continue;
    }
    status &= 0x38;
    if (status & STATUS_MD)
    {
      if (status & STATUS_MH)
      {
        Serial.println("[AS5600] WARNING: magnet too strong - increase air gap");
        return false;
      }
      if (status & STATUS_ML)
      {
        Serial.println("[AS5600] WARNING: magnet too weak - decrease air gap");
        return false;
      }
      Serial.println("[AS5600] Magnet detected OK");
      return true;
    }
    delay(100);
  }
  Serial.println("[AS5600] ERROR: no magnet detected after 3s");
  return false;
}

void correctAngle()
{
  correctedAngle = degAngle - startAngle;
  if (correctedAngle < 0)
    correctedAngle += 360.0f;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== AS5600 Ball Valve Encoder ===");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Serial.println("I2C initialized");
  checkMagnetPresence();
  uint16_t raw = readRawAngle();
  if (raw != READ_ERROR)
  {
    startAngle = rawToDegrees(raw);
    Serial.print("Start angle tared at: ");
    Serial.print(startAngle, 1);
    Serial.println(" deg");
  }
  else
  {
    Serial.println("WARNING: could not tare start angle");
  }
  Serial.println("=================================");
}

void loop()
{
  uint16_t raw = readRawAngle();
  if (raw == READ_ERROR)
  {
    Serial.println("ERROR: I2C read failed");
  }
  else
  {
    degAngle = rawToDegrees(raw);
    correctAngle();
    Serial.print("Raw: ");
    Serial.print(raw);
    Serial.print("  Absolute: ");
    Serial.print(degAngle, 1);
    Serial.print(" deg  Corrected: ");
    Serial.print(correctedAngle, 1);
    Serial.println(" deg");
  }
  delay(100);
}
