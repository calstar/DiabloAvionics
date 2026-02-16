/**
 * ESP32 S3: write a single 8-bit integer to SPIFFS (root).
 * Standalone .cpp — no PlatformIO project; add to your build as needed.
 */

#include <Arduino.h>
#include <SPIFFS.h>

static const char* FILE_PATH = "/value.bin";
static const uint8_t VALUE_TO_WRITE = 42;  // change as needed

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return;
  }

  File f = SPIFFS.open(FILE_PATH, "w");
  if (!f) {
    Serial.println("Failed to open file for writing");
    SPIFFS.end();
    return;
  }

  size_t written = f.write(&VALUE_TO_WRITE, 1);
  f.close();
  SPIFFS.end();

  if (written == 1) {
    Serial.printf("Wrote byte %u to %s\n", (unsigned)VALUE_TO_WRITE, FILE_PATH);
  } else {
    Serial.println("Write failed");
  }
}

void loop() {
  // one-shot: nothing to do
}
