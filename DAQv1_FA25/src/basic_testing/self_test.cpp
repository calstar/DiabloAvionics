#include "board_pins.h"
#include "noise_analyzer.h"
#include "my_ADS126X.h"
#include <Arduino.h>
#include <SPI.h>

static ADS126X ads126x;
NoiseAnalyzer noiseAnalyzer;

void setup() {
  Serial.begin(115200);

  SPI.begin(SCLK, MISOp, MOSIp, CS);
  SPI.setFrequency(1'000'000);
  SPI.setDataMode(SPI_MODE1);
  pinMode(DRDY_PIN, INPUT);


  ads126x.begin(CS);
  ads126x.stopADC1();

  // ads126x.setOutputTDACP(1);
  // ads126x.setOutputTDACN(1);
  ads126x.setOutputmagnitudeTDACP(0b00000111);
  ads126x.setOutputmagnitudeTDACN(0b00000000);
  ads126x.setInputMux(ADS126X_TDAC, ADS126X_TDAC);
  ads126x.bypassPGA();
  ads126x.setFilter(ADS126X_SINC4);
  ads126x.setRate(ADS126X_RATE_38400);
  ads126x.startADC1();
}

void loop() {
  while(digitalRead(DRDY_PIN) != LOW) {
    delayMicroseconds(10);
  }
  
  delayMicroseconds(25);

  const auto reading = ads126x.readADC1();
  Serial.print(F("TDAC reading: "));
  Serial.println(reading.value);
  noiseAnalyzer.addSample(reading.value);

  if (!reading.checksumValid) {
    Serial.println("Bad checksum!");
  }

  // if (noiseAnalyzer.ready()) {
  //       float rmsLsb = noiseAnalyzer.rmsNoiseLsb();
  //       float nfb    = noiseAnalyzer.noiseFreeBits(32);

  //       Serial.print("RMS noise: ");
  //       Serial.print(rmsLsb, 3);
  //       Serial.println(" LSB");

  //       Serial.print("Noise-free bits: ");
  //       Serial.println(nfb, 2);

  //       delay(500);  // don't spam too hard
  //   }
}
