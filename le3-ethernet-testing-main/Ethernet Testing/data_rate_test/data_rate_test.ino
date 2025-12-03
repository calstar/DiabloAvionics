#include <WiFi.h> // Required for ESP32 even when using Ethernet
#include <ETH.h>
#include <SPI.h>
#include <EthernetUdp.h>

// --- Define W5500 SPI Pins ---
#define ETH_CLK_PIN   7  // SCK (HSPI)
#define ETH_MISO_PIN  41  // MISO (HSPI)
#define ETH_MOSI_PIN  13  // MOSI (HSPI)
#define ETH_CS_PIN    6   // Your chosen Chip Select pin
#define ETH_PHY_ADDR  0

// --- Network Configuration for Direct Connection ---
IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);

// --- UDP Configuration ---
const int udpSendPort = 5005; // ESP32 sends from this port
IPAddress receiverIP(192, 168, 2, 1);
const int receiverPort = 5006; // Computer listens on this port
EthernetUDP udp;

// --- Data Buffer ---
const int bufferSize = 1472; // Maximum UDP payload size is 1472.  Use something smaller
byte dataBuffer[bufferSize];

unsigned long startTime;
unsigned long totalBytesSent = 0;
bool sending = true;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 UDP Max Data Rate Test (Sender)");

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  IPAddress ip(192, 168, 2, 100);


  // --- Initialize SPI with custom pins ---
  SPI.begin(ETH_CLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Ethernet.init(ETH_CS_PIN);

  // --- Initialize Ethernet with W5500 PHY and custom CS pin ---
  Serial.print("ESP32 IP Address: ");
  Serial.println(Ethernet.localIP());
  Ethernet.begin(mac, ip);
  Ethernet.setGatewayIP(gateway);
  Ethernet.setSubnetMask(subnet);
  
  Serial.println("Ethernet initialized.");

  Serial.print("Waiting a moment for link status...");
  delay(5000); // Wait for a few seconds

  // --- Initialize UDP ---
  udp.begin(udpSendPort);
  Serial.printf("Sending UDP data to %s:%d\n", receiverIP.toString().c_str(), receiverPort);

  // --- Fill the data buffer ---
  for (int i = 0; i < bufferSize; i++) {
    dataBuffer[i] = (byte)i; // Fill with a pattern
  }
  startTime = millis();
}

void loop() {
  if (sending)
  {
      // --- Send the UDP packet ---
      udp.beginPacket(receiverIP, receiverPort);
      udp.write(dataBuffer, bufferSize);
      udp.endPacket();
      totalBytesSent += bufferSize;

      unsigned long now = millis();
      if (now - startTime >= 5000) { // Print every 5 seconds
        float elapsedSeconds = (now - startTime) / 1000.0;
        float dataRate = totalBytesSent / elapsedSeconds; // Bytes per second
        float mbps = (dataRate * 8) / 1000000.0;           // Convert to Mbps
        Serial.printf("Time: %.2f s, Sent: %lu bytes, Data Rate: %.2f Mbps\n", elapsedSeconds, totalBytesSent, mbps);
        startTime = now;         // Reset start time
        totalBytesSent = 0;    // Reset byte counter
      }
  }
}
