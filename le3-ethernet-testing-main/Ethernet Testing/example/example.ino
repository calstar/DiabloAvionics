// THIS IS GOOD ONE
#include <WiFi.h> // Required for ESP32 even when using Ethernet
#include <ETH.h>
#include <SPI.h>
#include <EthernetUdp.h>

// --- Define W5500 SPI Pins ---
#define ETH_CLK_PIN   12  // SCK (HSPI)
#define ETH_MISO_PIN  13  // MISO (HSPI)
#define ETH_MOSI_PIN  11  // MOSI (HSPI)
#define ETH_CS_PIN    9   // Your chosen Chip Select pin
#define ETH_PHY_ADDR  0

// --- Network Configuration for Direct Connection ---
IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);

// --- UDP Configuration ---
const int udpSendPort = 5000; // ESP32 sends from this port
IPAddress receiverIP(192, 168, 2, 1);
const int receiverPort = 5000; // Computer listens on this port
EthernetUDP udp;

// --- Data Buffer ---
const int bufferSize = 1000; // Maximum UDP payload size is 1472.  Use something smaller
byte dataBuffer[bufferSize];

unsigned long startTime;
unsigned long totalBytesSent = 0;
bool sending = true;
unsigned long sendInterval = 10;
unsigned long previousTime = 0;
unsigned long packetCount = 0;
bool waitingForResponse = false;
unsigned long pingStartTime;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 UDP Max Data Rate Test (Sender)");

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  IPAddress ip(192, 168, 2, 100);


  // --- Initialize SPI with custom pins ---
  SPI.begin(ETH_CLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Ethernet.init(ETH_CS_PIN);

  // --- Initialize Ethernet with W5500 PHY and custom CS pin ---
  Ethernet.begin(mac, ip);
  Ethernet.setGatewayIP(gateway);
  Ethernet.setSubnetMask(subnet);
  
  Serial.println("Ethernet initialized.");
  Serial.print("ESP32 IP Address: ");
  Serial.println(Ethernet.localIP());

  Serial.println("Waiting a moment for link status...");
  delay(5000); // Wait for a few seconds

  // --- Initialize UDP ---
  udp.begin(udpSendPort);
  // Serial.printf("Sending UDP data to %s:%d\n", receiverIP.toString().c_str(), receiverPort);

  // --- Fill the data buffer ---
  for (int i = 0; i < bufferSize; i++) {
    dataBuffer[i] = (byte)i; // Fill with a pattern
  }
  startTime = millis();
  previousTime = startTime;
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Receive the packet
    byte receivedData[packetSize];
    udp.read(receivedData, packetSize);

    // Echo back the packet
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(receivedData, packetSize);
    udp.endPacket();
  }
}