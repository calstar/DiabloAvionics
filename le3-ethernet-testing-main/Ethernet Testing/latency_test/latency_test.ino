#include <WiFi.h>
#include <ETH.h>
#include <SPI.h>
#include <EthernetUdp.h>

// --- Define W5500 SPI Pins ---
#define ETH_CLK_PIN   12  // SCK (HSPI)
#define ETH_MISO_PIN  13  // MISO (HSPI)
#define ETH_MOSI_PIN  11  // MOSI (HSPI)
#define ETH_CS_PIN    9   // Your chosen Chip Select pin
#define ETH_PHY_ADDR  0

// --- Network Configuration ---
IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);

const int udpListenPort = 5005;     // Port the ESP32 will listen on (for potential commands)
IPAddress receiverIP(192,168,2,1); // IP address of your computer
const int receiverPort = 5006;       // Port your computer will listen on for data
EthernetUDP udp;

IPAddress clientIP;
uint16_t clientPort;
int client;

void setup() {

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  IPAddress ip(192, 168, 2, 100);

  Serial.begin(115200);
  Serial.println("ESP32 W5500 Basic Test");

  // --- Initialize SPI with custom pins ---
  SPI.begin(ETH_CLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Ethernet.init(ETH_CS_PIN);

  Serial.println("SPI initialized.");

  // --- Initialize Ethernet with W5500 PHY, PHY address, CS pin, and SPI host ---
  // ETH.begin(ETH_PHY_W5500, ETH_PHY_ADDR, ETH_CS_PIN, SPI3_HOST, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CLK_PIN);
  // Ethernet.config()
  // Ethernet
  Ethernet.begin(mac, ip);
  Ethernet.setGatewayIP(gateway);
  Ethernet.setSubnetMask(subnet);

  Serial.println("Ethernet initialized.");

  Serial.print("Waiting a moment for link status...");
  delay(5000); // Wait for a few seconds

  Serial.print("Ethernet Link Status: ");
  Serial.println(Ethernet.localIP());

  udp.begin(udpListenPort);
  Serial.printf("UDP listening on port %d\n", udpListenPort);
  Serial.printf("UDP streaming to %s:%d\n", receiverIP.toString().c_str(), receiverPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.printf("Received %d bytes from %s:%d\n", packetSize, udp.remoteIP().toString().c_str(), udp.remotePort());
    clientIP = udp.remoteIP();
    clientPort = udp.remotePort();

    char incomingPacket[packetSize + 1];
    int len = udp.read(incomingPacket, packetSize);
    if (len > 0) {
      incomingPacket[len] = 0;
      unsigned long responseTimestamp = millis();
      String response = String(incomingPacket) + ", ResponseTimestamp:" + String(responseTimestamp);
      int responseLength = response.length();
      char responseBuffer[responseLength + 1];
      response.toCharArray(responseBuffer, responseLength + 1);

      udp.beginPacket(clientIP, clientPort);
      udp.write(responseBuffer, responseLength);
      udp.endPacket();
      Serial.printf("Sent response: %s\n", responseBuffer);
    }
  }
  delay(1);
}