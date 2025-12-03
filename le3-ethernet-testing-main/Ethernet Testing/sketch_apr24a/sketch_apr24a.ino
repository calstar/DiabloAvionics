#include <WiFi.h> // Required for ESP32 even when using Ethernet
#include <Ethernet.h>
#include <SPI.h>
#include <EthernetUDP.h>

// --- Define W5500 SPI Pins ---
#define ETH_CLK_PIN   7  // SCK (HSPI)
#define ETH_MISO_PIN  41  // MISO (HSPI)
#define ETH_MOSI_PIN  13  // MOSI (HSPI)
#define ETH_CS_PIN    6   // Your chosen Chip Select pin
#define ETH_PHY_ADDR  0   // Often 0, check your module's documentation

IPAddress staticIP(192, 168, 2, 100); // Desired IP address for ESP32
IPAddress gateway(192, 168, 2, 1);   // IP address of your computer
IPAddress subnet(255, 255, 255, 0);  // Same subnet mask as your computer
IPAddress dns(192, 168, 2, 1);  

const int udpListenPort = 5005;     // Port the ESP32 will listen on (for potential commands)
IPAddress receiverIP(192,168,2,1); // IP address of your computer
const int receiverPort = 5006;       // Port your computer will listen on for data
EthernetUDP udp;

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
  String dataToSend = "SensorValue: " + String(random(0, 100)) + ", Timestamp: " + millis() + "\n";
  int dataLength = dataToSend.length();
  char dataBuffer[dataLength + 1]; // +1 for null terminator
  dataToSend.toCharArray(dataBuffer, dataLength + 1);

  // --- Send the UDP packet ---
  udp.beginPacket(receiverIP, receiverPort);
  udp.write(dataBuffer, dataLength); // Send the char array and its length
  udp.endPacket();
  Serial.print("Sent (UDP): ");
  Serial.print(dataToSend);
  // Serial.print(Ethernet.linkStatus());
  delay(50); // Adjust streaming rate (milliseconds)
}