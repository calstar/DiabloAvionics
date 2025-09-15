#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define ETH_CLK_PIN   7
#define ETH_MISO_PIN  41
#define ETH_MOSI_PIN  13
#define ETH_CS_PIN    6

IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);
IPAddress receiverIP(192, 168, 2, 1);
const int receiverPort = 5006;
EthernetUDP udp;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

void setup() {
  Serial.begin(115200);
  Serial.println("Minimal UDP Sender");

  // Start SPI with custom pins
  SPI.begin(ETH_CLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  delay(1000);

  // Initialize Ethernet with CS pin
  Ethernet.init(ETH_CS_PIN);
  delay(1000);

  // Start Ethernet with static IP
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);
  delay(1000);

  Serial.println("ESP32 IP: ");
  Serial.println(Ethernet.localIP());

  // Start UDP
  udp.begin(5005);
}

void loop() {
  String sensorValue = "Example data: " + String(random(0, 100));
  unsigned long timestamp = millis();
  String dataToSend = sensorValue + ", Timestamp: " + String(timestamp) + "\n";

  // Convert to C-string
  int dataLength = dataToSend.length();
  char dataBuffer[dataLength + 1];
  dataToSend.toCharArray(dataBuffer, dataLength + 1);

  // Send UDP packet
  udp.beginPacket(receiverIP, receiverPort);
  udp.write(dataBuffer, dataLength);
  udp.endPacket();

  Serial.print("Sent (UDP): ");
  Serial.print(dataToSend);

  delay(1000);
}
