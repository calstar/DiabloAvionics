#include "EthernetHandler.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <esp_system.h>

#define ETH_CLK_PIN   7
#define ETH_MISO_PIN  41
#define ETH_MOSI_PIN  13
#define ETH_CS_PIN    6

// Network Settings
IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);
IPAddress receiverIP(192, 168, 2, 1);
const int receiverPort = 5006;

EthernetUDP udp;

void EthernetInit() {
    // Generate MAC
    uint64_t chipid = ESP.getEfuseMac();
    byte mac[6];
    mac[0] = 0x02;
    mac[1] = 0x00;
    mac[2] = (chipid >> 32) & 0xFF;
    mac[3] = (chipid >> 24) & 0xFF;
    mac[4] = (chipid >> 16) & 0xFF;
    mac[5] = (chipid >> 8) & 0xFF;

    // SPI + Ethernet
    SPI.begin(ETH_CLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
    Ethernet.init(ETH_CS_PIN);
    Ethernet.begin(mac, staticIP, dns, gateway, subnet);

    udp.begin(5005);
}

void sendPacket(const String &data) {
    udp.beginPacket(receiverIP, receiverPort);
    udp.write(data.c_str(), data.length());
    udp.endPacket();
}