#pragma once
#include <Arduino.h>

// Initialize Ethernet
void EthernetInit();

// Send a UDP packet
void sendPacket(const String &data);
