#include <SPI.h>        

#include <Ethernet.h>

#include <EthernetUdp.h>



// Enter a MAC address and IP address for your controller below.

// The IP address will be dependent on your local network:

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress ip(192, 168, 1, 177);



unsigned int localPort = 8888;      // local port to listen on



// An EthernetUDP instance to let us send and receive packets over UDP

EthernetUDP Udp;



void setup() {

  // start the Ethernet and UDP:

  Ethernet.begin(mac,ip);

  Udp.begin(localPort);
  Udp.remotePort()

}



void loop() {

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

    Udp.write("hello");

    Udp.endPacket();

}