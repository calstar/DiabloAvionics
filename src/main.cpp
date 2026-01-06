#include <Arduino.h>

// put function declarations here:
void updateStateMachine();
void handleSensor();
void handleNetworking();

// Define the states of the sensor node
enum class SensorState {
    INIT,
    CHECK_SERVER,
    NORMAL_OPERATION,
    SERVER_DOWN,
    FALLBACK_MODE,
    ERROR
};


void setup() {
  Serial.begin(9600);

  //Wait for Server

  //Wait for config



  //Good to go
  Serial.print("Setup complete. Entering main loop.\n");
}

void loop() {
  updateStateMachine();
  handleSensor();
  handleNetworking();
}

void updateStateMachine(){

}
void handleSensor(){

}
void handleNetworking(){

}