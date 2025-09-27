#include <MCP23S17.h>
#include <ADS126X.h>
#include <SPI.h>
#include <state_machine.h>

#define SENSE_CS_1 40
#define SENSE_DRDY_1 4 
#define SENSE_CS_2 42
#define SENSE_DRDY_2 35
#define PYRO_CS_1 16
#define PYRO_CS_2 4
#define MOSI 5
#define MISO 41
#define CLK 13

#define VALVE_FUP 36 // Fuel upstream solenoid open
#define VALVE_FDP 35 // Fuel downstream solenoid open
#define VALVE_OUP 34 // LOX upstream solenoid open
#define VALVE_ODP 33 // LOX downstream solenoid open
#define FUELVENT 37 //SOL FVP
#define LOXVENT 38 //SOL OVP
#define PRESSURELINE 32

#define FUELMAIN 39 //actuators
#define LOXMAIN 40 //actuators

#define UP_PRESSURE 10

// Pressure transducers
// Fuel path
#define PT_I 13  // INJECTOR
#define PT_P 14 //UPSTREAM PRESSURE FOR BOTH FUEL AND OXIDIZER

#define PT_O1 9   // OX TANK PRESSURE 
#define PT_F1 6 // FUEL TANK PRESSURE

// LOX path - new pins need to be defined
#define PT_O2 10   // LOX DOWNSTREAM pressure
#define PT_F2 12 // FUEL DOWNSTREAM pressure

float calculatePressure(float raw_value, float PT_A, float PT_B, float PT_C, float PT_D) {
    return (PT_A * pow(raw_value, 3)) +
           (PT_B * pow(raw_value, 2)) +
           (PT_C * raw_value) + PT_D;
}

float readPT(int channel) {
  delay(10);
  SENSE_1.readADC1(channel, ADS126X_AINCOM);
  delay(10);
  long raw = SENSE_1.readADC1(channel, ADS126X_AINCOM);
  float voltage = (float)raw * 5.0 / 2147483648.0;
  return voltage;
}

void mosfetCloseAllValves() {
    for (int i = 0; i < 9; i++) {
      PYRO_1_MCP->write1(i, 0);
    }
}