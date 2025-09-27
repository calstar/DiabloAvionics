// New (Module 8)
#define DRDY_PIN 14
#define DRDY 14
#define MOSI 5
#define MISO 41
#define SCLK 13
#define CS 37
#define START 43

// ADC COM Pin
int neg_pin = 0xA;   // AINCOM

// ADC reference voltage
float vRef = 2.5f;

// ADC Scale
float adcScale = 2147483647.0f; // = 2^(32-1)-1

// Channels
uint8_t channels[] = {1,2,3,4};
size_t num_channels = sizeof(channels) / sizeof(channels[0]);  




// Old (Module 8)
#define DOUT 41 // MISO
#define DIN 5   // MOSI
#define SCLK 13 
#define CS 37
#define START 43