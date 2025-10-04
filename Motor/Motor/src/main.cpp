// Install: ESP32Encoder library by madhephaestus

#include <ESP32Encoder.h>
#include <Arduino.h>

ESP32Encoder enc;

// L298N pins
const int ENA = 25;   // PWM
const int IN1 = 32;   // DIR
const int IN2 = 23;   // DIR

// Encoder pins
const int ENCA = 26;
const int ENCB = 27;

// Control gain
float Kp = 0.003f;     // Increase if too sluggish, decrease if it oscillates

// Optional: set your gearbox to convert counts <-> degrees later if you want
const float GEAR_RATIO = 30.0f;   // example only
const int   CPR_4X_MOTOR = 64;    // hall quad at 4x per motor rev
const float CPR_OUT = CPR_4X_MOTOR * GEAR_RATIO;

void motorDrive(float u) {
  // u in [-1, 1]
  u = constrain(u, -1.0f, 1.0f);
  int pwm = (int)(fabs(u) * 255.0f);

  if (u >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  ledcWrite(0, pwm);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Encoder
  // ESP32Encoder::useInternalWeakPullResistors = ESP32Encoder::Pullup::UP;
  enc.attachFullQuad(ENCA, ENCB);
  enc.clearCount();

  // L298N control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(0, 8000, 8);    // channel 0, 8 kHz, 8-bit (good for L298N)
  ledcAttachPin(ENA, 0);
  motorDrive(0);

  Serial.println("Ready. Toggling between 0 and +1000 counts.");
  Serial.println("If direction is wrong, swap encoder A/B or swap motor leads.");
}

long target = 1000;      // counts
uint32_t toggle_t0 = 0;
uint32_t print_t0  = 0;

void loop() {
  long pos = enc.getCount();
  float err = (float)(target - pos);
  float u = Kp * err;          // simple P control
  motorDrive(u);

  // Telemetry
  if (millis() - print_t0 > 200) {
    print_t0 = millis();
    // Optional degrees readout if GEAR_RATIO is set correctly
    float deg = CPR_OUT > 0 ? (pos * 360.0f / CPR_OUT) : 0.0f;
    Serial.printf("pos=%ld  target=%ld  u=%.3f  deg=%.1f\n", pos, target, u, deg);
  }

  // Toggle target every 3 s
  if (millis() - toggle_t0 > 3000) {
    toggle_t0 = millis();
    target = (target == 0) ? 1000 : 0;
  }

  delay(5);
}
