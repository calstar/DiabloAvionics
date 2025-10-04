// Install: ESP32Encoder library by madhephaestus

#include <ESP32Encoder.h>
#include <Arduino.h>

ESP32Encoder enc;

// L298N pins (your latest wiring)
const int ENA = 25;   // PWM
const int IN1 = 23;
const int IN2 = 5;

// Encoder pins (your latest wiring)
const int ENCA = 19;
const int ENCB = 21;

// ---- Tuning / mech constants ----
float Kp = 0.0015f;          // start here; raise if too slow, lower if oscillates
const int   CPR_4X_MOTOR = 64;     // encoder counts per MOTOR rev at 4x (set true value)
const float GEAR_RATIO    = 210.0f; // internal gearbox ratio you reported
const float CPR_OUT       = CPR_4X_MOTOR * GEAR_RATIO;

// PWM setup
const int   PWM_CH   = 0;
const int   PWM_FREQ = 8000;
const int   PWM_BITS = 8;
const int   PWM_MAX  = (1<<PWM_BITS)-1;
const int   PWM_MIN  = 45;         // deadband to overcome L298N + gearmotor friction

// Control state
volatile int dir_sign = +1;        // set by autoPolarityCal()
long target = 1000;                // counts
uint32_t toggle_t0 = 0;
uint32_t print_t0  = 0;

void motorCoast() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CH, 0);
}

void motorDrive(float u) {
  // u in [-1, 1] after dir_sign applied
  u = constrain(u, -1.0f, 1.0f);
  int pwm = 0;

  if (fabs(u) < 1e-3f) {
    motorCoast();
    return;
  }

  if (u > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    pwm = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * u);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    pwm = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * (-u));
  }
  ledcWrite(PWM_CH, pwm);
}

// One-shot polarity calibration: apply a small forward nudge and measure Δcounts
void autoPolarityCal() {
  Serial.println("Auto-polarity: nudging forward to detect sign...");
  enc.clearCount();

  // gentle forward nudge
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CH, 60);      // small PWM above deadband
  delay(150);
  motorCoast();

  long dpos = enc.getCount();
  if (dpos < 0) {
    dir_sign = -1;
    Serial.println("Auto-polarity: inverted (dir_sign = -1).");
  } else {
    dir_sign = +1;
    Serial.println("Auto-polarity: OK (dir_sign = +1).");
  }

  // Reset reference after calibration
  enc.clearCount();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Encoder
  enc.attachFullQuad(ENCA, ENCB);
  enc.clearCount();

  // L298N control pins + PWM
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_BITS);
  ledcAttachPin(ENA, PWM_CH);
  motorCoast();

  Serial.println("Ready. Calibrating direction, then toggling 0 <-> +1000 counts.");
  autoPolarityCal();    // determine dir_sign before control starts
}

void loop() {
  long pos = enc.getCount();
  float err = (float)(target - pos);

  float u = dir_sign * Kp * err;
  u = constrain(u, -1.0f, 1.0f);
  motorDrive(u);

  // Telemetry
  if (millis() - print_t0 > 200) {
    print_t0 = millis();
    float deg = CPR_OUT > 0 ? (pos * 360.0f / CPR_OUT) : 0.0f;
    int pwm_now = ledcRead(PWM_CH);
    Serial.printf("pos=%ld  target=%ld  err=%.0f  dir=%d  pwm=%d  deg=%.1f\n",
                  pos, target, err, dir_sign, pwm_now, deg);
  }

  // Toggle target every 3 s
  if (millis() - toggle_t0 > 3000) {
    toggle_t0 = millis();
    target = (target == 0) ? 1000 : 0;
    Serial.printf("New target: %ld counts\n", target);
  }

  delay(5);
}
