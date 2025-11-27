#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <math.h>

MPU6050 mpu;
Servo motor;

const int servoPin = 9;

// --- your original thresholds & sampling ---
const float tremorThreshold = 0.05;   // SAME threshold you gave
const int sampleWindow = 100;         // SAME
const int sampleDelay  = 10;          // SAME (≈100 Hz sampling)

// --- variance buffer ---
float accelBuffer[sampleWindow];
int indexBuf = 0;

// --- Servo Vibration Control ---
// Mirror movement in opposite direction
const int REST_ANGLE = 90;  // neutral center
const int ANGLE_A    = 65;  // backward small (90 - 25)
const int ANGLE_B    = 35;  // backward larger (90 - 55)

const unsigned long VIBE_INTERVAL_MS = 60; // frequency of vibration
unsigned long lastToggle = 0;
bool toggleState = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  motor.attach(servoPin);
  motor.write(REST_ANGLE); // start in neutral position

  Serial.println("Tremor Detection + Servo Opposite Direction Ready");
}

void loop() {
  // --- read MPU6050 accel ---
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float gx = ax / 16384.0;
  float gy = ay / 16384.0;
  float gz = az / 16384.0;

  float totalAccel = sqrt(gx*gx + gy*gy + gz*gz) - 1.0;

  // --- circular buffer ---
  accelBuffer[indexBuf] = totalAccel;
  indexBuf = (indexBuf + 1) % sampleWindow;

  // --- variance calculation ---
  float mean = 0, var = 0;
  for (int i = 0; i < sampleWindow; i++) mean += accelBuffer[i];
  mean /= sampleWindow;

  for (int i = 0; i < sampleWindow; i++) var += pow(accelBuffer[i] - mean, 2);
  var /= sampleWindow;

  bool tremor = (var > tremorThreshold && var < 0.8); // SAME logic

  // --- servo logic: vibrate only while tremor exists ---
  if (tremor) {
    unsigned long now = millis();
    if (now - lastToggle >= VIBE_INTERVAL_MS) {
      toggleState = !toggleState;
      motor.write(toggleState ? ANGLE_A : ANGLE_B);
      lastToggle = now;
    }
  } else {
    // stop immediately & reset position
    motor.write(REST_ANGLE);
    toggleState = false;
  }

  delay(sampleDelay);
}


