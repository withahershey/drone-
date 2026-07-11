#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>

// ---- I2C (MPU6050) ----
// Uses default ESP32 I2C pins: SDA = GPIO21, SCL = GPIO22

// ---- Radio ----
#define RADIO_CE_PIN   5
#define RADIO_CSN_PIN  25

// ---- ESC signal pins (wire to the 4-in-1 ESC's M1-M4 signal pads) ----
#define M1_PIN 15
#define M2_PIN 14
#define M3_PIN 13
#define M4_PIN 4

MPU6050 mpu(Wire);
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
Servo escM1, escM2, escM3, escM4;

// This address must be IDENTICAL to the hand controller's.
const byte address[6] = "DRONE";

struct ControlPacket {
  float roll;
  float pitch;
  float yawRate;
  uint8_t throttle;
  bool armed;
};

ControlPacket packet = {0, 0, 0, 0, false};
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 300; // ms with no packet before we cut motors

// ---- PID state ----
float rollI = 0, pitchI = 0, yawI = 0;
float rollPrevErr = 0, pitchPrevErr = 0, yawPrevErr = 0;
unsigned long lastPidTime = 0;

// ---- STARTER PID GAINS ----
float rollKp = 2.0,  rollKi = 0.0,  rollKd = 0.5;
float pitchKp = 2.0, pitchKi = 0.0, pitchKd = 0.5;
float yawKp = 1.5,   yawKi = 0.0,   yawKd = 0.0;

const int ESC_MIN = 1000;
const int ESC_MAX = 2000;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while (status != 0) {
    // Halt here if the IMU never connects
  }

  Serial.println("Keep the drone flat and still for calibration...");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Calibration done.");

  if (!radio.begin()) {
    Serial.println("Radio hardware not responding! Check wiring/power.");
    while (1) {}
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76); // must match the hand controller's channel exactly
  radio.openReadingPipe(0, address);
  radio.startListening();

  escM1.setPeriodHertz(50);
  escM2.setPeriodHertz(50);
  escM3.setPeriodHertz(50);
  escM4.setPeriodHertz(50);
  escM1.attach(M1_PIN, ESC_MIN, ESC_MAX);
  escM2.attach(M2_PIN, ESC_MIN, ESC_MAX);
  escM3.attach(M3_PIN, ESC_MIN, ESC_MAX);
  escM4.attach(M4_PIN, ESC_MIN, ESC_MAX);

  // Send minimum signal so the ESCs arm/initialize correctly.
  escM1.writeMicroseconds(ESC_MIN);
  escM2.writeMicroseconds(ESC_MIN);
  escM3.writeMicroseconds(ESC_MIN);
  escM4.writeMicroseconds(ESC_MIN);
  delay(3000); // give the ESCs time to power up and beep-arm

  lastPidTime = millis();
  Serial.println("Drone flight controller ready. Waiting for armed signal...");
}

void loop() {
  mpu.update();

  // ---- Receive the latest command packet, if one arrived ----
  if (radio.available()) {
    radio.read(&packet, sizeof(packet));
    lastPacketTime = millis();
  }

  // ---- Failsafe: cut motors if we haven't heard from the controller recently ----
  bool signalLost = (millis() - lastPacketTime) > FAILSAFE_TIMEOUT;
  bool motorsAllowed = packet.armed && !signalLost;

  // ---- Read the drone's own current attitude ----
  float currentRoll  = mpu.getAngleX();
  float currentPitch = mpu.getAngleY();
  float currentYawRate = mpu.getGyroZ();

  // ---- PID timing ----
  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastPidTime = now;

  // ---- Roll PID: target = packet.roll (from hand), measured = currentRoll ----
  float rollErr = packet.roll - currentRoll;
  rollI += rollErr * dt;
  rollI = constrain(rollI, -50, 50); // anti-windup clamp
  float rollD = (rollErr - rollPrevErr) / dt;
  rollPrevErr = rollErr;
  float rollOut = rollKp * rollErr + rollKi * rollI + rollKd * rollD;

  // ---- Pitch PID ----
  float pitchErr = packet.pitch - currentPitch;
  pitchI += pitchErr * dt;
  pitchI = constrain(pitchI, -50, 50);
  float pitchD = (pitchErr - pitchPrevErr) / dt;
  pitchPrevErr = pitchErr;
  float pitchOut = pitchKp * pitchErr + pitchKi * pitchI + pitchKd * pitchD;

  // ---- Yaw rate PID: target = packet.yawRate, measured = currentYawRate ----
  float yawErr = packet.yawRate - currentYawRate;
  yawI += yawErr * dt;
  yawI = constrain(yawI, -50, 50);
  float yawD = (yawErr - yawPrevErr) / dt;
  yawPrevErr = yawErr;
  float yawOut = yawKp * yawErr + yawKi * yawI + yawKd * yawD;

  // ---- Convert 0-255 throttle to a base ESC pulse width ----
  int baseThrottle = map(packet.throttle, 0, 255, ESC_MIN, ESC_MAX);

  // ---- Standard X-quad motor mixing ----
  // If the drone tips the WRONG way when you tilt it by hand during bench
  // testing, flip the sign on that axis here rather than rewiring anything.
  int m1 = baseThrottle + pitchOut + rollOut - yawOut; // front-left  CCW
  int m2 = baseThrottle + pitchOut - rollOut + yawOut; // front-right CW
  int m3 = baseThrottle - pitchOut + rollOut + yawOut; // rear-left   CW
  int m4 = baseThrottle - pitchOut - rollOut - yawOut; // rear-right  CCW

  m1 = constrain(m1, ESC_MIN, ESC_MAX);
  m2 = constrain(m2, ESC_MIN, ESC_MAX);
  m3 = constrain(m3, ESC_MIN, ESC_MAX);
  m4 = constrain(m4, ESC_MIN, ESC_MAX);

  if (motorsAllowed) {
    escM1.writeMicroseconds(m1);
    escM2.writeMicroseconds(m2);
    escM3.writeMicroseconds(m3);
    escM4.writeMicroseconds(m4);
  } else {
    escM1.writeMicroseconds(ESC_MIN);
    escM2.writeMicroseconds(ESC_MIN);
    escM3.writeMicroseconds(ESC_MIN);
    escM4.writeMicroseconds(ESC_MIN);
    // Reset integrators so they don't wind up while sitting disarmed.
    rollI = 0; pitchI = 0; yawI = 0;
  }

  delay(4); // ~250Hz control loop -- adjust based on real-world performance
}
