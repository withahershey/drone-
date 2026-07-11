#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <RF24.h>

// ---- I2C (MPU6050) ----
#define I2C_SDA_PIN 17
#define I2C_SCL_PIN 16

// ---- Radio (nRF24L01+PA+LNA) ----
#define RADIO_CE_PIN    27
#define RADIO_CSN_PIN   4
#define RADIO_MOSI_PIN  25
#define RADIO_MISO_PIN  33
#define RADIO_SCK_PIN   26

// ---- Throttle potentiometer (NOT in original BOM -- add a cheap 10k pot) ----
#define THROTTLE_PIN 34   // analog input, ESP32 ADC1 channel

// ---- Arm/disarm push button (NOT in original BOM -- add a cheap button) ----
#define ARM_BUTTON_PIN 32

MPU6050 mpu(Wire);
SPIClass radioSPI(VSPI);
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

// This address must be IDENTICAL on both the hand controller and the drone.
const byte address[6] = "DRONE";

struct ControlPacket {
  float roll;       // hand tilt left/right, degrees, target attitude
  float pitch;      // hand tilt forward/back, degrees, target attitude
  float yawRate;     // wrist twist rate, deg/sec, target yaw rate
  uint8_t throttle; // 0-255
  bool armed;       // true = motors allowed to spin on the drone
};

ControlPacket packet;

bool armed = false;
bool lastButtonState = HIGH;
unsigned long lastDebounce = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);

  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while (status != 0) {
    // Halt here if the IMU never connects -- check wiring before continuing.
  }

  Serial.println("Keep the controller flat and still for calibration...");
  delay(1000);
  mpu.calcOffsets(); // calibrates gyro + accel offsets
  Serial.println("Calibration done.");

  radioSPI.begin(RADIO_SCK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CSN_PIN);
  if (!radio.begin(&radioSPI)) {
    Serial.println("Radio hardware not responding! Check wiring/power.");
    while (1) {}
  }
  radio.setPALevel(RF24_PA_LOW); 
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76); 
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("Hand controller ready. Starts DISARMED.");
}

void loop() {
  mpu.update();

  // Hand tilt becomes the target roll/pitch angle, clamped to a safe range.
  packet.roll  = constrain(mpu.getAngleX(), -45, 45);
  packet.pitch = constrain(mpu.getAngleY(), -45, 45);
  packet.yawRate = constrain(mpu.getGyroZ(), -100, 100);

  // Throttle: map potentiometer 0-4095 (ESP32 ADC range) to 0-255.
  int raw = analogRead(THROTTLE_PIN);
  packet.throttle = map(raw, 0, 4095, 0, 255);

  // Arm/disarm toggles on each button press (debounced).
  bool buttonState = digitalRead(ARM_BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH && millis() - lastDebounce > 250) {
    armed = !armed;
    lastDebounce = millis();
    Serial.println(armed ? "ARMED" : "DISARMED");
  }
  lastButtonState = buttonState;
  packet.armed = armed;

  radio.write(&packet, sizeof(packet));

  delay(20);
}
