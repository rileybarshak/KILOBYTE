#include <Wire.h>

// -------------------- Pin Definitions --------------------
#define PWM   9
#define PWM2  10
#define HALL_SENSOR A0

// -------------------- I2C Variables --------------------
byte receivedThrottle = 180; 
byte receivedLeft = 1;   // default "not pressed"
byte receivedRight = 1;  // default "not pressed"

unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT = 200;  // ms until FAILSAFE activates

// -------------------- Throttle --------------------
float leftThrottle  = 180;      // left motor
float rightThrottle = 180;      // right motor
const float accelStep = 4.0;     // how much each button press adds
const float decayStep = 1.5;     // how fast it returns to neutral
const float minThrottle = 180;

// -------------------- Hall Effect Sensor --------------------
const int hallEffectLow = 500;     // baseline should stay below this
const int hallEffectHigh = 650;    // magnet pass should be above this
const float wheelDiameter = 24.13;   // wheel diameter in centimeters (9.5inch = 24.13cm)
const uint8_t magnetsPerRevolution = 1;
const unsigned long rpmTimeout = 2000; // threshold in ms to consider rpm 0

volatile uint16_t rpmToSend = 0;

bool pulseArmed = true;
unsigned long lastPulseMicros = 0;
unsigned long lastPulseMillis = 0;
float currentRpm = 0.0;

// -------------------- I2C Receive ----------------------
void receiveEvent(int numBytes) {
  if (numBytes >= 3) {
    receivedThrottle = Wire.read();
    receivedLeft = Wire.read();
    receivedRight = Wire.read();

    lastPacketTime = millis();   // mark last good packet
  }
}

// -------------------- I2C Request --------------------
void requestEvent() {
  uint16_t rpm = rpmToSend;
  Wire.write((uint8_t)(rpm & 0xFF));
  Wire.write((uint8_t)(rpm >> 8));
}

// -------------------- Accelerate With Button Press --------------------
void updateThrottle() {
  // Left motor
  if (receivedLeft == 0) {             // button pressed
    leftThrottle += accelStep;
    if (leftThrottle > receivedThrottle) leftThrottle = receivedThrottle;
  } else {                             // button not pressed
    if (leftThrottle > minThrottle) {
      leftThrottle -= decayStep;
      if (leftThrottle < minThrottle) leftThrottle = minThrottle;
    }
  }

  // Right motor
  if (receivedRight == 0) {
    rightThrottle += accelStep;
    if (rightThrottle > receivedThrottle) rightThrottle = receivedThrottle;
  } else {
    if (rightThrottle > minThrottle) {
      rightThrottle -= decayStep;
      if (rightThrottle < minThrottle) rightThrottle = minThrottle;
    }
  }
}

// -------------------- Hall Sensor RPM --------------------
void updateRpm() {
  int hallEffectRaw = analogRead(HALL_SENSOR);
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  if (pulseArmed && hallEffectRaw >= hallEffectHigh) {
    if (lastPulseMicros != 0) {
      unsigned long periodMicros = nowMicros - lastPulseMicros;
      if (periodMicros > 0) {
        currentRpm = 60000000.0 /* 1 minute in microseconds */ / (periodMicros * magnetsPerRevolution);
      }
    }

    lastPulseMicros = nowMicros;
    lastPulseMillis = nowMillis;
    pulseArmed = false;
  } else if (!pulseArmed && hallEffectRaw <= hallEffectLow) {
    pulseArmed = true;
  }

  if (lastPulseMillis == 0 || (nowMillis - lastPulseMillis) > rpmTimeout) {
    currentRpm = 0.0;
  }

  if (currentRpm < 0) currentRpm = 0; // prevent negative RPM
  if (currentRpm > 65535) currentRpm = 65535; // prevent overflow

  noInterrupts(); // ensure variable doesn't change in the middle of setting it
  rpmToSend = (uint16_t)(currentRpm + 0.5); // round to nearest integer
  interrupts();
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  Wire.begin(8);  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  pinMode(PWM, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(HALL_SENSOR, INPUT);
}

// -------------------- Main Loop --------------------
void loop() {
  unsigned long now = millis();

  // FAILSAFE: no packet recently?
  if (now - lastPacketTime > PACKET_TIMEOUT) {
    receivedLeft = 1;
    receivedRight = 1;
    leftThrottle = 180;
    rightThrottle = 180;
    Serial.println("Communication error");
  }

  // Update throttle based on buttons
  updateThrottle();
  updateRpm();

  // Apply throttle to both motors unless one button disables it
  analogWrite(PWM,  (receivedRight == 0 ? receivedThrottle : 180));
  analogWrite(PWM2, (receivedLeft  == 0 ? receivedThrottle : 180));

  // Speed (km/h) derived from wheel diameter and current RPM, kept here for tuning.
  float wheelCircumference = (wheelDiameter / 100.0) * 3.14159265; // in meters
  float speed = (currentRpm * wheelCircumference * 60.0) / 1000.0; // in km/h

// Debugging output

//   Serial.print("Left: ");
//   Serial.print(receivedThrottle);
//   Serial.print(receivedLeft);
//   Serial.print("  Right: ");
//   Serial.println(receivedThrottle);
//   Serial.print(receivedRight);

//   Serial.print("RPM: ");
//   Serial.print((uint16_t)currentRpm);
//   Serial.print("  Speed(km/h): ");
//   Serial.println(speed, 2);

  delay(10);
}

