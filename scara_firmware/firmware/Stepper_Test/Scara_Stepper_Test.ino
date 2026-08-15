#include <AccelStepper.h>

// CNC Shield V3 - X Axis
#define STEP_PIN    2
#define DIR_PIN     5
#define ENABLE_PIN  8

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

long microstepCount = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(500);
}

void loop() {

  if (stepper.runSpeed()) {
    microstepCount++;

    // Print every 100 microsteps
    if (microstepCount % 100 == 0) {

      Serial.print("Microsteps = ");
      Serial.print(microstepCount);

      Serial.print(" | Full Steps = ");
      Serial.print(microstepCount / 4);

      Serial.print(" | Revolutions = ");
      Serial.println(microstepCount / 800.0, 3);
    }
  }
}