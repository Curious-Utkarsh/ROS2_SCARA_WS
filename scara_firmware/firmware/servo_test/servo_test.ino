/*
   Servo test - move the gripper servo by hand and read the angle back

   Use this to find the two angles you need for the gripper: the one where it is
   fully open, and the one where it is fully closed. Put those numbers into
   GRIPPER_OPEN_ANGLE and GRIPPER_CLOSED_ANGLE in scara_moveit_control.ino.

   Wiring: servo signal wire  -> Z+ on the CNC Shield V3, which is pin 11
           servo red wire     -> 5V
           servo brown/black  -> GND

   Open the Serial Monitor, set it to 115200 baud and "Newline", then type:

       30      go to 30 degrees
       +       open a bit more   (5 degrees)
       -       close a bit more  (5 degrees)
       s       sweep slowly across the whole range and back
       ?       print this help again

   This gripper can only travel between 5 and 55 degrees. Anything outside that
   is refused, because past those angles the servo grinds against a stop.

   ---------------------------------------------------------------------------
   IMPORTANT: A SERVO CANNOT TELL YOU WHERE IT IS

   A normal hobby servo has three wires: power, ground, and one signal wire that
   only carries orders *to* the servo. Nothing comes back. So when you call
   gripperServo.read(), the library does not ask the servo anything - it simply
   hands back the last number you gave to write().

   This sketch prints both numbers side by side so you can see that they are
   always the same. If the servo is jammed, or the arm is pushing against it,
   the printed angle will still look perfect while the real horn is somewhere
   else entirely. Trust your eyes, not the number.

   (If you ever do need real feedback, you have to open the servo and solder a
   wire onto its internal potentiometer, then read that with an analog pin.)
   ---------------------------------------------------------------------------
*/

#include <Servo.h>

#define SERVO_PIN 11        // Z+ endstop header on the CNC Shield V3

// The gripper positions ROS uses, in metres, and what they mean in degrees.
// These match scara_moveit_control.ino - keep the two files the same.
const float GRIPPER_CLOSED_METRES = -0.06;
const float GRIPPER_OPEN_METRES   =  0.00;
const int   GRIPPER_CLOSED_ANGLE  = 5;
const int   GRIPPER_OPEN_ANGLE    = 55;

// Past these angles the servo pushes against a mechanical stop. The sweep and
// the + / - keys stop here so you cannot cook the servo by accident.
const int SAFE_LOWEST_ANGLE  = 5;
const int SAFE_HIGHEST_ANGLE = 55;

const int STEP_SIZE = 5;    // how far + and - move the servo

Servo gripperServo;
int angle = GRIPPER_OPEN_ANGLE;    // the angle we last asked for


// ===========================================================================
// Helpers
// ===========================================================================

void printHelp() {
  Serial.println();
  Serial.println("=== gripper servo test ===");
  Serial.print("type a number between ");
  Serial.print(SAFE_LOWEST_ANGLE);
  Serial.print(" and ");
  Serial.print(SAFE_HIGHEST_ANGLE);
  Serial.println(" to go to that angle");
  Serial.println("type + to open more, - to close more, s to sweep, ? for help");
  Serial.println();
}

// Work out what this angle means to ROS, in metres.
float angleToMetres(int servoAngle) {
  float span = GRIPPER_OPEN_METRES - GRIPPER_CLOSED_METRES;
  float fraction = (float)(servoAngle - GRIPPER_CLOSED_ANGLE) /
                   (float)(GRIPPER_OPEN_ANGLE - GRIPPER_CLOSED_ANGLE);
  return GRIPPER_CLOSED_METRES + fraction * span;
}

void printAngle() {
  Serial.print("asked for: ");
  Serial.print(angle);

  // This is the "reading" - but see the note at the top of the file. The Servo
  // library is only repeating what we told it, so this always matches.
  Serial.print(" deg   servo.read(): ");
  Serial.print(gripperServo.read());

  Serial.print(" deg   ROS would call this: ");
  Serial.print(angleToMetres(angle), 4);
  Serial.print(" m");

  if (angle <= SAFE_LOWEST_ANGLE) {
    Serial.print("   (fully closed)");
  }
  if (angle >= SAFE_HIGHEST_ANGLE) {
    Serial.print("   (fully open)");
  }
  Serial.println();
}

// Send the servo to the angle we have chosen and say what happened.
void moveServo() {
  angle = constrain(angle, SAFE_LOWEST_ANGLE, SAFE_HIGHEST_ANGLE);
  gripperServo.write(angle);
  delay(300);                 // give it a moment to actually get there
  printAngle();
}

// Walk all the way out and back, slowly, so you can watch for the point where
// the gripper stops moving or starts to strain.
void sweep() {
  Serial.println("sweeping across the whole range and back, watch the gripper");

  for (int a = SAFE_LOWEST_ANGLE; a <= SAFE_HIGHEST_ANGLE; a = a + STEP_SIZE) {
    gripperServo.write(a);
    Serial.print("  ");
    Serial.println(a);
    delay(200);
  }
  for (int a = SAFE_HIGHEST_ANGLE; a >= SAFE_LOWEST_ANGLE; a = a - STEP_SIZE) {
    gripperServo.write(a);
    Serial.print("  ");
    Serial.println(a);
    delay(200);
  }

  Serial.println("sweep finished");
  moveServo();                // go back to where we were before
}

void handleInput(String input) {
  if (input.length() == 0) {
    return;
  }

  if (input == "+") {
    angle = angle + STEP_SIZE;
    moveServo();
  }
  else if (input == "-") {
    angle = angle - STEP_SIZE;
    moveServo();
  }
  else if (input == "s" || input == "S") {
    sweep();
  }
  else if (input == "?") {
    printHelp();
  }
  else {
    // Anything else should be a number. toInt() gives 0 for rubbish, so we
    // check the first character looks like a digit before believing it.
    char first = input.charAt(0);
    if (first >= '0' && first <= '9') {
      angle = input.toInt();
      moveServo();
    } else {
      Serial.println("sorry, I did not understand that - type ? for help");
    }
  }
}


// ===========================================================================
// setup and loop
// ===========================================================================

void setup() {
  Serial.begin(115200);

  gripperServo.attach(SERVO_PIN);
  // If your servo will not reach both ends, try telling attach() how long the
  // pulses should be:  gripperServo.attach(SERVO_PIN, 600, 2500);

  gripperServo.write(angle);

  printHelp();
  printAngle();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    handleInput(input);
  }
}
