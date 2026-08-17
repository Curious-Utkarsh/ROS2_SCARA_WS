/*
   SCARA Robot - MoveIt control firmware
   Drives the real arm from ROS 2 / MoveIt over USB serial.

   Hardware: Arduino Uno + CNC Shield V3 + A4988/DRV8825 drivers
   Libraries: AccelStepper, Servo   (install AccelStepper from Library Manager)

   ---------------------------------------------------------------------------
   WHAT ROS SENDS US

     "J 0.5 -0.05 0.0 0.0 0.0"    <- go to these joint positions
                                     column shoulder forearm wrist gripper
                                     in radians and metres
     "H"   this position is now zero
     "F"   free the motors (you can push the arm by hand)
     "L"   lock the motors (holding torque back on)
     "C 1" run the conveyor belt      "C 0" stop it
     "?"   print the banner

   WHAT WE SEND BACK

     "R"        I have started up and I am ready
     "B"        Busy - I got your target and I am moving
     "D"        Done - every motor has arrived
     "# ..."    a message for a human to read

   ROS waits for "D" before sending the next target, so we only ever have one
   job at a time. That is what keeps this program simple.

   ---------------------------------------------------------------------------
   THERE ARE NO SENSORS ON THIS ROBOT

   The motors are steppers with no encoders, so we can never actually know where
   the arm is. We just count the steps we send out and hope. That means:

     *** PARK THE ARM AT ITS ZERO POSITION BEFORE YOU TURN THE POWER ON ***

   because when this program starts it simply declares "wherever I am is zero".

   ---------------------------------------------------------------------------
   HOW THIS FILE IS LAID OUT

     1. Pins
     2. Numbers you may need to change (gearing, speed, limits)
     3. Small helper functions
     4. setup()  - runs once
     5. loop()   - runs over and over
*/

#include <AccelStepper.h>
#include <Servo.h>


// ===========================================================================
// 1. PINS
// ===========================================================================

// On a CNC Shield V3 one pin turns all four drivers on or off together.
// It is "active low", which means LOW = motors on, HIGH = motors off.
#define ENABLE_PIN 8

// AccelStepper(1, STEP, DIR) - the 1 means "a step and direction driver".
// The shield calls these X, Y, Z and A. We call them what they actually move.
AccelStepper columnStepper(1, 2, 5);      // X axis - turns the whole column
AccelStepper shoulderStepper(1, 3, 6);    // Y axis - leadscrew, moves up/down
AccelStepper forearmStepper(1, 4, 7);     // Z axis - turns the forearm
AccelStepper wristStepper(1, 12, 13);     // A axis - turns the wrist

// The shield only has four motor slots and the arm uses all of them, so the
// gripper is a hobby servo. Its signal wire goes to the Z+ endstop socket,
// which is Arduino pin 11.
#define GRIPPER_PIN 11
Servo gripperServo;

// The conveyor belt is not a motor we control, it is a mains motor behind a
// relay: we can only switch it on and off. The relay's signal wire goes to the
// Y+ endstop socket, which is Arduino pin 10.
//
// Pin 10 is free: the four steppers use 2/5, 3/6, 4/7 and 12/13, and the servo
// has 11. CHECK YOUR SHIELD with a continuity tester before you wire it up --
// some CNC Shield revisions label the endstop headers differently, and if yours
// is not pin 10 this one number is all you change.
#define CONVEYOR_PIN 10

// Most little relay boards switch on when their input is pulled LOW. If yours
// works back to front - belt runs when it should be stopped - change this to
// false and nothing else.
const boolean CONVEYOR_ACTIVE_LOW = true;


// ===========================================================================
// 2. NUMBERS YOU MAY NEED TO CHANGE
// ===========================================================================

// --- How many steps make one full turn of a motor? ---
// A normal stepper takes 200 steps per turn. The little jumpers under each
// driver split every step into smaller ones. This robot has a jumper on M1
// only, which means quarter steps: 200 x 4 = 800.
const float STEPS_PER_MOTOR_TURN = 800.0;

const float ONE_TURN_IN_RADIANS = 6.283185;   // 2 x pi

// --- Gearing ---
// "19:1" means the motor turns 19 times to turn the joint once.
const float COLUMN_GEARING  = 19.0;   // 800 x 19 / 2pi = 2419 steps/rad
const float FOREARM_GEARING = 16.0;   // 800 x 16 / 2pi = 2037 steps/rad
const float WRIST_GEARING   = 1.0;    // <-- NOT MEASURED YET, please check!

// The shoulder is a leadscrew instead of a gear. One turn of the motor lifts
// the arm by 8 mm, which is 0.008 metres.  ->  800 / 0.008 = 100000 steps/m
//
// Measured 2026-08-16: 4600 microsteps (= 5.75 motor turns) moved the nut 45 mm.
// That implies 45 / 5.75 = 7.83 mm per turn, 2.2% short of 8 mm. Left at 8.0
// because 7.83 is not a leadscrew lead anyone makes (a T8 screw is 2 mm pitch x
// 4 starts = exactly 8 mm) and 1 mm over 45 mm is inside ruler + backlash error.
// To settle it, drive 12000 steps (15 turns) one way only and measure: 120 mm
// means 8 mm is right, 117.4 mm means the lead really is 7.83 and this should
// become 0.00783 (-> 102222 steps/m, and shoulder max_velocity 0.0117 m/s).
const float SHOULDER_MM_PER_TURN = 0.008;

// --- Which way round is each motor? ---
// If a joint moves the OPPOSITE way to the robot in RViz, change its 1 to -1.
// Doing it here is easier and safer than swapping the DIR wire on the shield,
// and it keeps the wiring diagram in the docs true.
const int COLUMN_DIRECTION   =  1;
const int SHOULDER_DIRECTION = -1;   // measured on this robot: needs reversing
const int FOREARM_DIRECTION  =  1;
const int WRIST_DIRECTION    =  1;

// --- Steps per unit of movement ---
// This is the number we actually use: how many steps to move one radian
// (for the joints that turn) or one metre (for the shoulder that slides).
// A negative value here simply means the motor spins the other way; the joint
// limits are checked before this, in joint units, so they still apply.
const float COLUMN_STEPS_PER_RADIAN   = COLUMN_DIRECTION   * STEPS_PER_MOTOR_TURN * COLUMN_GEARING / ONE_TURN_IN_RADIANS;
const float SHOULDER_STEPS_PER_METRE  = SHOULDER_DIRECTION * STEPS_PER_MOTOR_TURN / SHOULDER_MM_PER_TURN;
const float FOREARM_STEPS_PER_RADIAN  = FOREARM_DIRECTION  * STEPS_PER_MOTOR_TURN * FOREARM_GEARING / ONE_TURN_IN_RADIANS;
const float WRIST_STEPS_PER_RADIAN    = WRIST_DIRECTION    * STEPS_PER_MOTOR_TURN * WRIST_GEARING / ONE_TURN_IN_RADIANS;

// --- Top speed, in steps per second ---
// An Arduino Uno can only make about 4000 steps every second in total, shared
// between all four motors, so these four numbers add up to 3500 and no more.
// If you make them bigger the motors will start skipping steps, and a skipped
// step is invisible - the robot just quietly ends up in the wrong place.
const float COLUMN_TOP_SPEED   = 1000.0;
const float SHOULDER_TOP_SPEED = 1200.0;
const float FOREARM_TOP_SPEED  = 800.0;
const float WRIST_TOP_SPEED    = 500.0;

// How quickly a motor is allowed to build up to its top speed.
// Twice the top speed means it takes half a second to get there.
const float SPEED_UP_RATE = 2.0;

// --- How far each joint is allowed to move ---
// These are the same numbers as the robot model (the URDF) in ROS. We check
// them here as well, so a bad message can never drive the arm into itself.
const float COLUMN_LOWEST    = -2.0;    // radians
const float COLUMN_HIGHEST   =  2.0;
const float SHOULDER_LOWEST  = -0.15;   // metres
const float SHOULDER_HIGHEST =  0.02;
const float FOREARM_LOWEST   = -2.0;    // radians
const float FOREARM_HIGHEST  =  2.0;
const float WRIST_LOWEST     = -2.0;    // radians
const float WRIST_HIGHEST    =  2.0;

// --- Gripper ---
// ROS talks about the gripper in metres: 0 is open, -0.06 is closed.
// The servo talks in degrees. These two pairs tell us how to swap between them.
//
// 5 and 55 are the ends of the travel measured on this gripper with
// firmware/servo_test/servo_test.ino. Do not widen them: past those angles the
// servo pushes against a mechanical stop, which makes it buzz, get hot and
// eventually burn out.
//
// IF THE GRIPPER WORKS BACKWARDS - opens when it should close - swap these two
// numbers over. Nothing else needs to change.
const float GRIPPER_CLOSED_METRES = -0.06;
const float GRIPPER_OPEN_METRES   =  0.00;
const int   GRIPPER_CLOSED_ANGLE  = 5;
const int   GRIPPER_OPEN_ANGLE    = 55;

// The ends of the servo's travel, whichever way round the two angles above are.
// Nothing is ever allowed past these.
const int SERVO_LOWEST_ANGLE  = 5;
const int SERVO_HIGHEST_ANGLE = 55;

// A servo cannot tell us when it has finished, so we just wait this long.
const int GRIPPER_WAIT_TIME = 400;   // milliseconds


// ===========================================================================
// Things the program keeps track of while it runs
// ===========================================================================

boolean isMoving = false;             // true between sending "B" and "D"
float gripperTarget = 0.0;            // where we last told the gripper to go
unsigned long gripperFinishTime = 0;  // when the servo should be settled
boolean conveyorRunning = false;      // is the belt switched on right now?
unsigned long conveyorWrittenTime = 0;  // when we last wrote the belt pin

// How often to write the belt state to the pin again. The relay switches a
// mains motor, and the electrical noise that makes can flip a bit in the pin
// register without this sketch asking for it. Writing what we want a few times
// a second costs a couple of microseconds and turns a stuck belt into a glitch
// nobody notices.
const unsigned long CONVEYOR_REWRITE_TIME = 200;   // milliseconds


// ===========================================================================
// 3. SMALL HELPER FUNCTIONS
// ===========================================================================

// Turn the motors on. They will hold their position and resist being moved.
void lockMotors() {
  digitalWrite(ENABLE_PIN, LOW);      // low means on
}

// Turn the motors off, so you can push the arm around by hand.
void freeMotors() {
  digitalWrite(ENABLE_PIN, HIGH);     // high means off
}

// Switch the conveyor belt on or off.
void setConveyor(boolean run) {
  conveyorRunning = run;

  // pinMode again as well as digitalWrite. If noise from the belt motor ever
  // knocked the pin out of OUTPUT mode it would float, and a floating input is
  // exactly the state where a relay does whatever it likes.
  pinMode(CONVEYOR_PIN, OUTPUT);

  if (CONVEYOR_ACTIVE_LOW) {
    digitalWrite(CONVEYOR_PIN, run ? LOW : HIGH);
  } else {
    digitalWrite(CONVEYOR_PIN, run ? HIGH : LOW);
  }
  conveyorWrittenTime = millis();
}

void printBanner() {
  Serial.println("# SCARA MoveIt control");
  Serial.println("# send: J column shoulder forearm wrist gripper");
  Serial.println("# also: H = zero here, F = free motors, L = lock motors");
  Serial.println("# belt: C 1 = run conveyor, C 0 = stop conveyor");
}

// Change a gripper position in metres into an angle for the servo.
int gripperAngleFor(float metres) {
  // map() only works with whole numbers, so we count in millimetres instead
  // of metres: -0.06 metres becomes -60 millimetres.
  int millimetres = metres * 1000;
  int closedMillimetres = GRIPPER_CLOSED_METRES * 1000;    // -60
  int openMillimetres   = GRIPPER_OPEN_METRES * 1000;      //   0

  int servoAngle = map(millimetres,
                       closedMillimetres, openMillimetres,
                       GRIPPER_CLOSED_ANGLE, GRIPPER_OPEN_ANGLE);

  // Never ask the servo for an angle it cannot physically reach.
  return constrain(servoAngle, SERVO_LOWEST_ANGLE, SERVO_HIGHEST_ANGLE);
}

// Have all four motors reached the place we sent them?
boolean allMotorsArrived() {
  if (columnStepper.distanceToGo() != 0) return false;
  if (shoulderStepper.distanceToGo() != 0) return false;
  if (forearmStepper.distanceToGo() != 0) return false;
  if (wristStepper.distanceToGo() != 0) return false;
  return true;
}

// Forget where we think we are and call this spot zero.
void zeroHere() {
  columnStepper.setCurrentPosition(0);
  shoulderStepper.setCurrentPosition(0);
  forearmStepper.setCurrentPosition(0);
  wristStepper.setCurrentPosition(0);
  isMoving = false;
  Serial.println("# this position is now zero");
  Serial.println("D");
}

// Count how many numbers are in a piece of text like "0.5 -0.05 0 0 0".
// We use this to throw away messages that arrived broken.
int howManyNumbers(String text) {
  int count = 0;
  text.trim();
  while (text.length() > 0) {
    count = count + 1;
    int space = text.indexOf(' ');
    if (space == -1) {
      break;                          // no more spaces, that was the last one
    }
    text = text.substring(space + 1);
    text.trim();
  }
  return count;
}

// Take the first number off the front of some text and give it back.
//
// The & in "String &text" means we are handed the caller's own text, not a
// copy, so when we chop the number off the front the caller sees that too.
// Call it five times and you get the five numbers one after another.
float takeNextNumber(String &text) {
  text.trim();
  int space = text.indexOf(' ');
  String piece;

  if (space == -1) {
    piece = text;                     // the last number, no space after it
    text = "";
  } else {
    piece = text.substring(0, space);
    text = text.substring(space + 1);
  }
  return piece.toFloat();
}

// Handle a "J ..." message: work out where to go and start moving.
void startNewMove(String message) {
  String numbers = message.substring(1);   // chop off the "J"

  // If the message did not arrive in one piece, ignore it completely.
  // Moving to half a message would be worse than not moving at all.
  if (howManyNumbers(numbers) != 5) {
    Serial.println("# broken message, ignoring it");
    return;
  }

  float column   = takeNextNumber(numbers);
  float shoulder = takeNextNumber(numbers);
  float forearm  = takeNextNumber(numbers);
  float wrist    = takeNextNumber(numbers);
  float gripper  = takeNextNumber(numbers);

  // constrain() keeps a number between two limits.
  column   = constrain(column,   COLUMN_LOWEST,   COLUMN_HIGHEST);
  shoulder = constrain(shoulder, SHOULDER_LOWEST, SHOULDER_HIGHEST);
  forearm  = constrain(forearm,  FOREARM_LOWEST,  FOREARM_HIGHEST);
  wrist    = constrain(wrist,    WRIST_LOWEST,    WRIST_HIGHEST);
  gripper  = constrain(gripper,  GRIPPER_CLOSED_METRES, GRIPPER_OPEN_METRES);

  // Turn the positions into step counts and tell each motor where to go.
  // moveTo() does not move anything by itself - run() down in loop() does.
  columnStepper.moveTo(column * COLUMN_STEPS_PER_RADIAN);
  shoulderStepper.moveTo(shoulder * SHOULDER_STEPS_PER_METRE);
  forearmStepper.moveTo(forearm * FOREARM_STEPS_PER_RADIAN);
  wristStepper.moveTo(wrist * WRIST_STEPS_PER_RADIAN);

  // Only bother the servo if the gripper actually needs to change, otherwise
  // every single message would make us wait for it again.
  if (gripper != gripperTarget) {
    gripperTarget = gripper;
    gripperServo.write(gripperAngleFor(gripper));
    gripperFinishTime = millis() + GRIPPER_WAIT_TIME;
  }

  isMoving = true;
  Serial.println("B");
}

// Read one line from ROS and do what it says.
void readMessage() {
  String message = Serial.readStringUntil('\n');
  message.trim();

  if (message.length() == 0) {
    return;
  }

  char firstLetter = message.charAt(0);

  if (firstLetter == 'J') {
    startNewMove(message);
  }
  else if (firstLetter == 'H') {
    zeroHere();
  }
  else if (firstLetter == 'F') {
    freeMotors();
    Serial.println("# motors are free, you can move the arm by hand");
  }
  else if (firstLetter == 'L') {
    lockMotors();
    Serial.println("# motors are locked");
  }
  else if (firstLetter == 'C') {
    // "C 1" or "C 0". Anything that is not a 1 stops the belt, because the
    // safe thing to do with a half-read message is to stop.
    //
    // Read the argument itself instead of hunting for a '1' anywhere in the
    // line. A '1' can turn up in a line that arrived spliced onto the tail of
    // a J message ("...0.1000C 0"), and the old test would have read that as
    // "run" - a stop command that starts the belt.
    String argument = message.substring(1);
    argument.trim();
    boolean run = (argument.length() > 0 && argument.charAt(0) == '1');
    setConveyor(run);
    Serial.println(run ? "# conveyor running" : "# conveyor stopped");
  }
  else if (firstLetter == '?') {
    printBanner();
  }
}


// ===========================================================================
// 4. SETUP - this runs once, when the Arduino starts
// ===========================================================================

void setup() {
  Serial.begin(115200);

  // Don't wait longer than this for the rest of a line to turn up. Without it
  // the Arduino would sit and wait a whole second and the motors would pause.
  Serial.setTimeout(20);

  pinMode(ENABLE_PIN, OUTPUT);
  lockMotors();

  // The belt starts STOPPED. Nothing should start moving on its own just
  // because somebody plugged the USB cable in.
  pinMode(CONVEYOR_PIN, OUTPUT);
  setConveyor(false);

  columnStepper.setMaxSpeed(COLUMN_TOP_SPEED);
  shoulderStepper.setMaxSpeed(SHOULDER_TOP_SPEED);
  forearmStepper.setMaxSpeed(FOREARM_TOP_SPEED);
  wristStepper.setMaxSpeed(WRIST_TOP_SPEED);

  columnStepper.setAcceleration(COLUMN_TOP_SPEED * SPEED_UP_RATE);
  shoulderStepper.setAcceleration(SHOULDER_TOP_SPEED * SPEED_UP_RATE);
  forearmStepper.setAcceleration(FOREARM_TOP_SPEED * SPEED_UP_RATE);
  wristStepper.setAcceleration(WRIST_TOP_SPEED * SPEED_UP_RATE);

  // We have no sensors, so we simply say that wherever the arm is, is zero.
  columnStepper.setCurrentPosition(0);
  shoulderStepper.setCurrentPosition(0);
  forearmStepper.setCurrentPosition(0);
  wristStepper.setCurrentPosition(0);

  // If your servo does not open or close all the way, try giving attach() the
  // pulse lengths as well, like this:  gripperServo.attach(GRIPPER_PIN, 600, 2500);
  gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);

  printBanner();
  Serial.println("R");        // tell ROS we are ready
}


// ===========================================================================
// 5. LOOP - this runs over and over, thousands of times a second
// ===========================================================================

void loop() {

  // Did ROS send us anything?
  if (Serial.available() > 0) {
    readMessage();
  }

  // run() makes each motor take a step if it is time for one. It has to be
  // called as often as possible, which is why there is nothing slow in here.
  columnStepper.run();
  shoulderStepper.run();
  forearmStepper.run();
  wristStepper.run();

  // Say again what the belt should be doing. Nothing here changes the state,
  // it only writes the state we already hold back onto the pin.
  if (millis() - conveyorWrittenTime >= CONVEYOR_REWRITE_TIME) {
    setConveyor(conveyorRunning);
  }

  // Have we finished the job? Say so once, and then wait for the next one.
  if (isMoving && allMotorsArrived() && millis() >= gripperFinishTime) {
    isMoving = false;
    Serial.println("D");
  }
}
