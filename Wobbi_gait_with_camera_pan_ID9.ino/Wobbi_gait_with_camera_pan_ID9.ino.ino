#include <SoftwareSerial.h>

// D10 = RX unused
// D11 = TX -> RX pins on both Hiwonder bus servo controllers
SoftwareSerial busSerial(10, 11);

// ==========================
// Servo IDs: legs
// ==========================
const byte L_ANKLE_PITCH = 1;
const byte L_KNEE        = 2;
const byte L_HIP_PITCH   = 3;
const byte L_HIP_YAW     = 4;

const byte R_HIP_YAW     = 5;
const byte R_HIP_PITCH   = 6;
const byte R_KNEE        = 7;
const byte R_ANKLE_PITCH = 8;

// ==========================
// Servo ID: camera pan servo
// ID9 is mounted on top of the legs
// 500 = centre
// 400 = robot right
// 600 = robot left
// ==========================
const byte PAN_SERVO_ID = 9;

const int PAN_CENTER = 500;
const int PAN_RIGHT  = 400;
const int PAN_LEFT   = 600;

const int PAN_MIN = 400;
const int PAN_MAX = 600;

int currentPanPos = PAN_CENTER;

// ==========================
// Neutral pose for leg servos
// index 0 unused
// ==========================
int neutral[9] = {
  0,
  540, // ID1 L_Ankle_Pitch
  470, // ID2 L_Knee
  530, // ID3 L_Hip_Pitch
  520, // ID4 L_Hip_Yaw
  540, // ID5 R_Hip_Yaw
  500, // ID6 R_Hip_Pitch
  550, // ID7 R_Knee
  550  // ID8 R_Ankle_Pitch
};

// ==========================
// Direction mapping
//
// ID1-4: 480 = physical +, 520 = physical -
// physical + means servo value decreases
//
// ID5-8: 480 = physical -, 520 = physical +
// physical + means servo value increases
// ==========================
int dirSign[9] = {
  0,
  -1, // ID1
  -1, // ID2
  -1, // ID3
  -1, // ID4
   1, // ID5
   1, // ID6
   1, // ID7
   1  // ID8
};

// ==========================
// Safety range for leg servos
// ==========================
const int SERVO_MIN = 180;
const int SERVO_MAX = 820;

// ==========================
// Gait tuning parameters
// ==========================
int SHIFT_TO_LEFT  = 50;
int SHIFT_TO_RIGHT = 55;

int LEFT_HIP_FORWARD  = 230;
int RIGHT_HIP_FORWARD = 230;

int LEFT_KNEE_LIFT  = 55;
int RIGHT_KNEE_LIFT = 55;

int LEFT_TOE_UP  = 100;
int RIGHT_TOE_UP = 100;

int LEFT_KNEE_PLACE  = 20;
int RIGHT_KNEE_PLACE = 20;

int LEFT_SUPPORT_KNEE_BEND  = 25;
int RIGHT_SUPPORT_KNEE_BEND = 25;

int LEFT_SUPPORT_ANKLE_COMP  = 20;
int RIGHT_SUPPORT_ANKLE_COMP = 20;

int MOVE_TIME_SHIFT = 2400;
int MOVE_TIME_LIFT  = 2500;
int MOVE_TIME_PLACE = 2600;
int MOVE_TIME_FAST  = 1600;

// ==========================
// Stable stop pose tuning
// This is used when receiving x.
// It is more stable than pure neutral pose.
// ==========================
int STOP_KNEE_BEND = 15;
int STOP_ANKLE_COMP = 8;

bool repeatGait = false;

// ==========================
// Basic helper functions
// ==========================
int clampValue(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

void writeLowHigh(uint16_t value) {
  busSerial.write(value & 0xFF);
  busSerial.write((value >> 8) & 0xFF);
}

void copyNeutral(int pose[]) {
  for (byte id = 1; id <= 8; id++) {
    pose[id] = neutral[id];
  }
}

// physicalOffset:
// + means physical + direction in your direction table
// - means physical - direction in your direction table
void addPhysicalOffset(int pose[], byte servoID, int physicalOffset) {
  pose[servoID] = pose[servoID] + dirSign[servoID] * physicalOffset;
}

// ==========================
// Send one servo position
// Used for ID9 camera pan servo
// ==========================
void sendSingleServo(byte servoID, int pos, uint16_t moveTime) {
  pos = clampValue(pos, 180, 820);

  byte count = 1;

  busSerial.write(0x55);
  busSerial.write(0x55);

  busSerial.write(count * 3 + 5);
  busSerial.write(0x03);  // CMD_SERVO_MOVE
  busSerial.write(count);

  writeLowHigh(moveTime);

  busSerial.write(servoID);
  writeLowHigh(pos);
}

void movePanServo(int pos) {
  pos = clampValue(pos, PAN_MIN, PAN_MAX);
  currentPanPos = pos;

  sendSingleServo(PAN_SERVO_ID, currentPanPos, 300);

  Serial.print("Pan servo ID9 -> ");
  Serial.println(currentPanPos);
}

void centerPanServo() {
  movePanServo(PAN_CENTER);
}

// ==========================
// Serial command reading
// This allows Python to send:
// n, w, g, r, x, u
// or P500 / P400 / P600 for ID9
// ==========================
void readSerialCommand();

void smartDelay(unsigned long holdTime) {
  unsigned long startTime = millis();

  while (millis() - startTime < holdTime) {
    readSerialCommand();
    delay(5);
  }
}

// ==========================
// Send leg pose: IDs 1-8 only
// ID9 is controlled separately
// ==========================
void sendPose(const int pose[], uint16_t moveTime, uint16_t holdTime) {
  byte count = 8;

  busSerial.write(0x55);
  busSerial.write(0x55);

  busSerial.write(count * 3 + 5);
  busSerial.write(0x03);  // CMD_SERVO_MOVE
  busSerial.write(count);

  writeLowHigh(moveTime);

  for (byte id = 1; id <= 8; id++) {
    int pos = clampValue(pose[id], SERVO_MIN, SERVO_MAX);
    busSerial.write(id);
    writeLowHigh(pos);
  }

  smartDelay(holdTime);
}

// ==========================
// Basic poses
// ==========================
void neutralLegPose() {
  int pose[9];
  copyNeutral(pose);

  Serial.println("Leg neutral pose");
  sendPose(pose, 1500, 1000);
}

// Stable stop pose:
// Used for x command.
// Legs return to a slightly bent stable pose.
// ID9 keeps current direction.
void stableStopLegPose() {
  int pose[9];
  copyNeutral(pose);

  // Slight knee bend for a softer and more stable standing pose
  addPhysicalOffset(pose, L_KNEE, STOP_KNEE_BEND);
  addPhysicalOffset(pose, R_KNEE, STOP_KNEE_BEND);

  // Small ankle compensation to help both feet stay planted
  addPhysicalOffset(pose, L_ANKLE_PITCH, STOP_ANKLE_COMP);
  addPhysicalOffset(pose, R_ANKLE_PITCH, STOP_ANKLE_COMP);

  Serial.println("Stable stop pose: legs only, pan keeps current direction");
  sendPose(pose, 1500, 1000);
}

void neutralAllPose() {
  repeatGait = false;
  neutralLegPose();
  centerPanServo();

  Serial.println("All neutral: legs + camera pan");
}

// Shift weight to the left leg
void shiftToLeftSupport() {
  int pose[9];
  copyNeutral(pose);

  addPhysicalOffset(pose, L_HIP_YAW, +SHIFT_TO_LEFT);
  addPhysicalOffset(pose, R_HIP_YAW, +SHIFT_TO_LEFT);

  Serial.println("1. Shift weight to LEFT leg");
  sendPose(pose, MOVE_TIME_SHIFT, 900);
}

// Shift weight to the right leg
void shiftToRightSupport() {
  int pose[9];
  copyNeutral(pose);

  addPhysicalOffset(pose, L_HIP_YAW, -SHIFT_TO_RIGHT);
  addPhysicalOffset(pose, R_HIP_YAW, -SHIFT_TO_RIGHT);

  Serial.println("4. Shift weight to RIGHT leg");
  sendPose(pose, MOVE_TIME_SHIFT, 900);
}

// ==========================
// Right leg cycle
// Left leg supports the body; right leg is the swing leg
// ==========================
void rightLegLift() {
  int pose[9];
  copyNeutral(pose);

  addPhysicalOffset(pose, L_HIP_YAW, +SHIFT_TO_LEFT);
  addPhysicalOffset(pose, R_HIP_YAW, +SHIFT_TO_LEFT);

  addPhysicalOffset(pose, R_HIP_PITCH, RIGHT_HIP_FORWARD);
  addPhysicalOffset(pose, R_KNEE, RIGHT_KNEE_LIFT);
  addPhysicalOffset(pose, R_ANKLE_PITCH, RIGHT_TOE_UP);

  Serial.println("2. Right leg bend / lift");
  sendPose(pose, MOVE_TIME_LIFT, 900);
}

void rightLegPlaceWithLeftSupportBend() {
  int pose[9];
  copyNeutral(pose);

  addPhysicalOffset(pose, L_HIP_YAW, +SHIFT_TO_LEFT);
  addPhysicalOffset(pose, R_HIP_YAW, +SHIFT_TO_LEFT);

  addPhysicalOffset(pose, R_HIP_PITCH, RIGHT_HIP_FORWARD);
  addPhysicalOffset(pose, R_KNEE, RIGHT_KNEE_PLACE);
  addPhysicalOffset(pose, R_ANKLE_PITCH, RIGHT_TOE_UP / 3);

  addPhysicalOffset(pose, L_KNEE, LEFT_SUPPORT_KNEE_BEND);
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_SUPPORT_ANKLE_COMP);

  Serial.println("3. Right foot forward place + LEFT support knee bend");
  sendPose(pose, MOVE_TIME_PLACE, 1000);
}

// ==========================
// Left leg cycle
// Right leg supports the body; left leg is the swing leg
// ==========================
void leftLegLift() {
  int pose[9];
  copyNeutral(pose);

  addPhysicalOffset(pose, L_HIP_YAW, -SHIFT_TO_RIGHT);
  addPhysicalOffset(pose, R_HIP_YAW, -SHIFT_TO_RIGHT);

  addPhysicalOffset(pose, L_HIP_PITCH, LEFT_HIP_FORWARD);
  addPhysicalOffset(pose, L_KNEE, LEFT_KNEE_LIFT);
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_TOE_UP);

  Serial.println("5. Left leg bend / lift");
  sendPose(pose, MOVE_TIME_LIFT, 900);
}

void leftLegPlaceWithRightSupportBend() {
  int pose[9];
  copyNeutral(pose);

  addPhysicalOffset(pose, L_HIP_YAW, -SHIFT_TO_RIGHT);
  addPhysicalOffset(pose, R_HIP_YAW, -SHIFT_TO_RIGHT);

  addPhysicalOffset(pose, L_HIP_PITCH, LEFT_HIP_FORWARD);
  addPhysicalOffset(pose, L_KNEE, LEFT_KNEE_PLACE);
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_TOE_UP / 3);

  addPhysicalOffset(pose, R_KNEE, RIGHT_SUPPORT_KNEE_BEND);
  addPhysicalOffset(pose, R_ANKLE_PITCH, RIGHT_SUPPORT_ANKLE_COMP);

  Serial.println("6. Left foot forward place + RIGHT support knee bend");
  sendPose(pose, MOVE_TIME_PLACE, 1000);
}

void transferBackToLeftAfterLeftPlace() {
  int pose[9];
  copyNeutral(pose);

  addPhysicalOffset(pose, L_HIP_YAW, +SHIFT_TO_LEFT);
  addPhysicalOffset(pose, R_HIP_YAW, +SHIFT_TO_LEFT);

  addPhysicalOffset(pose, L_KNEE, LEFT_SUPPORT_KNEE_BEND / 2);
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_SUPPORT_ANKLE_COMP / 2);

  Serial.println("7. Transfer weight back to LEFT leg");
  sendPose(pose, MOVE_TIME_SHIFT, 900);
}

// ==========================
// Test sequences
// ==========================
void weightShiftTest() {
  Serial.println("Weight shift test start");

  stableStopLegPose();
  shiftToLeftSupport();
  shiftToRightSupport();
  shiftToLeftSupport();
  stableStopLegPose();

  Serial.println("Weight shift test end");
}

void gaitCycleFromLeftSupport() {
  rightLegLift();
  rightLegPlaceWithLeftSupportBend();

  shiftToRightSupport();

  leftLegLift();
  leftLegPlaceWithRightSupportBend();

  transferBackToLeftAfterLeftPlace();
}

void improvedGaitOnce() {
  Serial.println("One improved gait cycle start");

  stableStopLegPose();
  shiftToLeftSupport();
  gaitCycleFromLeftSupport();
  stableStopLegPose();

  Serial.println("One improved gait cycle end");
}

// ==========================
// Optional: unload knee servos
// Robot must be supported before using this
// ==========================
void unloadKneeServos() {
  busSerial.write(0x55);
  busSerial.write(0x55);

  busSerial.write(0x05); // count + 3 = 2 + 3
  busSerial.write(0x14); // CMD_MULT_SERVO_UNLOAD
  busSerial.write(0x02); // servo count

  busSerial.write(L_KNEE);
  busSerial.write(R_KNEE);

  Serial.println("Knee servos unloaded. Robot must be supported.");
}

// ==========================
// Command handling
// ==========================
void handleCommand(char cmd) {
  if (cmd == 'n') {
    repeatGait = false;
    neutralAllPose();
  }

  else if (cmd == 'w') {
    repeatGait = false;
    weightShiftTest();
  }

  else if (cmd == 'g') {
    repeatGait = false;
    improvedGaitOnce();
  }

  else if (cmd == 'r') {
    if (!repeatGait) {
      Serial.println("Repeat gait started. Send x to stop.");
      stableStopLegPose();
      shiftToLeftSupport();
      repeatGait = true;
    }
  }

  else if (cmd == 'x') {
    Serial.println("Repeat gait stopped. Legs return to stable stop pose. Pan servo keeps current direction.");
    repeatGait = false;
    stableStopLegPose();
  }

  else if (cmd == 'u') {
    repeatGait = false;
    unloadKneeServos();
  }
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '\n' || cmd == '\r' || cmd == ' ') {
      continue;
    }

    // Python sends P500, P400, P600...
    if (cmd == 'P' || cmd == 'p') {
      int panPos = Serial.parseInt();
      movePanServo(panPos);
    }

    else {
      handleCommand(cmd);
    }
  }
}

// ==========================
// Arduino setup / loop
// ==========================
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20);

  busSerial.begin(9600);

  delay(2000);

  Serial.println("Wobbi gait + camera pan controller ready.");
  Serial.println("Commands:");
  Serial.println("n = all neutral: legs + pan center");
  Serial.println("w = weight shift test");
  Serial.println("g = one complete gait cycle");
  Serial.println("r = repeat gait loop");
  Serial.println("x = stop walking and return legs to stable stop pose");
  Serial.println("u = unload knee servos, only when robot is supported");
  Serial.println("P500 = pan center");
  Serial.println("P400 = pan robot right");
  Serial.println("P600 = pan robot left");

  neutralAllPose();
}

void loop() {
  readSerialCommand();

  if (repeatGait) {
    gaitCycleFromLeftSupport();
    readSerialCommand();
  }
}