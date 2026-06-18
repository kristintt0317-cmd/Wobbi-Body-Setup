#include <SoftwareSerial.h>

// D10 = RX unused
// D11 = TX -> RX pins on both servo controllers
SoftwareSerial busSerial(10, 11);

// ==========================
// Servo IDs
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
// Neutral pose
// ID4 has been adjusted to 520
// index 0 unused
// ==========================
int neutral[9] = {
  0,
  540, // ID1 L_Ankle_Pitch
  470, // ID2 L_Knee / L_Ankle
  550, // ID3 L_Hip_Pitch
  520, // ID4 L_Hip_Yaw
  540, // ID5 R_Hip_Yaw
  500, // ID6 R_Hip_Pitch
  550, // ID7 R_Knee / R_Ankle
  550  // ID8 R_Ankle_Pitch
};

// ==========================
// Direction mapping from your table
//
// ID1-4: 480 = +, 520 = -
// physical + = servo value decreases
//
// ID5-8: 480 = -, 520 = +
// physical + = servo value increases
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
// Safety range
// Large-amplitude tuning version; do not run at the mechanical limit immediately
// If jamming, overheating, or structural collision occurs, revert to 260-740 first
// ==========================
const int SERVO_MIN = 180;
const int SERVO_MAX = 820;

// ==========================
// Gait tuning parameters
// Left and right parameters are separated to make asymmetry easier to correct
// ==========================

// Weight-shift amplitude
int SHIFT_TO_LEFT  = 50;
int SHIFT_TO_RIGHT = 55;

// Forward swing amplitude of the non-support leg
int LEFT_HIP_FORWARD  = 200;
int RIGHT_HIP_FORWARD = 200;

// Bend amplitude of the non-support leg
// If the knee servos overheat, reduce this to 35 or 25 first
int LEFT_KNEE_LIFT  = 55;
int RIGHT_KNEE_LIFT = 55;

// Toe-up amplitude of the non-support leg
int LEFT_TOE_UP  = 100;
int RIGHT_TOE_UP = 100;

// Small knee bend retained during foot placement
int LEFT_KNEE_PLACE  = 20;
int RIGHT_KNEE_PLACE = 20;

// Slight bend of the support leg while bearing weight
int LEFT_SUPPORT_KNEE_BEND  = 25;
int RIGHT_SUPPORT_KNEE_BEND = 25;

// Ankle compensation for the support leg
int LEFT_SUPPORT_ANKLE_COMP  = 15;
int RIGHT_SUPPORT_ANKLE_COMP = 15;

// Motion timing: slowed down because the feet are heavy
int MOVE_TIME_SHIFT = 2400;
int MOVE_TIME_LIFT  = 2500;
int MOVE_TIME_PLACE = 2600;
int MOVE_TIME_FAST  = 1600;

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
// + means physical + direction in your table
// - means physical - direction in your table
void addPhysicalOffset(int pose[], byte servoID, int physicalOffset) {
  pose[servoID] = pose[servoID] + dirSign[servoID] * physicalOffset;
}

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

  delay(holdTime);
}

// ==========================
// Basic poses
// ==========================
void neutralPose() {
  int pose[9];
  copyNeutral(pose);

  Serial.println("Neutral pose");
  sendPose(pose, 1500, 1000);
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
// Left leg supports the body; right leg acts as the non-support leg
// ==========================

// Bend / lift the right leg
void rightLegLift() {
  int pose[9];
  copyNeutral(pose);

  // Keep the weight on the left leg
  addPhysicalOffset(pose, L_HIP_YAW, +SHIFT_TO_LEFT);
  addPhysicalOffset(pose, R_HIP_YAW, +SHIFT_TO_LEFT);

  // Swing the right leg forward
  addPhysicalOffset(pose, R_HIP_PITCH, RIGHT_HIP_FORWARD);

  // Bend the right knee
  addPhysicalOffset(pose, R_KNEE, RIGHT_KNEE_LIFT);

  // Lift the right toe
  addPhysicalOffset(pose, R_ANKLE_PITCH, RIGHT_TOE_UP);

  Serial.println("2. Right leg bend / lift");
  sendPose(pose, MOVE_TIME_LIFT, 900);
}

// Place the right leg forward while bending the left support leg
void rightLegPlaceWithLeftSupportBend() {
  int pose[9];
  copyNeutral(pose);

  // Keep the weight on the left leg until the right foot is placed securely
  addPhysicalOffset(pose, L_HIP_YAW, +SHIFT_TO_LEFT);
  addPhysicalOffset(pose, R_HIP_YAW, +SHIFT_TO_LEFT);

  // Keep the right leg in the forward position
  addPhysicalOffset(pose, R_HIP_PITCH, RIGHT_HIP_FORWARD);

  // Reduce the right knee from a large bend to a small bend to simulate foot placement
  addPhysicalOffset(pose, R_KNEE, RIGHT_KNEE_PLACE);

  // Bring the right ankle closer to flat while keeping a slight toe-up angle
  addPhysicalOffset(pose, R_ANKLE_PITCH, RIGHT_TOE_UP / 3);

  // Slightly bend the left leg as the support leg
  addPhysicalOffset(pose, L_KNEE, LEFT_SUPPORT_KNEE_BEND);

  // Add a small compensation to the left ankle
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_SUPPORT_ANKLE_COMP);

  Serial.println("3. Right foot forward place + LEFT support knee bend");
  sendPose(pose, MOVE_TIME_PLACE, 1000);
}

// ==========================
// Left leg cycle
// Right leg supports the body; left leg acts as the non-support leg
// ==========================

// Bend / lift the left leg
void leftLegLift() {
  int pose[9];
  copyNeutral(pose);

  // Keep the weight on the right leg
  addPhysicalOffset(pose, L_HIP_YAW, -SHIFT_TO_RIGHT);
  addPhysicalOffset(pose, R_HIP_YAW, -SHIFT_TO_RIGHT);

  // Swing the left leg forward
  addPhysicalOffset(pose, L_HIP_PITCH, LEFT_HIP_FORWARD);

  // Bend the left knee
  addPhysicalOffset(pose, L_KNEE, LEFT_KNEE_LIFT);

  // Lift the left toe
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_TOE_UP);

  Serial.println("5. Left leg bend / lift");
  sendPose(pose, MOVE_TIME_LIFT, 900);
}

// Place the left leg forward while bending the right support leg
void leftLegPlaceWithRightSupportBend() {
  int pose[9];
  copyNeutral(pose);

  // Keep the weight on the right leg until the left foot is placed securely
  addPhysicalOffset(pose, L_HIP_YAW, -SHIFT_TO_RIGHT);
  addPhysicalOffset(pose, R_HIP_YAW, -SHIFT_TO_RIGHT);

  // Keep the left leg in the forward position
  addPhysicalOffset(pose, L_HIP_PITCH, LEFT_HIP_FORWARD);

  // Reduce the left knee from a large bend to a small bend to simulate foot placement
  addPhysicalOffset(pose, L_KNEE, LEFT_KNEE_PLACE);

  // Bring the left ankle closer to flat
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_TOE_UP / 3);

  // Slightly bend the right leg as the support leg
  addPhysicalOffset(pose, R_KNEE, RIGHT_SUPPORT_KNEE_BEND);

  // Add compensation to the right ankle
  addPhysicalOffset(pose, R_ANKLE_PITCH, RIGHT_SUPPORT_ANKLE_COMP);

  Serial.println("6. Left foot forward place + RIGHT support knee bend");
  sendPose(pose, MOVE_TIME_PLACE, 1000);
}

// After the left leg is placed, shift the weight back to the left leg for the next cycle
void transferBackToLeftAfterLeftPlace() {
  int pose[9];
  copyNeutral(pose);

  // Shift the weight back to the left leg to prepare for the next right-leg lift
  addPhysicalOffset(pose, L_HIP_YAW, +SHIFT_TO_LEFT);
  addPhysicalOffset(pose, R_HIP_YAW, +SHIFT_TO_LEFT);

  // Keep a slight support bend after the left leg has just been placed
  addPhysicalOffset(pose, L_KNEE, LEFT_SUPPORT_KNEE_BEND / 2);
  addPhysicalOffset(pose, L_ANKLE_PITCH, LEFT_SUPPORT_ANKLE_COMP / 2);

  Serial.println("7. Transfer weight back to LEFT leg, ready for next cycle");
  sendPose(pose, MOVE_TIME_SHIFT, 900);
}

// ==========================
// Test sequences
// ==========================
void weightShiftTest() {
  Serial.println("Weight shift test start");

  neutralPose();
  shiftToLeftSupport();
  shiftToRightSupport();
  shiftToLeftSupport();

  Serial.println("Weight shift test end");
}

// One complete cycle: starts from left support and returns to left support
void gaitCycleFromLeftSupport() {
  rightLegLift();
  rightLegPlaceWithLeftSupportBend();

  shiftToRightSupport();

  leftLegLift();
  leftLegPlaceWithRightSupportBend();

  transferBackToLeftAfterLeftPlace();
}

// Single-cycle test: start from neutral, shift to the left leg, then run one cycle
void improvedGaitOnce() {
  Serial.println("Improved continuous gait cycle start");

  neutralPose();
  shiftToLeftSupport();
  gaitCycleFromLeftSupport();

  Serial.println("Improved continuous gait cycle end");
}

// ==========================
// Optional: unload knee servos
// The robot must be supported before unloading the servos
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
    neutralPose();
  }

  if (cmd == 'w') {
    repeatGait = false;
    weightShiftTest();
  }

  if (cmd == 'g') {
    repeatGait = false;
    improvedGaitOnce();
  }

  if (cmd == 'r') {
    Serial.println("Repeat continuous gait started. Send x to stop.");

    // Before continuous looping, perform the initial weight shift only once
    neutralPose();
    shiftToLeftSupport();

    repeatGait = true;
  }

  if (cmd == 'x') {
    Serial.println("Repeat gait stopped.");
    repeatGait = false;
    neutralPose();
  }

  if (cmd == 'u') {
    repeatGait = false;
    unloadKneeServos();
  }
}

void readSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    while (Serial.available() > 0) {
      Serial.read();
    }

    handleCommand(cmd);
  }
}

// ==========================
// Arduino setup / loop
// ==========================
void setup() {
  Serial.begin(9600);
  busSerial.begin(9600);

  delay(2000);

  Serial.println("Wobbi continuous gait controller ready.");
  Serial.println("Commands:");
  Serial.println("n = neutral pose");
  Serial.println("w = weight shift test");
  Serial.println("g = one complete gait cycle");
  Serial.println("r = repeat gait loop");
  Serial.println("x = stop repeat and return neutral");
  Serial.println("u = unload knee servos, only when robot is supported");

  neutralPose();
}

void loop() {
  readSerialCommand();

  if (repeatGait) {
    gaitCycleFromLeftSupport();
    readSerialCommand();
  }
}