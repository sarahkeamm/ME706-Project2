// =============================================================
//  BNO08X IMU TEST CODE v4  –  Arduino Mega
//
//  Changes from v3:
//    - Position only integrates when motors_active = true
//    - Velocity zeroed immediately on stopRobot()
//    - Threshold lowered to 0.15 (stationary drift no longer matters)
//
//  TEST MODES (change TEST_MODE define below):
//    0 = Raw IMU print          – heading + position, no motors
//    1 = Position tracking      – push robot by hand, watch serial
//    2 = Drive straight (PID)   – forward 2 s with heading hold
//    3 = Turn to angle (PID)    – type angle in Serial Monitor
// =============================================================

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

// ── Test selector ────────────────────────────────────────────
#define TEST_MODE 3

// ── Motor pins (matches base project) ────────────────────────
const byte PIN_LF = 46;
const byte PIN_LR = 47;
const byte PIN_RR = 50;
const byte PIN_RF = 51;

Servo lf_motor, lr_motor, rr_motor, rf_motor;

// ── Base drive speed (µs offset, 0–400 sensible range) ───────
int baseSpeed = 150;

// ── BNO08X ───────────────────────────────────────────────────
#define BNO08X_RESET -1
Adafruit_BNO08x   bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ── Heading state ─────────────────────────────────────────────
float yaw_raw    = 0.0f;
float yaw_offset = 0.0f;

// ── Position – world frame (metres from 0,0) ─────────────────
float vel_x = 0, vel_y = 0;
float pos_x = 0, pos_y = 0;
unsigned long last_imu_us = 0;

// ── Motor active flag – gates position integration ───────────
bool motors_active = false;

// Forward declarations
float wrapAngle(float a);
float getHeading();
void  updateIMU();
void  mecanumDrive(float x, float y, float rotation);
void  stopRobot();
void  resetPosition();

// ── PID ───────────────────────────────────────────────────────
struct PID {
  float kp, ki, kd;
  float integral;
  float prev_error;

  float compute(float target, float current) {
    float error  = wrapAngle(target - current);
    integral    += error;
    integral     = constrain(integral, -200.0f, 200.0f);
    float deriv  = error - prev_error;
    prev_error   = error;
    return kp * error + ki * integral + kd * deriv;
  }

  void reset() { integral = 0; prev_error = 0; }
};

PID straightPID = { 2.0f, 0.01f, 0.2f, 0, 0 };
PID turnPID     = { 5.0f, 0.01f, 0.3f, 0, 0 };

// ── Drive-straight state ──────────────────────────────────────
#define DRIVE_DURATION_MS 2000
float         straight_target  = 0;
bool          straight_running = false;
bool          straight_done    = false;
unsigned long straight_start   = 0;

// ── Turn-to-angle state ───────────────────────────────────────
float turn_target     = 0;
bool  turn_target_set = false;
bool  turn_done       = true;
#define TURN_DEADBAND 2.0f

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== BNO08X IMU Test v4 ==="));
  Serial.print(F("TEST_MODE = ")); Serial.println(TEST_MODE);

  lf_motor.attach(PIN_LF);
  lr_motor.attach(PIN_LR);
  rr_motor.attach(PIN_RR);
  rf_motor.attach(PIN_RF);
  stopRobot();

  if (!bno08x.begin_I2C()) {
    Serial.println(F("ERROR: BNO08X not found. Check wiring."));
    while (1) delay(10);
  }
  Serial.println(F("BNO08X found."));

  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);

  // Let fusion settle then tare heading
  delay(500);
  for (int i = 0; i < 50; i++) { updateIMU(); delay(10); }
  yaw_offset = yaw_raw;
  Serial.print(F("Heading tared. Offset = ")); Serial.println(yaw_offset, 2);

  resetPosition();
  last_imu_us = micros();

  if (TEST_MODE == 3)
    Serial.println(F("Type a target angle (degrees) and press Enter."));

  Serial.println(F("--- START ---"));
}

// =============================================================
//  LOOP
// =============================================================
void loop() {
  updateIMU();

  switch (TEST_MODE) {
    case 0: testRawPrint();      break;
    case 1: testPositionTrack(); break;
    case 2: testDriveStraight(); break;
    case 3: testTurnToAngle();   break;
    case 4: testDriveBackward(); break;
    case 5: testStrafeLeft();    break;
    case 6: testStrafeRight();   break;
  }

  delay(10);
}

// =============================================================
//  TEST 0 – Raw IMU print, no motors
// =============================================================
void testRawPrint() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();

  Serial.print(F("Heading: ")); Serial.print(getHeading(), 1);
  Serial.print(F("°  X: "));   Serial.print(pos_x, 3);
  Serial.print(F(" m  Y: "));  Serial.print(pos_y, 3);
  Serial.println(F(" m"));
}

// =============================================================
//  TEST 1 – Position tracking (push robot by hand)
//  Note: motors_active is false so position won't update here.
//  This mode is now mainly useful for heading verification.
//  To test position, use TEST_MODE 2.
// =============================================================
void testPositionTrack() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 250) return;
  lastPrint = millis();

  Serial.print(F("X: "));       Serial.print(pos_x, 3);
  Serial.print(F(" m  Y: "));   Serial.print(pos_y, 3);
  Serial.print(F(" m  Hdg: ")); Serial.print(getHeading(), 1);
  Serial.println(F("°"));
}

// =============================================================
//  TEST 2 – Drive straight for DRIVE_DURATION_MS with PID
// =============================================================
void testDriveStraight() {
  if (straight_done) return;

  if (!straight_running) {
    straight_target  = getHeading();
    straight_start   = millis();
    straight_running = true;
    straightPID.reset();
    resetPosition();
    Serial.print(F("Driving straight. Target = "));
    Serial.println(straight_target, 1);
  }

  if (millis() - straight_start < DRIVE_DURATION_MS) {
    float correction = straightPID.compute(straight_target, getHeading());
    correction *= 0.01f;
    correction  = constrain(correction, -0.5f, 0.5f);

    mecanumDrive(0.0f, 1.0f, correction);  // motors_active set true inside

    Serial.print(F("Hdg: "));     Serial.print(getHeading(), 1);
    Serial.print(F("°  Corr: ")); Serial.print(correction, 4);
    Serial.print(F("  X: "));     Serial.print(pos_x, 3);
    Serial.print(F(" m  Y: "));   Serial.print(pos_y, 3);
    Serial.println(F(" m"));

  } else {
    stopRobot();  // motors_active set false inside, velocity zeroed
    straight_done = true;
    Serial.println(F("Drive complete."));
    Serial.print(F("Final  X: ")); Serial.print(pos_x, 3);
    Serial.print(F(" m  Y: "));    Serial.println(pos_y, 3);
  }
}

// =============================================================
//  TEST 3 – Turn to angle typed in Serial Monitor (PID)
// =============================================================
void testTurnToAngle() {
  if (Serial.available() > 0) {
    turn_target     = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    turn_target_set = true;
    turn_done       = false;
    turnPID.reset();
    resetPosition();
    Serial.print(F("Turning to: ")); Serial.print(turn_target, 1);
    Serial.println(F("°"));
  }

  if (!turn_target_set || turn_done) return;

  float currentHeading = getHeading();
  float error          = wrapAngle(turn_target - currentHeading);

  if (fabs(error) <= TURN_DEADBAND) {
    stopRobot();  // motors_active set false, velocity zeroed
    turn_done = true;
    Serial.println(F("Turn complete."));
    Serial.print(F("Final heading: ")); Serial.println(currentHeading, 1);
    Serial.println(F("Enter next target angle:"));
    return;
  }

  float correction = turnPID.compute(turn_target, currentHeading);
  correction *= 0.04f;
  correction  = constrain(correction, -0.5f, 0.5f);

  mecanumDrive(0.0f, 0.0f, correction);  // motors_active set true inside

  Serial.print(F("Hdg: "));     Serial.print(currentHeading, 1);
  Serial.print(F("°  Err: "));  Serial.print(error, 1);
  Serial.print(F("°  Corr: ")); Serial.println(correction, 4);
}

// =============================================================
//  IMU UPDATE
// =============================================================
void updateIMU() {
  unsigned long now_us = micros();
  float dt = (now_us - last_imu_us) / 1e6f;
  last_imu_us = now_us;

  if (dt > 0.05f) dt = 0.05f;  // clamp runaway dt

  while (bno08x.getSensorEvent(&sensorValue)) {

    // ── Game rotation vector → yaw (no magnetometer) ─────
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      float yaw_rad = atan2(2.0f * (qr * qk + qi * qj),
                            1.0f - 2.0f * (qj * qj + qk * qk));
      yaw_raw = degrees(yaw_rad);
      if (yaw_raw < 0) yaw_raw += 360.0f;
    }

    // ── Linear acceleration → world frame position ────────
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      float ax_sensor = sensorValue.un.linearAcceleration.x;
      float ay_sensor = sensorValue.un.linearAcceleration.y;

      const float THRESHOLD = 0.15f;
      if (fabs(ax_sensor) < THRESHOLD) ax_sensor = 0;
      if (fabs(ay_sensor) < THRESHOLD) ay_sensor = 0;

      // Rotate from robot frame into world frame
      float heading_rad = getHeading() * PI / 180.0f;
      float ax_world = ax_sensor * cos(heading_rad) - ay_sensor * sin(heading_rad);
      float ay_world = ax_sensor * sin(heading_rad) + ay_sensor * cos(heading_rad);

      if (motors_active) {
        // Only integrate while motors are running
        vel_x += ax_world * dt;
        vel_y += ay_world * dt;
        pos_x += vel_x    * dt;
        pos_y += vel_y    * dt;
      } else {
        // Robot stopped – zero velocity so nothing carries over
        vel_x = 0;
        vel_y = 0;
      }
    }
  }
}

// =============================================================
//  MECANUM INVERSE KINEMATICS
//  x = strafe right (+1), y = forward (+1), rotation = CW (+1)
// =============================================================
void mecanumDrive(float x, float y, float rotation) {
  motors_active = true;   // gates position integration in updateIMU
  rotation = -rotation;

  float lf =  y + x + rotation;
  float lr =  y - x + rotation;
  float rf =  y - x - rotation;
  float rr =  y + x - rotation;

  float maxVal = max(max(fabs(lf), fabs(lr)), max(fabs(rf), fabs(rr)));
  if (maxVal > 1.0f) { lf /= maxVal; lr /= maxVal; rf /= maxVal; rr /= maxVal; }

  lf_motor.writeMicroseconds(1500 + (int)(lf * baseSpeed));
  lr_motor.writeMicroseconds(1500 + (int)(lr * baseSpeed));
  rf_motor.writeMicroseconds(1500 - (int)(rf * baseSpeed));
  rr_motor.writeMicroseconds(1500 - (int)(rr * baseSpeed));
}

// =============================================================
//  HELPERS
// =============================================================
float getHeading() {
  return wrapAngle(yaw_raw - yaw_offset);
}

float wrapAngle(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void resetPosition() {
  pos_x = pos_y = vel_x = vel_y = 0;
}

void stopRobot() {
  motors_active = false;  // stop position integration immediately
  vel_x = 0;              // zero velocity so nothing carries over
  vel_y = 0;

  lf_motor.writeMicroseconds(1500);
  lr_motor.writeMicroseconds(1500);
  rr_motor.writeMicroseconds(1500);
  rf_motor.writeMicroseconds(1500);
}

void testDriveBackward() {
  static bool done    = false;
  static bool running = false;
  static unsigned long start = 0;

  if (done) return;
  if (!running) {
    running = true;
    start   = millis();
    resetPosition();
    Serial.println(F("Driving backward..."));
  }

  if (millis() - start < DRIVE_DURATION_MS) {
    mecanumDrive(0.0f, -1.0f, 0.0f);   // y = -1 = backward
    Serial.print(F("Hdg: "));  Serial.print(getHeading(), 1);
    Serial.print(F("  X: "));  Serial.print(pos_x, 3);
    Serial.print(F("  Y: "));  Serial.println(pos_y, 3);
  } else {
    stopRobot();
    done = true;
    Serial.println(F("Backward complete."));
    Serial.print(F("Final  X: ")); Serial.print(pos_x, 3);
    Serial.print(F("  Y: "));      Serial.println(pos_y, 3);
  }
}

void testStrafeLeft() {
  static bool done    = false;
  static bool running = false;
  static unsigned long start = 0;
  static float locked_heading = 0;   // ADD - captures heading at start

  if (done) return;
  if (!running) {
    running        = true;
    start          = millis();
    locked_heading = getHeading();   // ADD - lock heading now
    straightPID.reset();
    resetPosition();
    Serial.println(F("Strafing left..."));
  }

  if (millis() - start < DRIVE_DURATION_MS) {
    float correction = straightPID.compute(locked_heading, getHeading());
    correction *= 0.01f;
    correction = constrain(correction, -0.3f, 0.3f);

    mecanumDrive(-1.0f, 0.0f, correction);  // strafe left + heading hold

    Serial.print(F("Hdg: "));  Serial.print(getHeading(), 1);
    Serial.print(F("  X: "));  Serial.print(pos_x, 3);
    Serial.print(F("  Y: "));  Serial.println(pos_y, 3);
  } else {
    stopRobot();
    done = true;
    Serial.println(F("Strafe left complete."));
    Serial.print(F("Final  X: ")); Serial.print(pos_x, 3);
    Serial.print(F("  Y: "));      Serial.println(pos_y, 3);
  }
}

void testStrafeRight() {
  static bool done    = false;
  static bool running = false;
  static unsigned long start = 0;
  static float locked_heading = 0;   // ADD

  if (done) return;
  if (!running) {
    running        = true;
    start          = millis();
    locked_heading = getHeading();   // ADD
    straightPID.reset();
    resetPosition();
    Serial.println(F("Strafing right..."));
  }

  if (millis() - start < DRIVE_DURATION_MS) {
    float correction = straightPID.compute(locked_heading, getHeading());
    correction *= 0.01f;
    correction = constrain(correction, -0.3f, 0.3f);

    mecanumDrive(1.0f, 0.0f, correction);   // strafe right + heading hold

    Serial.print(F("Hdg: "));  Serial.print(getHeading(), 1);
    Serial.print(F("  X: "));  Serial.print(pos_x, 3);
    Serial.print(F("  Y: "));  Serial.println(pos_y, 3);
  } else {
    stopRobot();
    done = true;
    Serial.println(F("Strafe right complete."));
    Serial.print(F("Final  X: ")); Serial.print(pos_x, 3);
    Serial.print(F("  Y: "));      Serial.println(pos_y, 3);
  }
}
