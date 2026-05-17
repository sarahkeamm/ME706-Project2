// ============================================================
//  avoidtest.ino
//  Obstacle avoidance test — behavioural architecture + IMU
//
//  Behaviour priority (lowest → highest):
//    1. cruise()   — drive forward with IMU heading hold
//    2. avoid()    — strafe around front obstacles, then realign
//    3. escape()   — nudge forward if rear IR triggered
//
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

// ----------------------------------------------------------------
//  MOTOR PINS
// ----------------------------------------------------------------
const byte left_front  = 46;
const byte left_rear   = 47;
const byte right_rear  = 50;
const byte right_front = 51;

Servo lf_motor, lr_motor, rr_motor, rf_motor;
const int baseSpeed = 150;   // µs offset into mecanumDrive

// ----------------------------------------------------------------
//  SONAR
// ----------------------------------------------------------------
const int TRIG_PIN  = 48;
const int ECHO_PIN  = 49;
const unsigned int MAX_DIST = 23200;  // ~400 cm timeout

// ----------------------------------------------------------------
//  SONAR SERVO
// ----------------------------------------------------------------
Servo sensor_servo;
const int SERVO_PIN        = 10;
const int SERVO_LEFT       = 165;  // sweep left  (slightly inward)
const int SERVO_CENTRE     = 90;
const int SERVO_RIGHT      = 15;   // sweep right (slightly inward)
const int SWEEP_SETTLE_MS  = 250;  // ms to settle before reading

// ----------------------------------------------------------------
//  IR SENSOR PINS  (kept identical to base code)
// ----------------------------------------------------------------
const int IR_FRONT_LEFT  = A6;
const int IR_FRONT_RIGHT = A4;
const int IR_BACK_LEFT   = A7;
const int IR_BACK_RIGHT  = A5;

// Calibrated distances (cm) — filled by read_IR_sensors()
float front_left_IR  = 0;
float front_right_IR = 0;
float rear_left_IR   = 0;
float rear_right_IR  = 0;
float sonar_fwd      = 999;

// ----------------------------------------------------------------
//  DISTANCE THRESHOLDS  — tune to your arena
// ----------------------------------------------------------------
const float IR_FRONT_DANGER_CM  = 20.0f;  // front obstacle: strafe
const float IR_REAR_DANGER_CM   = 15.0f;  // rear obstacle: nudge fwd
const float SONAR_OBSTACLE_CM   = 25.0f;  // sonar: blocked
const float SONAR_CLEAR_CM      = 40.0f;  // sonar: clear to drive
const float SONAR_SIDE_CLEAR_CM = 30.0f;  // side sweep: clear to strafe into

// ----------------------------------------------------------------
//  PHOTO TRANSISTORS
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  HARDCODED FIRE TARGET  — change for each test run
//  0 = straight ahead, positive = left (CCW), negative = right (CW)
// ----------------------------------------------------------------
const float FIRE_BEARING_DEG = 0.0f; 

// ----------------------------------------------------------------
//  IMU
// ----------------------------------------------------------------
#define BNO08X_RESET -1
Adafruit_BNO08x   bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float yaw_raw     = 0.0f;
float yaw_offset  = 0.0f;
unsigned long last_imu_us = 0;
bool  motors_active = false;

// heading_locked: the world-frame heading we want to maintain while driving.
// Set once after initial alignment, updated after each realign.
float heading_locked = 0.0f;

// ----------------------------------------------------------------
//  PID
// ----------------------------------------------------------------
struct PID {
  float kp, ki, kd, integral, prev_error;

  float compute(float target, float current) {
    float error = wrapAngle(target - current);
    integral    = constrain(integral + error, -200.0f, 200.0f);
    float out   = kp * error + ki * integral + kd * (error - prev_error);
    prev_error  = error;
    return out;
  }
  void reset() { integral = 0; prev_error = 0; }
};

// Tune these values from your IMU test results
PID straightPID = { 2.0f, 0.01f, 0.2f, 0, 0 };
PID turnPID     = { 5.0f, 0.01f, 0.3f, 0, 0 };

// ----------------------------------------------------------------
//  DRIVE SPEEDS  (0.0 – 1.0, scaled by baseSpeed)
// ----------------------------------------------------------------
const float DRIVE_SPEED  = 0.8f;
const float STRAFE_SPEED = 0.7f;
const float TURN_SPEED   = 0.5f;
const float REALIGN_DEADBAND = 3.0f;   // degrees

// ----------------------------------------------------------------
//  STATE MACHINE  (unchanged from base code)
// ----------------------------------------------------------------
enum STATE { INITIALISING, RUNNING, STOPPED };

// ----------------------------------------------------------------
//  BEHAVIOUR FLAGS  (unchanged from base code pattern)
// ----------------------------------------------------------------
int cruise_output_flag  = 0;
int avoid_output_flag   = 0;
int escape_output_flag  = 0;

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
float wrapAngle(float a);
float getHeading();
void  updateIMU();
void  mecanumDrive(float x, float y, float rotation);
void  stopRobot();
void  enable_motors();
void  disable_motors();
float read_sonarsensor();
void  read_IR_sensors();
void  spin_to_heading(float target_deg);
void  realign();

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // Sonar
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Sonar servo — centre
  sensor_servo.attach(SERVO_PIN);
  sensor_servo.write(SERVO_CENTRE);
  delay(500);

  // IMU
  if (!bno08x.begin_I2C()) {
    Serial.println(F("ERROR: BNO08X not found."));
    while (1) delay(10);
  }
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

  // Settle then tare heading
  delay(500);
  for (int i = 0; i < 50; i++) { updateIMU(); delay(10); }
  yaw_offset = yaw_raw;
  Serial.print(F("Heading tared. Offset=")); Serial.println(yaw_offset, 2);

  last_imu_us = micros();
}

// ================================================================
//  LOOP  — identical structure to base code
// ================================================================
void loop() {
  static STATE machine_state = INITIALISING;
  switch (machine_state) {
    case INITIALISING: machine_state = initialising(); break;
    case RUNNING:      machine_state = running();      break;
    case STOPPED:      machine_state = stopped();      break;
  }
}

// ================================================================
//  STATE: INITIALISING
// ================================================================
STATE initialising() {
  enable_motors();
  Serial.println(F("INITIALISING"));

  // Rotate once to face fire bearing using IMU PID
  spin_to_heading(FIRE_BEARING_DEG);
  heading_locked = getHeading();
  straightPID.reset();

  Serial.print(F("Aligned. heading_locked=")); Serial.println(heading_locked, 1);
  return RUNNING;
}

// ================================================================
//  STATE: RUNNING
// ================================================================
STATE running() {
  updateIMU();
  read_IR_sensors();
  sensor_servo.write(SERVO_CENTRE);
  sonar_fwd = read_sonarsensor();

  // Debug print every 200ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print(F("Hdg:")); Serial.print(getHeading(), 1);
    Serial.print(F(" FL:")); Serial.print(front_left_IR, 1);
    Serial.print(F(" FR:")); Serial.print(front_right_IR, 1);
    Serial.print(F(" BL:")); Serial.print(rear_left_IR, 1);
    Serial.print(F(" BR:")); Serial.print(rear_right_IR, 1);
    Serial.print(F(" Sonar:")); Serial.println(sonar_fwd, 1);
  }

  // Stop if fire reached
  int pl = analogRead(PHOTO_LEFT_PIN);
  int pr = analogRead(PHOTO_RIGHT_PIN);
  if (pl > PHOTO_FIRE_CLOSE && pr > PHOTO_FIRE_CLOSE) {
    Serial.println(F("Fire reached — STOPPED"));
    return STOPPED;
  }

  // Run behaviours, highest priority last
  cruise();
  escape();  // sets escape_output_flag if rear triggered
  avoid();   // sets avoid_output_flag if front triggered;
             // also blocks/strafes/realigns directly if needed

  arbitrate();
  return RUNNING;
}

// ================================================================
//  STATE: STOPPED
// ================================================================
STATE stopped() {
  stopRobot();
  disable_motors();
  while (true) delay(1000);
  return STOPPED;
}

// ================================================================
//  BEHAVIOUR: CRUISE
//  Default — drive forward holding heading_locked via PID.
//  Lowest priority; always sets its flag.
// ================================================================
void cruise() {
  cruise_output_flag = 1;
}

// ================================================================
//  BEHAVIOUR: AVOID
//  Checks front IR and sonar. If blocked:
//    - Picks strafe direction (single IR side, or sonar sweep)
//    - Strafes with heading hold until front is clear
//    - Calls realign() to spin back to heading_locked
//  Sets avoid_output_flag so arbitrate() skips cruise that cycle.
// ================================================================
void avoid() {
  bool front_left_blocked  = (front_left_IR  < IR_FRONT_DANGER_CM);
  bool front_right_blocked = (front_right_IR < IR_FRONT_DANGER_CM);
  bool sonar_blocked       = (sonar_fwd      < SONAR_OBSTACLE_CM);

  if (!front_left_blocked && !front_right_blocked && !sonar_blocked) {
    avoid_output_flag = 0;
    return;
  }

  // Signal arbitrate to suppress cruise this cycle
  avoid_output_flag = 1;

  Serial.println(F("[AVOID] Obstacle detected"));

  // ── Pick strafe direction ────────────────────────────────────
  float strafe_x;  // mecanumDrive: +1.0 = right, -1.0 = left

  if (front_left_blocked && !front_right_blocked && !sonar_blocked) {
    // Only left IR triggered → strafe right
    strafe_x = 1.0f;
    Serial.println(F("  Left IR only → strafe RIGHT"));

  } else if (front_right_blocked && !front_left_blocked && !sonar_blocked) {
    // Only right IR triggered → strafe left
    strafe_x = -1.0f;
    Serial.println(F("  Right IR only → strafe LEFT"));

  } else {
    // Both IR or sonar blocked ----------------- sweep sonar to find clearer side ---------- may neeed to change so doesn't go back the way it came 
    sensor_servo.write(SERVO_LEFT);  delay(SWEEP_SETTLE_MS);
    float dist_left = read_sonarsensor();

    sensor_servo.write(SERVO_RIGHT); delay(SWEEP_SETTLE_MS);
    float dist_right = read_sonarsensor();

    sensor_servo.write(SERVO_CENTRE); delay(100);

    Serial.print(F("  Sonar L=")); Serial.print(dist_left, 1);
    Serial.print(F("cm R="));      Serial.println(dist_right, 1);

    if (dist_left == dist_right) {
      // Equal — use fire bearing to break the tie
      strafe_x = (FIRE_BEARING_DEG >= 0.0f) ? -1.0f : 1.0f;
      Serial.println(F("  Equal — using fire bearing"));
    } else {
      strafe_x = (dist_left > dist_right) ? -1.0f : 1.0f;
      // we may need to add in if previous strafe so doesn't go back the way it came 
      Serial.println(dist_left > dist_right ? F("  Left clearer → LEFT") : F("  Right clearer → RIGHT"));
    }
  }

  // ── Strafe until front sensors clear ────────────────────────
  float strafe_heading = getHeading();  // hold this heading while strafing
  straightPID.reset();

  while (true) {
    updateIMU();

    read_IR_sensors();
    sensor_servo.write(SERVO_CENTRE);
    float dist = read_sonarsensor();

    bool fl_clear = (front_left_IR  >= IR_FRONT_DANGER_CM);
    bool fr_clear = (front_right_IR >= IR_FRONT_DANGER_CM);
    bool sonar_clear = (dist >= SONAR_CLEAR_CM);

    if (fl_clear && fr_clear && sonar_clear) {
      Serial.println(F("  Front clear — stopping strafe"));
      stopRobot();
      break;
    }

    // Rear IR during strafe — pause briefly if triggered
    // (forward is still blocked so we can't drive forward;
    //  just pause to let the situation settle) ---------------------- I feel like we might need to add in some edge case like if it's wedged 
    bool rl = (rear_left_IR  < IR_REAR_DANGER_CM);
    bool rr = (rear_right_IR < IR_REAR_DANGER_CM);
    if (rl || rr) {
      Serial.println(F("  Rear IR during strafe — pausing"));
      stopRobot();
      delay(300);
      continue;
    }

    // Strafe with PID heading hold
    float correction = constrain(straightPID.compute(strafe_heading, getHeading()) * 0.01f, -0.3f, 0.3f);
    mecanumDrive(strafe_x, 0.0f, correction);
    delay(10);
  }

  // ── Realign to fire heading ──────────────────────────────────
  realign();
}

// ================================================================
//  BEHAVIOUR: ESCAPE
//  Rear IR triggered — nudge forward to clear, if front allows.
//  Higher priority than cruise, lower than avoid.
// ================================================================
void escape() {
  bool rl = (rear_left_IR  < IR_REAR_DANGER_CM);
  bool rr = (rear_right_IR < IR_REAR_DANGER_CM);

  if (!rl && !rr) {
    escape_output_flag = 0;
    return;
  }

  escape_output_flag = 1;
  Serial.print(F("[ESCAPE] Rear: L=")); Serial.print(rear_left_IR, 1);
  Serial.print(F(" R="));               Serial.println(rear_right_IR, 1);

  // Only nudge forward if front is clear
  bool front_clear = (front_left_IR  >= IR_FRONT_DANGER_CM) &&
                     (front_right_IR >= IR_FRONT_DANGER_CM) &&
                     (sonar_fwd      >= SONAR_OBSTACLE_CM);
  if (front_clear) {
    mecanumDrive(0.0f, DRIVE_SPEED * 0.5f, 0.0f);
    delay(200);
    stopRobot();
  } else {
    //wedged 
  }
  // If front also blocked, avoid() will handle it next cycle
}

// ================================================================
//  ARBITRATE
//  Lowest-priority flag applied first; higher ones overwrite.
//  avoid() and escape() act directly above, so here we just
//  execute cruise if nothing higher fired.
// ================================================================
void arbitrate() {
  if (avoid_output_flag || escape_output_flag) {
    // avoid() and escape() already drove the motors directly —
    // nothing more to do this cycle
    avoid_output_flag  = 0;
    escape_output_flag = 0;
    cruise_output_flag = 0;
    return;
  }

  if (cruise_output_flag) {
    float correction = constrain(straightPID.compute(heading_locked, getHeading()) * 0.01f, -0.3f, 0.3f);
    mecanumDrive(0.0f, DRIVE_SPEED, correction);
    cruise_output_flag = 0;
  }
}

// ================================================================
//  REALIGN
//  Spins back to heading_locked using IMU PID.
//  Fine-trims with phototransistors if fire is visible.
//  Re-locks heading_locked from actual IMU heading when done.
// ================================================================
void realign() {
  Serial.println(F("[REALIGN] Spinning back to fire heading"));
  turnPID.reset();

  // Coarse IMU spin
  while (true) {
    updateIMU();
    float error = wrapAngle(heading_locked - getHeading());
    if (fabs(error) <= REALIGN_DEADBAND) break;
    float correction = constrain(turnPID.compute(heading_locked, getHeading()) * 0.04f, -TURN_SPEED, TURN_SPEED);
    mecanumDrive(0.0f, 0.0f, correction);
    delay(10);
  }
  stopRobot();
  delay(100);

  // // Fine trim with phototransistors if fire is visible
  // for (int i = 0; i < 30; i++) {
  //   int pl    = analogRead(PHOTO_LEFT_PIN);
  //   int pr    = analogRead(PHOTO_RIGHT_PIN);
  //   if ((pl + pr) < PHOTO_FIRE_VISIBLE) break;  // not visible — trust IMU
  //   int delta = pr - pl;
  //   if (abs(delta) <= PHOTO_DEAD_ZONE) break;   // balanced — done
  //   float spin = (delta > 0) ? -TURN_SPEED * 0.5f : TURN_SPEED * 0.5f;
  //   mecanumDrive(0.0f, 0.0f, spin);
  //   delay(60);
  //   stopRobot();
  //   delay(40);
  // }

  // Re-lock heading from actual IMU reading
  heading_locked = getHeading();
  straightPID.reset();
  Serial.print(F("[REALIGN] Done. heading_locked=")); Serial.println(heading_locked, 1);
}

// ================================================================
//  SPIN TO HEADING  (blocking — called once in initialising())
// ================================================================
void spin_to_heading(float target_deg) {
  turnPID.reset();
  while (true) {
    updateIMU();
    float error = wrapAngle(target_deg - getHeading());
    if (fabs(error) <= 2.0f) break;
    float correction = constrain(turnPID.compute(target_deg, getHeading()) * 0.04f, -0.5f, 0.5f);
    mecanumDrive(0.0f, 0.0f, correction);
    delay(10);
  }
  stopRobot();
  delay(150);
}

// ================================================================
//  READ IR SENSORS 
// ================================================================
void read_IR_sensors() {
  long sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
  for (int i = 0; i < 4; i++) {
    sum1 += analogRead(IR_FRONT_LEFT);
    sum2 += analogRead(IR_FRONT_RIGHT);
    sum3 += analogRead(IR_BACK_LEFT);
    sum4 += analogRead(IR_BACK_RIGHT);
    delay(5);
  }
  float signal1 = sum1 / 4.0f;
  float signal2 = sum2 / 4.0f;
  float signal3 = sum3 / 4.0f;
  float signal4 = sum4 / 4.0f;

  // Front sensors — long range calibration
  float IR1 = 17948.0f * pow(signal1, -1.22f);
  IR1 = (IR1 - 0.1596f) / 0.8007f;
  front_left_IR = IR1;

  float IR2 = 17948.0f * pow(signal2, -1.22f);
  IR2 = (IR2 + 2.0700f) / 0.9163f;
  front_right_IR = IR2;

  // Rear sensors — medium range calibration
  float IR3 = 17948.0f * pow(signal3, -1.22f);
  IR3 = (IR3 + 10.6425f) / 2.9208f;
  rear_left_IR = IR3;

  float IR4 = 17948.0f * pow(signal4, -1.22f);
  IR4 = (IR4 + 10.8049f) / 2.9308f;
  rear_right_IR = IR4;
}

// ================================================================
//  READ SONAR  
// ================================================================
float read_sonarsensor() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

   // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
  return cm; 
}

void HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    // SerialCom->println("HC-SR04: Out of range");
  } else {
    // SerialCom->print("HC-SR04:");
    // SerialCom->print(cm);
    // SerialCom->println("cm");
  }
}

// ================================================================
//  IMU UPDATE
// ================================================================
void updateIMU() {
  unsigned long now_us = micros();
  float dt = constrain((now_us - last_imu_us) / 1e6f, 0.0f, 0.05f);
  last_imu_us = now_us;

  while (bno08x.getSensorEvent(&sensorValue)) {
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
  }
  (void)dt;  // dt reserved for position integration if needed later
}

// ================================================================
//  MECANUM DRIVE  (unchanged from base code)
//  x = strafe right (+1), y = forward (+1), rotation = CW (+1)
// ================================================================
void mecanumDrive(float x, float y, float rotation) {
  motors_active = true;
  rotation = -rotation;
  float lf =  y + x + rotation;
  float lr =  y - x + rotation;
  float rf =  y - x - rotation;
  float rr =  y + x - rotation;
  float mx = max(max(fabs(lf), fabs(lr)), max(fabs(rf), fabs(rr)));
  if (mx > 1.0f) { lf /= mx; lr /= mx; rf /= mx; rr /= mx; }
  lf_motor.writeMicroseconds(1500 + (int)(lf * baseSpeed));
  lr_motor.writeMicroseconds(1500 + (int)(lr * baseSpeed));
  rf_motor.writeMicroseconds(1500 - (int)(rf * baseSpeed));
  rr_motor.writeMicroseconds(1500 - (int)(rr * baseSpeed));
}

// ================================================================
//  HELPERS
// ================================================================
float getHeading() { return wrapAngle(yaw_raw - yaw_offset); }

float wrapAngle(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void stopRobot() {
  motors_active = false;
  lf_motor.writeMicroseconds(1500);
  lr_motor.writeMicroseconds(1500);
  rf_motor.writeMicroseconds(1500);
  rr_motor.writeMicroseconds(1500);
}

void enable_motors() {
  lf_motor.attach(left_front);
  lr_motor.attach(left_rear);
  rr_motor.attach(right_rear);
  rf_motor.attach(right_front);
}

void disable_motors() {
  lf_motor.detach(); 
  lr_motor.detach();
  rr_motor.detach();
  rf_motor.detach();
}
