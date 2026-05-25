#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

unsigned long strafe_start_ms   = 0;
const unsigned long MIN_STRAFE_MS = 800;

// define the control pin of each motor
const byte left_front = 46;
const byte left_rear  = 47;
const byte right_rear = 50;
const byte right_front = 51;

bool sonar_fwd_triggered = false;

// ----------------------------------------------------------------
//  FAN
// ----------------------------------------------------------------
const int FAN_PIN = 45;

// ----------------------------------------------------------------
//  SONAR
// ----------------------------------------------------------------
const int TRIG_PIN  = 48;
const int ECHO_PIN  = 49;
const unsigned int MAX_DIST = 23200;  // ~400 cm timeout

// three machine states
enum STATE {
    INITIALISING,
    RUNNING,
    STOPPED
};

// define motion states for FSM
enum MOTION {
    MOVE,
    STOP,
    FAN_ON,
    FAN_OFF,
    FINISH
};

struct DriveCommand {
    float x;         // strafe: +1 = right, -1 = left
    float y;         // forward: +1 = forward
    float rotation;  // spin: +1 = CW, -1 = CCW
};

// shared move_input — written by whichever behaviour wins arbitration
DriveCommand move_input;

// per-behaviour MOTION commands and flags
MOTION detect_fire_command;
MOTION cruise_command;
MOTION avoid_obstacle_command;
MOTION extinguish_fire_command;
MOTION realign_to_fire_command;
MOTION motor_input;

int detect_fire_output_flag    = 0;
int cruise_output_flag         = 0;
int avoid_obstacle_output_flag = 0;
int extinguish_fire_output_flag = 0;
int realign_to_fire_output_flag = 0;

// ----------------------------------------------------------------
//  FIRE DETECTION VARIABLES AND CONSTANTS
// ----------------------------------------------------------------
float headingOffset  = 0.0f;
float currentHeading = 0.0f;
float headingError   = 0.0f;
float velX           = 0.0f;
float distX          = 0.0f;
unsigned long lastTimeMicros = 0;
int   fires_extinguished = 0;
int   scan_360       = 0;   // 0 = scanning, 1 = scan done, -1 = aligned
float spin_angle     = 0;
float max_spin_angle = 0;
int   max_sensor_value = 0;
int   min_sensor_value = 1023;

// ----------------------------------------------------------------
//  REALIGN VARIABLES AND CONSTANTS
// ----------------------------------------------------------------
int rotate   = 0;
int last_dir = -1;
#define RIGHT 1
#define LEFT  0

int ir_detect;
int ultrasonic_distance;
int bumper_left;
int bumper_right;
int bumper_back;

// ----------------------------------------------------------------
//  PHOTO TRANSISTORS
// ----------------------------------------------------------------
int sensorValues[4];
int Photopins[] = {A8, A9, A10, A11};

// create servo objects for each motor
Servo lf_motor;
Servo lr_motor;
Servo rr_motor;
Servo rf_motor;

int speed_val = 120;
const int baseSpeed = 150;
int speed_change;

// ----------------------------------------------------------------
//  AVOID OBSTACLE VARIABLES AND CONSTANTS
// ----------------------------------------------------------------
float avoid_strafe_dir   = 0.0f;
bool  avoid_aligned      = false;
bool  currently_strafing = false;
float last_strafe_dir    = 0.0f;

const float IR_FRONT_DANGER_CM  = 7.0f;
const float IR_FRONT_WARNING_CM = 15.0f;
const float IR_REAR_DANGER_CM   = 15.0f;
const float SONAR_FRONT_OBSTACLE = 7.0f;
const float ROBOT_CLEARANCE      = 25.0f;
const float SIDE_DANGER          = 10.0f;

// ----------------------------------------------------------------
//  IR SENSOR PINS
// ----------------------------------------------------------------
const int IR_FRONT_LEFT  = A6;
const int IR_FRONT_RIGHT = A7;
const int IR_BACK_LEFT   = A4;
const int IR_BACK_RIGHT  = A5;

// ----------------------------------------------------------------
//  SONAR SERVO
// ----------------------------------------------------------------
Servo sensor_servo;
const int SERVO_PIN       = 10;
const int SERVO_LEFT      = 165;
const int SERVO_CENTRE    = 90;
const int SERVO_RIGHT     = 15;
const int SWEEP_SETTLE_MS = 250;

// Calibrated distances (cm)
float front_left_IR  = 0;
float front_right_IR = 0;
float rear_left_IR   = 0;
float rear_right_IR  = 0;
float sonar          = 999;

// ----------------------------------------------------------------
//  FIRE / HEADING
// ----------------------------------------------------------------
float FIRE_BEARING_DEG = 0.0f;

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

float heading_locked = 0.0f;
int   fire_count     = 0;

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
void  robotMove();
void  serial_read_conditions();
float GYRO_reading();

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

PID straightPID = { 2.0f, 0.01f, 0.2f, 0, 0 };
PID turnPID     = { 5.0f, 0.01f, 0.3f, 0, 0 };

// ----------------------------------------------------------------
//  DRIVE SPEEDS
// ----------------------------------------------------------------
const float DRIVE_SPEED      = 0.8f;
const float STRAFE_SPEED     = 0.7f;
const float TURN_SPEED       = 0.5f;
const float REALIGN_DEADBAND = 3.0f;

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);

    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    sensor_servo.attach(SERVO_PIN);
    sensor_servo.write(SERVO_CENTRE);
    delay(500);

    if (!bno08x.begin_I2C()) {
        Serial.println(F("ERROR: BNO08X not found."));
        while (1) delay(10);
    }
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

    for (int i = 0; i < 4; i++) {
        pinMode(Photopins[i], INPUT);
    }

    motor_input = STOP;   // safe default before first arbitrate
    last_imu_us = micros();
}

// ================================================================
//  LOOP
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
//  STATE FUNCTIONS
// ================================================================
STATE initialising() {
    enable_motors();
    Serial.println("INITIALISING");

    delay(500);
    for (int i = 0; i < 50; i++) { updateIMU(); delay(10); }
    yaw_offset = yaw_raw;

    return RUNNING;
}

STATE running() {
    serial_read_conditions();
    cruise();
    avoid_obstacle();
   // realign_to_fire();
   // extinguish_fire();
 //   detect_fire();

    arbitrate();
    return RUNNING;
}

STATE stopped() {
    motor_input = FINISH;
    detect_fire_output_flag     = 0;
    cruise_output_flag          = 0;
    avoid_obstacle_output_flag  = 0;
    extinguish_fire_output_flag = 0;
    realign_to_fire_output_flag = 0;
    robotMove();
    return STOPPED;
}

// ================================================================
//  CRUISE
// ================================================================
void cruise() {
    float correction = 0;
    cruise_command     = MOVE;
    move_input         = {0.0f, DRIVE_SPEED, correction};
    cruise_output_flag = 1;
}

// ================================================================
//  REALIGN TO FIRE
// ================================================================
void realign_to_fire() {
    if (fires_extinguished >= 2) {
        // hand off to stopped state via the main FSM
        return;
    }

    if (sensorValues[3] > 80 && sensorValues[2] > 80) {
        int dif    = sensorValues[0] - sensorValues[1];
        int buffer = 0;

        if (sensorValues[1] > 30 && sensorValues[0] > 30) {
            buffer = (sensorValues[1] < 300 || sensorValues[0] < 300) ? 10 : 50;

            if (abs(dif) <= buffer) {
                realign_to_fire_output_flag = 0;
                Serial.print(sensorValues[0]);
                Serial.print(",");
                Serial.println(sensorValues[1]);
            } else if (dif > buffer) {
                move_input = {0.0f, 0.0f, -0.5f};  // CW
                realign_to_fire_command     = MOVE;
                realign_to_fire_output_flag = 1;
                rotate  = -1;
                last_dir = RIGHT;
            } else {
                move_input = {0.0f, 0.0f, 0.5f};   // CCW
                realign_to_fire_command     = MOVE;
                realign_to_fire_output_flag = 1;
                rotate  = 1;
                last_dir = LEFT;
            }
        }
    } else {
        // no light — spin in the direction we last saw fire
        if (last_dir == LEFT) {
            rotate = -1;
        } else if (last_dir == RIGHT) {
            rotate = 1;
        }
        move_input = {0.0f, 0.0f, (1.0f * rotate)};
        realign_to_fire_command     = MOVE;
        realign_to_fire_output_flag = 1;
    }
}

// ================================================================
//  AVOID OBSTACLE
//  Sonar always forward. All 3 front sensors must clear before
//  disengaging. Direction locked on first trigger.
//
//  When both front IRs are blocked, rotate to equalise them
//  (robot squares up to the wall) before locking a strafe direction.
//  This ensures the strafe runs parallel to the wall rather than
//  angling into it.
// ================================================================

// How close the two IR readings need to be (cm) to count as square
const float WALL_ALIGN_TOLERANCE_CM = 2.0f;
bool wall_aligning = false;   // true while squaring up to wall

void avoid_obstacle() {
    bool fl_blocked    = (front_left_IR  < IR_FRONT_WARNING_CM);
    bool fr_blocked    = (front_right_IR < IR_FRONT_WARNING_CM);
    bool sonar_blocked = (sonar          < SONAR_FRONT_OBSTACLE);
    bool rl_blocked    = (rear_left_IR   < ROBOT_CLEARANCE);
    bool rr_blocked    = (rear_right_IR  < ROBOT_CLEARANCE);

    // All three front clear → disengage
    if (!fl_blocked && !fr_blocked && !sonar_blocked) {
        avoid_obstacle_output_flag = 0;
        avoid_strafe_dir           = 0.0f;
        avoid_aligned              = false;
        currently_strafing         = false;
        last_strafe_dir            = 0.0f;
        wall_aligning              = false;
        sonar_fwd_triggered        = false;
        Serial.println(F("[AVOID] All clear"));
        return;
    }

    avoid_obstacle_output_flag = 1;
    avoid_obstacle_command     = MOVE;

    // ── WALL ALIGN PHASE ────────────────────────────────────────
    // Enter alignment if both IRs are blocked and no strafe is locked yet.
    // Stay in it until the two readings are within tolerance.
    if (fl_blocked && fr_blocked && last_strafe_dir == 0.0f) {
        wall_aligning = true;
    }

    if (wall_aligning) {
        float diff = front_left_IR - front_right_IR;

        if (fabs(diff) <= WALL_ALIGN_TOLERANCE_CM) {
            // Readings are equal — robot is square to the wall
            wall_aligning = false;
            Serial.println(F("[AVOID] Wall aligned"));
            // Fall through immediately to lock strafe direction below
        } else {
            // Rotate toward whichever side is further away so both
            // readings converge:
            //   FL > FR  →  robot's left corner is further, rotate CW (-)
            //   FL < FR  →  robot's right corner is further, rotate CCW (+)
            float rot = (diff > 0) ? -0.4f : 0.4f;
            move_input = {0.0f, 0.0f, rot};
            Serial.print(F("[AVOID] Aligning to wall, diff="));
            Serial.println(diff);
            return;
        }
    }

    // ── STRAFE PHASE ─────────────────────────────────────────────
    // Lock direction on first entry (after alignment if both were blocked,
    // or immediately if only one IR was blocked to begin with).
    if (last_strafe_dir == 0.0f) {
        if (fl_blocked && !fr_blocked) {
            // Obstacle on left → strafe right unless rear-right is blocked
            last_strafe_dir = rr_blocked ? -1.0f : 1.0f;
        } else if (!fl_blocked && fr_blocked) {
            // Obstacle on right → strafe left unless rear-left is blocked
            last_strafe_dir = rl_blocked ? 1.0f : -1.0f;
        } else {
            // Both blocked but now square — pick whichever rear is clear
            last_strafe_dir = rl_blocked ? 1.0f : -1.0f;
        }
        last_dir = (last_strafe_dir > 0) ? RIGHT : LEFT;
        Serial.print(F("[AVOID] Direction locked: "));
        Serial.println(last_strafe_dir > 0 ? F("RIGHT") : F("LEFT"));
    }

    // Only flip if the committed rear closes off AND the other is clear
    if (last_strafe_dir > 0 && rr_blocked && !rl_blocked) {
        last_strafe_dir = -1.0f;
        last_dir        = LEFT;
        Serial.println(F("[AVOID] RR closed — flipping to LEFT"));
    } else if (last_strafe_dir < 0 && rl_blocked && !rr_blocked) {
        last_strafe_dir = 1.0f;
        last_dir        = RIGHT;
        Serial.println(F("[AVOID] RL closed — flipping to RIGHT"));
    }

    move_input         = {last_strafe_dir, 0.0f, 0.0f};
    currently_strafing = true;
    Serial.print(F("[AVOID] Strafing: "));
    Serial.println(last_strafe_dir > 0 ? F("RIGHT") : F("LEFT"));
}

// ================================================================
//  DETECT FIRE
//  scan_360 = 0  → spinning 360, tracking peak sensor reading
//  scan_360 = 1  → scan done, rotate back toward max_spin_angle
//  scan_360 = -1 → aligned, stop detect_fire
// ================================================================
void detect_fire() {
    if (scan_360 == 0) {
        if (spin_angle >= 350.0 && spin_angle < 358.0) {
            // Full rotation complete
            scan_360 = 1;
            detect_fire_output_flag = 1;
        } else if (spin_angle < 345.0) {
            if (sensorValues[3] > 80 && sensorValues[2] > 80) {
                int dif = sensorValues[0] - sensorValues[1];
                int buffer = (sensorValues[1] < 300 || sensorValues[0] < 300) ? 10 : 50;

                if (sensorValues[1] > 30 && sensorValues[0] > 30 && abs(dif) <= buffer) {
                    if (sensorValues[0] > max_sensor_value) {
                        max_sensor_value = sensorValues[0];
                        max_spin_angle   = spin_angle;
                    }
                    if (sensorValues[0] < min_sensor_value) {
                        min_sensor_value = sensorValues[0];
                    }
                }
            }
            move_input = {0.0f, 0.0f, 0.8f};  // antiCW spin
            detect_fire_command     = MOVE;
            detect_fire_output_flag = 1;
        }

    } else if (scan_360 == 1) {
        // Rotate toward the angle where sensor peak was recorded
        if (sensorValues[3] > 80 && sensorValues[2] > 80) {
            int dif    = sensorValues[0] - sensorValues[1];
            int buffer = (sensorValues[1] < 300 || sensorValues[0] < 300) ? 10 : 50;

            detect_fire_output_flag = 1;

            if (sensorValues[1] > min_sensor_value && sensorValues[0] > min_sensor_value
                && abs(dif) <= buffer) {
                // Centred on fire — hand off to realign/extinguish
                scan_360 = -1;
                detect_fire_output_flag = 0;
            }
        } else {
            // Not seeing fire yet — spin toward where peak was
            float dir = (max_spin_angle < 180.0f) ? 0.8f : -0.8f;
            move_input = {0.0f, 0.0f, dir};
            detect_fire_command     = MOVE;
            detect_fire_output_flag = 1;
        }

    } else {
        // scan_360 == -1: aligned, detect_fire yields
        detect_fire_output_flag = 0;
    }
}

// ================================================================
//  EXTINGUISH FIRE
// ================================================================
void extinguish_fire() {
    if (realign_to_fire_output_flag == 0 && sonar > 5 && sensorValues[1] > 700) {
        // Aligned and fire visible but not close enough — drive forward
        extinguish_fire_output_flag = 1;
        extinguish_fire_command     = MOVE;
        move_input = {0.0f, 0.5f, 0.0f};
    } else if (realign_to_fire_output_flag == 0 && sonar <= 5 && sensorValues[1] > 700) {
        // Close enough — turn fan on
        extinguish_fire_output_flag = 1;
        extinguish_fire_command     = FAN_ON;
    } else if (motor_input == FAN_ON && realign_to_fire_output_flag == 1) {
        // Was fanning but lost alignment — fan off, count extinguished
        extinguish_fire_output_flag = 1;
        extinguish_fire_command     = FAN_OFF;
        fires_extinguished++;
    } else {
        extinguish_fire_output_flag = 0;
    }
}

// ================================================================
//  ARBITRATE
// ================================================================
void arbitrate() {
    if (cruise_output_flag == 1){
        motor_input = cruise_command;}
    // if (realign_to_fire_output_flag == 1)
    //     motor_input = realign_to_fire_command;
    if (avoid_obstacle_output_flag == 1){
        motor_input = avoid_obstacle_command;}
    // if (extinguish_fire_output_flag == 1)
    //     motor_input = extinguish_fire_command;
    // if (detect_fire_output_flag == 1)
    //     motor_input = detect_fire_command;

    // if (fires_extinguished >= 2) {
    //     motor_input = FINISH;
    // }

    robotMove();
}

// ================================================================
//  SERIAL READ CONDITIONS
// ================================================================
void serial_read_conditions() {
    updateIMU();
    read_IR_sensors();

    for (int i = 0; i < 4; i++) {
        sensorValues[i] = analogRead(Photopins[i]);
    }

    spin_angle = GYRO_reading();
    sonar      = read_sonarsensor();
}

// ================================================================
//  ROBOT MOVE
// ================================================================
void robotMove() {
    switch (motor_input) {
        case MOVE:
            mecanumDrive(move_input.x, move_input.y, move_input.rotation);
            break;
        case STOP:
            stopRobot();
            break;
        case FAN_ON:
            stopRobot();
            digitalWrite(FAN_PIN, HIGH);
            break;
        case FAN_OFF:
            digitalWrite(FAN_PIN, LOW);
            break;
        case FINISH:
            stopRobot();
            disable_motors();
            Serial.println("TASK COMPLETE");
            while (1) delay(10);
            break;
    }
}

// ================================================================
//  SPIN TO HEADING  (blocking)
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

    front_left_IR  = 17948.0f * pow(signal1, -1.22f);
    front_left_IR  = (front_left_IR  + 4.4859f)  / 1.0697f;

    front_right_IR = 17948.0f * pow(signal2, -1.22f);
    front_right_IR = (front_right_IR + 5.6134f)  / 1.14117f;

    rear_left_IR   = 17948.0f * pow(signal3, -1.22f);
    rear_left_IR   = (rear_left_IR   + 7.7957f)  / 2.5496f;

    rear_right_IR  = 17948.0f * pow(signal4, -1.22f);
    rear_right_IR  = (rear_right_IR  + 10.8049f) / 2.9308f;
}

// ================================================================
//  READ SONAR
// ================================================================
float read_sonarsensor() {
    unsigned long t1, t2, pulse_width;

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    t1 = micros();
    while (digitalRead(ECHO_PIN) == 0) {
        if (micros() - t1 > (MAX_DIST + 1000)) {
            Serial.println("HC-SR04: NOT found");
            return 999;
        }
    }

    t1 = micros();
    while (digitalRead(ECHO_PIN) == 1) {
        if (micros() - t1 > (MAX_DIST + 1000)) {
            Serial.println("HC-SR04: Out of range");
            return 999;
        }
    }

    pulse_width = micros() - t1;
    if (pulse_width > MAX_DIST) {
        Serial.println("HC-SR04: Out of range");
        return 999;
    }
    return pulse_width / 58.0f;
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
    (void)dt;
}

// ================================================================
//  GYRO READING
// ================================================================
void tare_heading() {
    headingOffset  = currentHeading;
    velX           = 0.0f;
    distX          = 0.0f;
    lastTimeMicros = micros();
}

float GYRO_reading() {
    if (bno08x.wasReset()) {
        bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
        lastTimeMicros = micros();
    }

    while (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float qw = sensorValue.un.gameRotationVector.real;
            float qx = sensorValue.un.gameRotationVector.i;
            float qy = sensorValue.un.gameRotationVector.j;
            float qz = sensorValue.un.gameRotationVector.k;

            float yaw = atan2(2.0f * (qw * qz + qx * qy),
                              1.0f - 2.0f * (qy * qy + qz * qz));

            currentHeading = yaw * 180.0f / M_PI;
            while (currentHeading < 0.0f)    currentHeading += 360.0f;
            while (currentHeading >= 360.0f) currentHeading -= 360.0f;

            headingError = currentHeading - headingOffset;
            while (headingError < 0.0f)    headingError += 360.0f;
            while (headingError >= 360.0f) headingError -= 360.0f;

            return headingError;
        }
    }
    return headingError;
}

// ================================================================
//  MECANUM DRIVE
//  x = strafe right (+1), y = forward (+1), rotation = CW (+1)
//  Servo always points forward.
// ================================================================
void mecanumDrive(float x, float y, float rotation) {
    sensor_servo.write(SERVO_CENTRE);

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
