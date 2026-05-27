#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

// ================================================================
//  PIN DEFINITIONS
// ================================================================
const byte left_front  = 46;
const byte left_rear   = 47;
const byte right_rear  = 50;
const byte right_front = 51;

const int FAN_PIN   = 45;
const int TRIG_PIN  = 48;
const int ECHO_PIN  = 49;
const int SERVO_PIN = 10;

// IR sensor pins
const int IR_FRONT_LEFT  = A6;
const int IR_FRONT_RIGHT = A7;
const int IR_BACK_LEFT   = A4;
const int IR_BACK_RIGHT  = A5;

// Phototransistor pins
// A8 = short right, A9 = short left, A10 = long right, A11 = long left
int Photopins[] = {A8, A9, A10, A11};
// Index aliases for clarity
#define PT_SHORT_RIGHT 0
#define PT_SHORT_LEFT  1
#define PT_LONG_RIGHT  2
#define PT_LONG_LEFT   3

// ================================================================
//  PHOTOTRANSISTOR THRESHOLDS
// ================================================================
// Long-range: ambient tops out at ~24. Confirmed fire threshold is 70.
// Long-range sensors stop being useful at ~30 cm (short-range ~500 at that point).
#define LONG_FIRE_THRESHOLD      60    // long-range: fire definitely present
#define LONG_SATURATED_SHORT     500   // short-range value at which long-range is no longer reliable (~30 cm)
#define LONG_ALIGNED_BUFFER_HI   50   // alignment deadband when long sensors >300
#define LONG_ALIGNED_BUFFER_LO   10   // alignment deadband when either long sensor <300

// Short-range: fire visible and alignment should begin at >700.
// The same 700 threshold is used to gate the EXTINGUISH transition from RUNNING.
#define SHORT_FIRE_VISIBLE       30    // short-range: sensor is active (not ambient noise)
#define SHORT_FIRE_ALIGN         950   // short-range: fire ~10 cm away — begin extinguish

// ================================================================
//  STATE ENUMS
// ================================================================
enum STATE  { INITIALISING, DETECT_FIRE, RUNNING, EXTINGUISH, STOPPED };
enum MOTION { MOVE, STOP, FAN_ON, FAN_OFF, FINISH };

// ================================================================
//  DRIVE COMMAND STRUCT
// ================================================================
struct DriveCommand {
    float x;        // strafe:   +1 = right
    float y;        // forward:  +1 = forward
    float rotation; // spin:     +1 = CW
};

// ================================================================
//  BEHAVIOUR OUTPUT SLOTS
// ================================================================
DriveCommand cruise_move_input;
DriveCommand avoid_move_input;
DriveCommand realign_move_input;
DriveCommand move_input;

MOTION cruise_command;
MOTION avoid_obstacle_command;
MOTION realign_to_fire_command;
MOTION motor_input;

int cruise_output_flag          = 0;
int avoid_obstacle_output_flag  = 0;
int realign_to_fire_output_flag = 0;

// ================================================================
//  SENSOR VALUES
// ================================================================
int   sensorValues[4];          // phototransistors [SR, SL, LR, LL]
float front_left_IR   = 999;
float front_right_IR  = 999;
float rear_left_IR    = 999;
float rear_right_IR   = 999;
float sonar           = 999;
float currentHeading  = 0.0f;   // accumulates from GYRO_reading()
float headingOffset   = 0.0f;
float headingError    = 0.0f;

// ================================================================
//  FIRE DETECTION STATE
// ================================================================
int   fires_extinguished = 0;

// During 360 scan we record up to 2 candidate bearings
struct FireCandidate {
    float bearing;        // gyro angle at which candidate was centred
    int   peakLong;       // peak long-sensor value seen while aligned
    bool  valid;
};
FireCandidate fireA = {0, 0, false};
FireCandidate fireB = {0, 0, false};

// Scan helpers
float scan_start_heading = 0.0f;  // heading when scan began
float spin_angle         = 0.0f;  // degrees turned since scan start (0..360+)
bool  scan_complete      = false;

// Angle at which max long-sensor reading was seen (running max during scan)
float peak_angle         = 0.0f;
int   peak_long_value    = 0;
bool  in_candidate_window = false; // true while sensors > threshold
float candidate_start_angle = 0.0f;
float candidate_max_angle   = 0.0f;
int   candidate_max_value   = 0;
int   candidate_count       = 0;   // how many candidates found so far

// Target heading to drive towards (set after scan)
float target_bearing = 0.0f;
bool  target_locked  = false;

// ================================================================
//  REALIGN / DIRECTION
// ================================================================
int rotate   = 1;
int last_dir = -1;
#define RIGHT 1
#define LEFT  0

// ================================================================
//  OBSTACLE AVOIDANCE STATE
// ================================================================
float avoid_strafe_dir   = 0.0f;
bool  avoid_aligned      = false;
bool  currently_strafing = false;
float last_strafe_dir    = 0.0f;
bool  wall_aligning      = false;
bool  sonar_fwd_triggered = false;

const float IR_FRONT_WARNING_CM  = 15.0f;
const float IR_FRONT_DANGER_CM   = 15.0f;
const float IR_REAR_DANGER_CM    = 15.0f;
const float SONAR_FRONT_OBSTACLE = 15.0f;
const float ROBOT_CLEARANCE      = 25.0f;
const float SIDE_DANGER          = 10.0f;
const float SONAR_SIDE_OBSTACLE_CM = 25.0f;

// ================================================================
//  SERVO
// ================================================================
const int SERVO_LEFT   = 175;
const int SERVO_CENTRE = 90;
const int SERVO_RIGHT  = 5;
int servo_pos = SERVO_CENTRE;

// ================================================================
//  IMU
// ================================================================
#define BNO08X_RESET -1
Adafruit_BNO08x   bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
float yaw_raw    = 0.0f;
float yaw_offset = 0.0f;
unsigned long last_imu_us = 0;

// ================================================================
//  MOTORS / SERVOS
// ================================================================
Servo lf_motor, lr_motor, rr_motor, rf_motor;
Servo sensor_servo;
bool  motors_active = false;
int baseSpeed = 200;

// ================================================================
//  PID
// ================================================================
struct PID {
    float kp, ki, kd, integral, prev_error;
    static float wrapAngle(float a) {
        while (a >  180.0f) a -= 360.0f;
        while (a < -180.0f) a += 360.0f;
        return a;
    }
    float compute(float target, float current) {
        float error = wrapAngle(target - current);
        integral  = constrain(integral + error, -200.0f, 200.0f);
        float out = kp * error + ki * integral + kd * (error - prev_error);
        prev_error = error;
        return out;
    }
    void reset() { integral = 0; prev_error = 0; }
};
PID turnPID = { 5.0f, 0.01f, 0.3f, 0, 0 };

// ================================================================
//  DRIVE SPEEDS
// ================================================================
const float DRIVE_SPEED  = 0.8f;
const float TURN_SPEED   = 0.75f;

// ================================================================
//  SERIAL
// ================================================================
HardwareSerial *SerialCom;
const unsigned int MAX_DIST = 23200;

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
float wrapAngle(float a);
float getHeading();
void  updateIMU();
float GYRO_reading();
void  mecanumDrive(float x, float y, float rotation);
void  stopRobot();
void  enable_motors();
void  disable_motors();
float read_sonarsensor();
void  read_IR_sensors();
void  serial_read_conditions();
void  robotMove();
void  setServo(int pos);
void  avoid_obstacle();
void  cruise();
void  realign_to_fire();
void  arbitrate();
STATE detect_fire();
STATE running();
STATE extinguish();
STATE initialising();
STATE stopped();

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial1.begin(115200);
    SerialCom = &Serial1;

    pinMode(FAN_PIN,  OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    sensor_servo.attach(SERVO_PIN);
    sensor_servo.write(SERVO_CENTRE);
    delay(500);

    if (!bno08x.begin_I2C()) {
        SerialCom->println(F("ERROR: BNO08X not found."));
        while (1) delay(10);
    }
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

    for (int i = 0; i < 4; i++) pinMode(Photopins[i], INPUT);

    last_imu_us = micros();
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
    static STATE machine_state = INITIALISING;
    switch (machine_state) {
        case INITIALISING: machine_state = initialising(); break;
        case DETECT_FIRE:  machine_state = detect_fire();  break;
        case RUNNING:      machine_state = running();      break;
        case EXTINGUISH:   machine_state = extinguish();   break;
        case STOPPED:      machine_state = stopped();      break;
    }
}

// ================================================================
//  INITIALISING
// ================================================================
STATE initialising() {
    enable_motors();
    SerialCom->println(F("INITIALISING"));

    // Warm up IMU
    for (int i = 0; i < 50; i++) { updateIMU(); delay(10); }
    yaw_offset = yaw_raw;

    // Reset scan state
    scan_complete      = false;
    spin_angle         = 0.0f;
    candidate_count    = 0;
    in_candidate_window = false;
    peak_long_value    = 0;
    fireA.valid        = false;
    fireB.valid        = false;
    scan_start_heading = GYRO_reading();

    return DETECT_FIRE;
}

// ================================================================
//  DETECT FIRE  — 360° spin, find up to 2 fire candidates
// ================================================================
STATE detect_fire() {
    serial_read_conditions();

    // If we've found both fires and locked onto the nearer one,
    // go straight to RUNNING.
    if (target_locked) {
        SerialCom->print(F("DETECT_FIRE: heading to bearing "));
        SerialCom->println(target_bearing);
        return RUNNING;
    }

    // ── SCANNING ──────────────────────────────────────────────
    if (!scan_complete) {
        // Spin anti-clockwise
        move_input = {0.0f, 0.0f, 0.6f};
        motor_input = MOVE;
        robotMove();

        // spin_angle tracks total degrees turned (for 360° completion check).
        // current_gyro is the absolute heading for bearing storage.
        float current_gyro = GYRO_reading();

        // Long-range fire detection — both sensors must agree
        bool fire_visible = (sensorValues[PT_LONG_RIGHT] > LONG_FIRE_THRESHOLD &&
                             sensorValues[PT_LONG_LEFT]  > LONG_FIRE_THRESHOLD);

        if (fire_visible) {
            int longAvg = (sensorValues[PT_LONG_RIGHT] + sensorValues[PT_LONG_LEFT]) / 2;

            if (!in_candidate_window) {
                // Entering a new fire window
                in_candidate_window  = true;
                candidate_start_angle = spin_angle;
                candidate_max_angle   = current_gyro;  // store absolute heading
                candidate_max_value   = longAvg;
            } else {
                // Track peak within this window
                if (longAvg > candidate_max_value) {
                    candidate_max_value = longAvg;
                    candidate_max_angle = current_gyro;  // store absolute heading
                }
            }
        } else {
            if (in_candidate_window) {
                // Exiting a fire window — record this candidate
                in_candidate_window = false;
                SerialCom->print(F("Candidate found at gyro heading: "));
                SerialCom->println(candidate_max_angle);

                if (candidate_count == 0) {
                    fireA = { candidate_max_angle, candidate_max_value, true };
                    candidate_count = 1;
                } else if (candidate_count == 1) {
                    fireB = { candidate_max_angle, candidate_max_value, true };
                    candidate_count = 2;
                }
            }
        }

        // Check if we've done a full 360°
        if (spin_angle >= 355.0f) {
            scan_complete = true;
            stopRobot();
            delay(200);
            SerialCom->println(F("Scan complete"));

            // Handle lingering window at wrap-around
            if (in_candidate_window) {
                in_candidate_window = false;
                if (candidate_count == 0) {
                    fireA = { candidate_max_angle, candidate_max_value, true };
                    candidate_count = 1;
                } else if (candidate_count == 1) {
                    fireB = { candidate_max_angle, candidate_max_value, true };
                    candidate_count = 2;
                }
            }
        }

        return DETECT_FIRE;
    }

    // ── AFTER SCAN: choose target ──────────────────────────────
    if (!fireA.valid && !fireB.valid) {
        // Nothing found — do another scan
        SerialCom->println(F("No fire found, rescanning"));
        scan_complete   = false;
        spin_angle      = 0.0f;
        candidate_count = 0;
        scan_start_heading = GYRO_reading();
        return DETECT_FIRE;
    }

    // Pick the candidate with the higher long-sensor peak —
    // a stronger reading means the robot is closer or more aligned
    // (skip whichever fire index we have already extinguished).
    FireCandidate chosen;
    if (fires_extinguished == 0) {
        // First fire: pick the candidate with the stronger signal
        if (fireA.valid && fireB.valid) {
            chosen = (fireA.peakLong >= fireB.peakLong) ? fireA : fireB;
        } else if (fireA.valid) {
            chosen = fireA;
        } else {
            chosen = fireB;
        }
    } else {
        // Second fire: use whichever candidate is still valid
        // (we'll re-scan if neither is left, but normally one remains)
        if (fireA.valid && fireB.valid) {
            // Prefer the one we haven't approached — use the weaker signal
            // as a heuristic (stronger one was likely fire #1)
            chosen = (fireA.peakLong <= fireB.peakLong) ? fireA : fireB;
        } else if (fireA.valid) {
            chosen = fireA;
        } else if (fireB.valid) {
            chosen = fireB;
        } else {
            // No candidates left — rescan
            SerialCom->println(F("No second fire candidate, rescanning"));
            scan_complete   = false;
            spin_angle      = 0.0f;
            candidate_count = 0;
            fireA.valid     = false;
            fireB.valid     = false;
            scan_start_heading = GYRO_reading();
            return DETECT_FIRE;
        }
    }

    target_bearing = chosen.bearing;
    target_locked  = true;
    SerialCom->print(F("Target bearing: "));
    SerialCom->println(target_bearing);

    // Spin to face the target bearing
    spin_to_fire_bearing(target_bearing);

    // Reset RUNNING-state flags
    cruise_output_flag          = 0;
    avoid_obstacle_output_flag  = 0;
    realign_to_fire_output_flag = 0;
    last_strafe_dir             = 0.0f;
    avoid_strafe_dir            = 0.0f;
    currently_strafing          = false;
    last_dir                    = -1;

    return RUNNING;
}

// Blocking spin to an absolute gyro heading recorded during the scan.
// bearingDeg and GYRO_reading() are both 0–360 absolute headings,
// so shortest-path normalisation to ±180 gives the correct error direction.
void spin_to_fire_bearing(float bearingDeg) {
    SerialCom->print(F("Spinning to bearing: "));
    SerialCom->println(bearingDeg);
    turnPID.reset();

    unsigned long startMs = millis();
    float current, error;
    do {
        if (millis() - startMs > 5000) break; // safety timeout
        updateIMU();
        current = GYRO_reading();
        // Both values are 0–360; compute the shortest angular distance
        error = bearingDeg - current;
        // Normalise to ±180 within the 0–360 space
        if (error >  180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;
        float correction = constrain(error * 0.03f, -0.5f, 0.5f);
        mecanumDrive(0.0f, 0.0f, correction);
        delay(10);
    } while (fabs(error) > 3.0f);
    stopRobot();
    delay(150);
}

// ================================================================
//  RUNNING  — drive toward fire, avoid obstacles, realign
// ================================================================
STATE running() {
    serial_read_conditions();

    // ── Check short-range sensors AND sonar before entering EXTINGUISH.
    // Both short-range sensors must read above threshold AND sonar must
    // confirm the robot is within 10 cm, preventing false triggers.
    bool sr_left_ready  = sensorValues[PT_SHORT_LEFT]  > SHORT_FIRE_ALIGN;
    bool sr_right_ready = sensorValues[PT_SHORT_RIGHT] > SHORT_FIRE_ALIGN;
    bool sonar_close    = (sonar <= 17.0f);

    if (sr_left_ready && sr_right_ready && sonar_close) {
        stopRobot();
        SerialCom->println(F("Fire close -- switching to EXTINGUISH"));
        return EXTINGUISH;
    }

    // ── Run behaviours ──
    cruise();

    realign_to_fire();
    avoid_obstacle();
    arbitrate();

    return RUNNING;
}

// ================================================================
//  EXTINGUISH
// ================================================================
STATE extinguish() {
    serial_read_conditions();
    static bool          fan_running  = false;
    static unsigned long fan_start_ms = 0;
    sonar = read_sonarsensor();

    const unsigned long FAN_MIN_MS = 5000UL;   // always fan for at least 7 s
    const unsigned long FAN_MAX_MS = 12000UL;  // never fan for more than 12 s

    // Fire still present when both short-range sensors remain above align threshold
    bool sr_close = (sensorValues[PT_SHORT_LEFT]  > SHORT_FIRE_ALIGN &&
                     sensorValues[PT_SHORT_RIGHT] > SHORT_FIRE_ALIGN);
    
    while (sonar > 5) {
      move_input = {0.0f, 0.5f, 0.0f};
      motor_input = MOVE;
      robotMove(); 
      delay(50);
      sonar = read_sonarsensor();
    }
    stopRobot();
    delay(300);

    if (!fan_running) {
        stopRobot();
        digitalWrite(FAN_PIN, HIGH);
        fan_running  = true;
        fan_start_ms = millis();
        SerialCom->println(F("Fan ON"));
        return EXTINGUISH;
    }

    unsigned long fan_elapsed = millis() - fan_start_ms;

    // Keep fanning until minimum time has elapsed
    if (fan_elapsed < FAN_MIN_MS) {
        return EXTINGUISH;
    }

    // Between min and max: keep fanning only if sensors still see fire
    if (fan_elapsed < FAN_MAX_MS && sr_close) {
        return EXTINGUISH;
    }

    // Either sensors clear after min, or max time reached — stop fan
    digitalWrite(FAN_PIN, LOW);
    fan_running = false;
    fires_extinguished++;
    SerialCom->print(F("Fires extinguished: "));
    SerialCom->println(fires_extinguished);

    if (fires_extinguished >= 2) {
        return STOPPED;
    }

    // Invalidate whichever candidate matches the bearing we just extinguished
    if (fireA.valid && fabs(wrapAngle(fireA.bearing - target_bearing)) < 15.0f) {
        fireA.valid = false;
    } else if (fireB.valid && fabs(wrapAngle(fireB.bearing - target_bearing)) < 15.0f) {
        fireB.valid = false;
    }

    // Back away slightly before scanning
    move_input = {0.0f, -0.6f, 0.0f};
    motor_input = MOVE;
    for (int i = 0; i < 30; i++) { robotMove(); delay(20); }
    stopRobot();
    delay(300);

    // Reset scan for second fire — keep candidate_count reflecting still-valid entries
    target_locked      = false;
    scan_complete      = false;
    spin_angle         = 0.0f;
    candidate_count    = (fireA.valid ? 1 : 0) + (fireB.valid ? 1 : 0);
    in_candidate_window = false;
    scan_start_heading = GYRO_reading();

    // Clear nav flags
    cruise_output_flag          = 0;
    avoid_obstacle_output_flag  = 0;
    realign_to_fire_output_flag = 0;
    last_strafe_dir             = 0.0f;
    currently_strafing          = false;
    last_dir                    = -1;

    return DETECT_FIRE;
}

// ================================================================
//  STOPPED
// ================================================================
STATE stopped() {
    stopRobot();
    disable_motors();
    SerialCom->println(F("TASK COMPLETE"));
    while (1) delay(10);
    return STOPPED;
}

// ================================================================
//  CRUISE  — move forward (heading correction via realign_to_fire)
// ================================================================
void cruise() {
    cruise_command    = MOVE;
    cruise_move_input = {0.0f, DRIVE_SPEED, 0.0f};
    cruise_output_flag = 1;
}

void realign_to_fire() {
            if ((sensorValues[3] > 70 && sensorValues[2] > 70)) { //if light detected by long range
            int dif = sensorValues[0] - sensorValues[1];
            // SerialCom->println(dif);

            int buffer;
            if (sensorValues[1] > 30 && sensorValues[0] > 30) {
                if (sensorValues[1] < 300 || sensorValues[0] < 300) {
                    buffer = 10;
                } else {
                    buffer = 50;
                }

            if (abs(dif) <= buffer) {
                realign_to_fire_output_flag = 0;
            } else if (dif > buffer) {
                realign_move_input = {0.0f, 0.0f, -0.5f}; //CLOCKWISE
                realign_to_fire_command = MOVE;
                realign_to_fire_output_flag = 1;
                rotate = -1.0;
            } else if (dif < (-1 * buffer)) {
                realign_move_input = {0.0f, 0.0f, 0.5f}; //ANTI
                realign_to_fire_command = MOVE;
                realign_to_fire_output_flag = 1;
                rotate = 1.0;
            } 
            }
            } else if ((sensorValues[2] <= 60 || sensorValues[3] <= 60)) { //else if no light detected
            if (last_dir == LEFT) {
            rotate =  -1;
            last_dir = -1;
            } else if (last_dir == RIGHT) {
            rotate = 1;
            last_dir = -1;
            }
            realign_move_input = {0.0f, 0.0f, (1.0f*rotate)}; 
            realign_to_fire_command = MOVE;
            realign_to_fire_output_flag = 1;
            }
        
}


// ================================================================
//  REALIGN TO FIRE
// ================================================================
/*
void realign_to_fire() {
    int sr_right = sensorValues[PT_SHORT_RIGHT];
    int sr_left  = sensorValues[PT_SHORT_LEFT];
    int lr_right = sensorValues[PT_LONG_RIGHT];
    int lr_left  = sensorValues[PT_LONG_LEFT];

    // Decide which sensor pair to use for alignment.
    // Long-range becomes unreliable once short-range reads ~500 (robot ~30 cm away).
    // Once short-range reads >= SHORT_FIRE_ALIGN (700) use short-range differential.
    bool use_short = (sr_right >= SHORT_FIRE_ALIGN || sr_left >= SHORT_FIRE_ALIGN);
    bool long_visible = (lr_right > LONG_FIRE_THRESHOLD && lr_left > LONG_FIRE_THRESHOLD);
    bool short_active = (sr_right > SHORT_FIRE_VISIBLE  && sr_left > SHORT_FIRE_VISIBLE);

    if (use_short) {
        // ── SHORT-RANGE alignment (close approach / extinguish phase) ──
        if (!short_active) {
            realign_to_fire_output_flag = 0;
            return;
        }
        int dif    = sr_right - sr_left;
        int buffer = 50; // at close range sensors are ~1000 so 50 gives a tight deadband

        if (abs(dif) <= buffer) {
            realign_to_fire_output_flag = 0;  // aligned
        } else if (dif > buffer) {
            // Right stronger → fire to our right → turn CW
            realign_move_input          = {0.0f, 0.0f, -0.35f};
            realign_to_fire_command     = MOVE;
            realign_to_fire_output_flag = 1;
            rotate   = -1;
            last_dir = RIGHT;
        } else {
            // Left stronger → fire to our left → turn CCW
            realign_move_input          = {0.0f, 0.0f, 0.35f};
            realign_to_fire_command     = MOVE;
            realign_to_fire_output_flag = 1;
            rotate   = 1;
            last_dir = LEFT;
        }

    } else if (long_visible) {
        // ── LONG-RANGE alignment (approaching from distance) ──
        // Use short-range differential for fine correction if sensors are active,
        // otherwise trust long-range presence to confirm we're facing the fire.
        int dif = sr_right - sr_left;

        bool both_short_active = (sr_left  > SHORT_FIRE_VISIBLE &&
                                  sr_right > SHORT_FIRE_VISIBLE);
        if (!both_short_active) {
            realign_to_fire_output_flag = 0;
            return;
        }
        bool weak_signal = (sr_left < 300 || sr_right < 300);
        int  buffer      = weak_signal ? LONG_ALIGNED_BUFFER_LO : LONG_ALIGNED_BUFFER_HI;

        if (abs(dif) <= buffer) {
            realign_to_fire_output_flag = 0;
            rotate = (last_dir == LEFT) ? -1 : 1;
        } else if (dif > buffer) {
            realign_move_input          = {0.0f, 0.0f, -0.35f};
            realign_to_fire_command     = MOVE;
            realign_to_fire_output_flag = 1;
            rotate   = -1;
            last_dir = RIGHT;
        } else {
            realign_move_input          = {0.0f, 0.0f, 0.35f};
            realign_to_fire_command     = MOVE;
            realign_to_fire_output_flag = 1;
            rotate   = 1;
            last_dir = LEFT;
        }

    } else {
        // ── No fire visible — rotate slowly in last-known direction ──
        float spin_dir = (last_dir == LEFT)  ?  1.0f :
                         (last_dir == RIGHT) ? -1.0f : 1.0f;
        realign_move_input          = {0.0f, 0.0f, spin_dir * 0.5f};
        realign_to_fire_command     = MOVE;
        realign_to_fire_output_flag = 1;
    }
}
*/

// ================================================================
//  AVOID OBSTACLE  (logic preserved; only move_input assignment clarified)
// ================================================================
void setServo(int pos) {
    if (pos != servo_pos) {
        sensor_servo.write(pos);
        delay(200);
        servo_pos = pos;
    }
}

void avoid_obstacle() {
    bool fl_blocked = (front_left_IR  < IR_FRONT_WARNING_CM);
    bool fr_blocked = (front_right_IR < IR_FRONT_WARNING_CM);
    bool rl_blocked = false;
    bool rr_blocked = false;

    bool sonar_front_blocked = (servo_pos == SERVO_CENTRE) && (sonar < SONAR_FRONT_OBSTACLE);
    bool sonar_side_blocked  = (servo_pos != SERVO_CENTRE) && (sonar < SONAR_SIDE_OBSTACLE_CM);

    if (fl_blocked && !fr_blocked) {
        rl_blocked = rear_left_IR  < ROBOT_CLEARANCE;
        rr_blocked = rear_right_IR < SIDE_DANGER;
    }
    if (!fl_blocked && fr_blocked) {
        rl_blocked = rear_left_IR  < SIDE_DANGER;
        rr_blocked = rear_right_IR < ROBOT_CLEARANCE;
    }
    if ((fl_blocked && fr_blocked) || sonar_front_blocked) {
        rl_blocked = rear_left_IR  < ROBOT_CLEARANCE;
        rr_blocked = rear_right_IR < ROBOT_CLEARANCE;
    }

    if (!fl_blocked && !fr_blocked) {
        setServo(SERVO_CENTRE);
        if (!sonar_front_blocked) {
            avoid_obstacle_output_flag = 0;
            avoid_strafe_dir           = 0.0f;
            avoid_aligned              = false;
            currently_strafing         = false;
            last_strafe_dir            = 0.0f;
            wall_aligning              = false;
            sonar_fwd_triggered        = false;
            // After obstacle clear: re-lock onto fire heading
            target_locked = false;
            return;
        }
    }

    avoid_obstacle_output_flag = 1;
    avoid_obstacle_command     = MOVE;

     if (last_strafe_dir == 0.0f) {
        if (!rr_blocked && rl_blocked) {
            last_strafe_dir = 1.0f;   // right rear clear → strafe right
        } else if (!rl_blocked && rr_blocked) {
            last_strafe_dir = -1.0f;  // left rear clear → strafe left
        } else if (!rr_blocked && !rl_blocked) {
            // Both rear clear — use front sensor that triggered to pick side
            last_strafe_dir = fl_blocked ? 1.0f : -1.0f;
        } else {
            // Both rear blocked — default right
            last_strafe_dir = 1.0f;
        }
        last_dir = (last_strafe_dir > 0) ? RIGHT : LEFT;
        SerialCom->print(F("[AVOID] Direction locked: "));
        SerialCom->println(last_strafe_dir > 0 ? F("RIGHT") : F("LEFT"));
    }

    // ── FLIP only if strafe-side rear sensor closes up ─────────
    // Only flip for a dynamic obstacle appearing on our strafe side,
    // NOT because a front sensor is blocked (that is the wall we are avoiding).
    // if (last_strafe_dir > 0 && rr_blocked && !rl_blocked) {
    //     last_strafe_dir = -1.0f;
    //     last_dir        = LEFT;
    //     SerialCom->println(F("[AVOID] RR closed — flipping LEFT"));
    // } else if (last_strafe_dir < 0 && rl_blocked && !rr_blocked) {
    //     last_strafe_dir = 1.0f;
    //     last_dir        = RIGHT;
    //     SerialCom->println(F("[AVOID] RL closed — flipping RIGHT"));
    // }

    bool strafe_into_danger = (last_strafe_dir > 0 && front_right_IR < 10)
                           || (last_strafe_dir < 0 && front_left_IR  < 10);
    if (strafe_into_danger) {
        avoid_move_input = {0.0f, -0.6f, 0.0f};
    } else {
        avoid_move_input   = {last_strafe_dir, 0.0f, 0.0f};
        currently_strafing = true;
    }  

}
   

// ================================================================
//  ARBITRATE  (priority: extinguish > avoid > realign > cruise)
// ================================================================
void arbitrate() {
    motor_input = STOP;
    move_input  = {0.0f, 0.0f, 0.0f};
    baseSpeed = 200;

    if (cruise_output_flag == 1) {
        motor_input = cruise_command;
        move_input  = cruise_move_input;
    }
    if (realign_to_fire_output_flag == 1) {
        baseSpeed = 150;
        motor_input = realign_to_fire_command;
        move_input  = realign_move_input;
    }
    if (avoid_obstacle_output_flag == 1) {
        motor_input = avoid_obstacle_command;
        move_input  = avoid_move_input;
    }

    robotMove();
}

// ================================================================
//  SERIAL READ CONDITIONS
// ================================================================
void serial_read_conditions() {
    updateIMU();
    read_IR_sensors();

    // Average each phototransistor over 4 samples with a 5 ms gap,
    // matching the IR sensor averaging approach.
    long photoSums[4] = {0, 0, 0, 0};
    for (int s = 0; s < 4; s++) {
        for (int i = 0; i < 4; i++) {
            photoSums[i] += analogRead(Photopins[i]);
        }
        delay(5);
    }
    for (int i = 0; i < 4; i++) {
        sensorValues[i] = (int)(photoSums[i] / 4);
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
            delay(500);
            break;
        case FINISH:
            stopRobot();
            disable_motors();
            SerialCom->println(F("TASK COMPLETE"));
            while (1) delay(10);
            break;
    }
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
    float s1 = sum1 / 4.0f, s2 = sum2 / 4.0f,
          s3 = sum3 / 4.0f, s4 = sum4 / 4.0f;

    front_left_IR  = (17948.0f * pow(s1, -1.22f) + 4.4859f)  / 1.0697f;
    front_right_IR = (17948.0f * pow(s2, -1.22f) + 5.6134f)  / 1.14117f;
    rear_left_IR   = (17948.0f * pow(s3, -1.22f) + 7.7957f)  / 2.5496f;
    rear_right_IR  = (17948.0f * pow(s4, -1.22f) + 10.8049f) / 2.9308f;
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
        if ((micros() - t1) > (MAX_DIST + 1000)) return 999;
    }

    t1 = micros();
    while (digitalRead(ECHO_PIN) == 1) {
        if ((micros() - t1) > (MAX_DIST + 1000)) return 999;
    }

    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > MAX_DIST) return 999;
    return pulse_width / 58.0f;
}

// ================================================================
//  IMU UPDATE
// ================================================================
void updateIMU() {
    unsigned long now_us = micros();
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
}

// ================================================================
//  GYRO READING — returns accumulated angle (0..360+) from reset
// ================================================================
float GYRO_reading() {
    if (bno08x.wasReset()) {
        bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
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
