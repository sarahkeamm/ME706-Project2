#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

unsigned long strafe_start_ms   = 0;
const unsigned long MIN_STRAFE_MS = 800;   // tune this up if needed

// define the control pin of each motor
const byte left_front = 46;
const byte left_rear = 47;
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
DriveCommand detect_move_input;
DriveCommand cruise_move_input;
DriveCommand avoid_move_input;
DriveCommand extinguish_move_input;
DriveCommand realign_move_input;
DriveCommand move_input;


// per-behaviour MOTION commands and flags
MOTION detect_fire_command;
MOTION cruise_command;
MOTION avoid_obstacle_command;
MOTION extinguish_fire_command;
MOTION realign_to_fire_command;
MOTION motor_input;

int detect_fire_output_flag   = 0;
int cruise_output_flag        = 0;
int avoid_obstacle_output_flag = 0;
int extinguish_fire_output_flag = 0;
int realign_to_fire_output_flag = 0;

// ----------------------------------------------------------------
//  FIRE DETECTION VARIABLES AND CONSTANTS
// ----------------------------------------------------------------
float headingOffset = 0.0f;
float currentHeading = 0.0f;
float headingError = 0.0f;
float velX = 0.0f;
float distX = 0.0f;
unsigned long lastTimeMicros = 0;
int fires_extinguished = 0;
int scan_360 = 0;
float spin_angle = 0;
float max_spin_angle = 0;
int max_sensor_value = 0;
int min_sensor_value = 1023;



// ----------------------------------------------------------------
//  REALIGN VARIABLES AND CONSTANTS
// ----------------------------------------------------------------
int rotate = 0;
int last_dir = -1;
#define RIGHT 1
#define LEFT 0

int ir_detect;
int ultrasonic_distance;
int bumper_left;
int bumper_right;
int bumper_back;

// ----------------------------------------------------------------
//  PHOTO TRANSISTORS
// ----------------------------------------------------------------
int sensorValues[4];
int Photopins[] = {A8, A9, A10, A11}; // phototransistor pins

// create servo objects for each motor
Servo lf_motor;
Servo lr_motor;
Servo rr_motor;
Servo rf_motor;

int speed_val = 120;
const int baseSpeed = 150;   // µs offset into mecanumDrive
int speed_change;

// ----------------------------------------------------------------
//  AVOID OBSTACLE VARIABLES AND CONSTANTS
// ----------------------------------------------------------------
float avoid_strafe_dir   = 0.0f;
bool  avoid_aligned      = false;
bool  currently_strafing = false;
float last_strafe_dir    = 0.0f;

const float IR_FRONT_DANGER_CM  = 10.0f;
const float IR_FRONT_WARNING_CM = 15.0f;
const float IR_REAR_DANGER_CM   = 15.0f;
const float SONAR_FRONT_OBSTACLE = 10.0f;
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
const int SERVO_LEFT      = 175;
const int SERVO_CENTRE    = 90;
const int SERVO_RIGHT     = 5;
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

HardwareSerial *SerialCom;

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
    Serial1.begin(115200);
    SerialCom = &Serial1;

    //fan
    pinMode(FAN_PIN, OUTPUT);

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

    //spin_to_heading(FIRE_BEARING_DEG);
    //heading_locked = getHeading();
    //straightPID.reset();

    return RUNNING;
}

STATE running(){
    serial_read_conditions(); //read all sensors
    cruise();
    realign_to_fire();
    avoid_obstacle();
    extinguish_fire();
    detect_fire();

    arbitrate();
    //should set all sensor values to 0 here
    return RUNNING; 
} // STATE REPEATLY

STATE stopped() {
    motor_input = FINISH;
    detect_fire_output_flag = 0;
    cruise_output_flag = 0;
    avoid_obstacle_output_flag = 0;
    extinguish_fire_output_flag = 0;
    realign_to_fire_output_flag = 0;
    robotMove();
}

// ================================================================
//  CRUISE
// ================================================================
void cruise() {
    //float correction = constrain(straightPID.compute(heading_locked, getHeading()) * 0.01f, -0.3f, 0.3f);
    float correction = 0; // --------------------------- need to change once integrated with align_to_fire
    cruise_command = MOVE;
    cruise_move_input  = {0.0f, DRIVE_SPEED, correction};
    cruise_output_flag = 1;
}

void realign_to_fire() {
            if ((sensorValues[3] > 80 && sensorValues[2] > 80)) { //if light detected by long range
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
            } else if ((sensorValues[2] <= 80 || sensorValues[3] <= 80)) { //else if no light detected
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
//  AVOID OBSTACLE
// ================================================================
const float WALL_ALIGN_TOLERANCE_CM = 2.0f;
const float SONAR_SIDE_OBSTACLE_CM  = 15.0f;  // tune as needed
bool wall_aligning = false;
int  servo_pos     = SERVO_CENTRE;   // track current position to avoid redundant writes

// Helper — only move servo if position actually changed
void setServo(int pos) {
    if (pos != servo_pos) {
        sensor_servo.write(pos);
        delay(100);
        servo_pos = pos;
    }
}

void avoid_obstacle() {
    bool fl_blocked    = (front_left_IR  < IR_FRONT_WARNING_CM);
    bool fr_blocked    = (front_right_IR < IR_FRONT_WARNING_CM);
    bool rl_blocked    = false;
    bool rr_blocked    = false;

 
    // Sonar meaning depends on servo position:
    //   centre  → reading is forward
    //   left/right → reading is sideways (strafe-side obstacle check)
    bool sonar_front_blocked = (servo_pos == SERVO_CENTRE) && (sonar < SONAR_FRONT_OBSTACLE);
    bool sonar_side_blocked  = (servo_pos != SERVO_CENTRE) && (sonar < SONAR_SIDE_OBSTACLE_CM);

    if (fl_blocked && !fr_blocked) {
        rl_blocked = rear_left_IR < ROBOT_CLEARANCE;
        rr_blocked = rear_right_IR < SIDE_DANGER; 
    }
    if (!fl_blocked && fr_blocked){
        rl_blocked = rear_left_IR < SIDE_DANGER;
        rr_blocked = rear_right_IR < ROBOT_CLEARANCE; 
    }
    if (fl_blocked && fr_blocked || sonar_front_blocked){
        rl_blocked = rear_left_IR < ROBOT_CLEARANCE;
        rr_blocked = rear_right_IR < ROBOT_CLEARANCE; 
    }

    // ── ALL CLEAR ───────────────────────────────────────────────
    // Both front IRs gone → swing servo back to centre so next loop
    // reads forward, then check that forward reading is also clear.
    if (!fl_blocked && !fr_blocked) {
        setServo(SERVO_CENTRE);   // may be a no-op if already centre
        if (!sonar_front_blocked) {
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
        // IRs clear but sonar still sees something forward — keep avoiding
    }

    avoid_obstacle_output_flag = 1;
    avoid_obstacle_command     = MOVE;


    // ── LOCK STRAFE DIRECTION ────────────────────────────────────
    // Runs on first trigger for: one IR blocked, both IR blocked (post-align),
    // or sonar-only blocked.
    if (last_strafe_dir == 0.0f) {
        if (fl_blocked && !fr_blocked) {
            last_strafe_dir = rr_blocked ? -1.0f : 1.0f;
        } else if (!fl_blocked && fr_blocked) {
            last_strafe_dir = rl_blocked ? 1.0f : -1.0f;
        } else {
            // Both IRs blocked (now square) OR sonar-only → default left
            // unless rear-left is blocked
            last_strafe_dir = rl_blocked ? 1.0f : -1.0f;
        }
        last_dir = (last_strafe_dir > 0) ? RIGHT : LEFT;
        Serial.print(F("[AVOID] Direction locked: "));
        Serial.println(last_strafe_dir > 0 ? F("RIGHT") : F("LEFT"));
    }

    // ── SERVO: point sideways while strafing with IR blocked,
    //           stay centre for sonar-only or post-IR-clear cases ──
    if (currently_strafing && (fl_blocked || fr_blocked)) {
        setServo(last_strafe_dir > 0 ? SERVO_RIGHT : SERVO_LEFT);
    } else {
        setServo(SERVO_CENTRE);
    }

    // ── DIRECTION FLIP ───────────────────────────────────────────
    // Side sonar counts the same as the rear IR on the strafe side.
    if (last_strafe_dir > 0 && (rr_blocked || sonar_side_blocked) && !rl_blocked) {
        last_strafe_dir = -1.0f;
        last_dir        = LEFT;
        Serial.println(F("[AVOID] RR/sonar closed — flipping to LEFT"));
    } else if (last_strafe_dir < 0 && (rl_blocked || sonar_side_blocked) && !rr_blocked) {
        last_strafe_dir = 1.0f;
        last_dir        = RIGHT;
        Serial.println(F("[AVOID] RL/sonar closed — flipping to RIGHT"));
    }

    // ── BACK UP if strafe-side front IR is dangerously close ─────
    bool strafe_into_danger = (last_strafe_dir > 0 && front_right_IR < 10.0f)
                           || (last_strafe_dir < 0 && front_left_IR  < 10.0f);
    if (strafe_into_danger) {
        move_input = {0.0f, -0.6f, 0.0f};
        Serial.println(F("[AVOID] Danger on strafe side — backing up"));
        return;
    }

    move_input         = {last_strafe_dir, 0.0f, 0.0f};
    currently_strafing = true;
    Serial.print(F("[AVOID] Strafing: "));
    Serial.println(last_strafe_dir > 0 ? F("RIGHT") : F("LEFT"));
}


// ================================================================
//  DETECT FIRE
// ================================================================
void detect_fire() {

    if (fires_extinguished == 2) {
        stopped();
        return;
    }

    if (scan_360 == -1) {
        detect_fire_command = STOP;
        detect_fire_output_flag = 0;
    }
    //initial scan for fire
    if (scan_360 == 0) {
        if (spin_angle >= 350.0 && spin_angle < 358.0) {
            scan_360 = 1;
        } else if (spin_angle < 345.0) {
            if (sensorValues[3] > 80 && sensorValues[2] > 80) {
                int dif = sensorValues[0] - sensorValues[1];

                int buffer;
                if (sensorValues[1] > 30 && sensorValues[0] > 30) {
                    if (sensorValues[1] < 300 || sensorValues[0] < 300) {
                        buffer = 10;
                    } else {
                        buffer = 50;
                    }

                    if (abs(dif) <= buffer) {
                        if (max_sensor_value < sensorValues[0]) {
                            max_sensor_value = sensorValues[0];
                            max_spin_angle = spin_angle;
                        }

                        if (min_sensor_value > sensorValues[0]) {
                            min_sensor_value = sensorValues[0];
                        }
                    }
                }
            } 
        }

        detect_fire_output_flag = 1;
        detect_move_input = {0.0f, 0.0f, 0.8f}; // antiCLOCKWISE 
        detect_fire_command = MOVE;

    } else if (scan_360 == 1 && fires_extinguished == 0) {
        detect_fire_output_flag = 1;
        if (sensorValues[3] > 80 && sensorValues[2] > 80) {
                int dif = sensorValues[0] - sensorValues[1];
                int buffer;

                if (sensorValues[1] < 300 || sensorValues[0] < 300) {
                        buffer = 10;
                    } else {
                        buffer = 50;
                }

                if (sensorValues[1] > min_sensor_value && sensorValues[0] > min_sensor_value && abs(dif) <= buffer) {
                    scan_360 = -1;
                    detect_fire_output_flag = 0;
                    // SerialCom->println("detect = 0");
                } else {
                    if (max_spin_angle < 180) {
                        detect_move_input = {0.0f, 0.0f, 0.5f}; // antiCLOCKWISE 
                        detect_fire_command = MOVE;
                    } else {
                        detect_move_input = {0.0f, 0.0f, -0.5f}; // CLOCKWISE
                        detect_fire_command = MOVE;
                }
            }

        } else {
            if (max_spin_angle < 180) {
                detect_move_input = {0.0f, 0.0f, 0.8f}; // antiCLOCKWISE 
                detect_fire_command = MOVE;
            } else {
                detect_move_input = {0.0f, 0.0f, -0.8f}; // CLOCKWISE
                detect_fire_command = MOVE;
            }
        }
    } else if (scan_360 == 1 && fires_extinguished == 1) {
        SerialCom->print(detect_fire_output_flag);
        SerialCom->print(",");
        SerialCom->println(extinguish_fire_output_flag);
        if (sonar <= 15) {
            detect_fire_output_flag = 1;
            detect_move_input = {0.0f, -0.5f, 0.0f};
            detect_fire_command = MOVE;
        } else {
            detect_fire_output_flag = realign_to_fire_output_flag;
            detect_fire_command = realign_to_fire_command;
            detect_move_input = realign_move_input;
            if (detect_fire_output_flag == 0) {
                SerialCom->println("2nd fire found");
                detect_fire_command = STOP;
                detect_move_input = {0.0f, 0.0f, 0.0f};
                scan_360 = -1;
            }
        }
    }
}


// ================================================================
//  EXTINGUISH FIRE
// ================================================================
void extinguish_fire()
{

    if (realign_to_fire_output_flag == 0 && sonar > 5 && sensorValues[1] > 700 ) {
        extinguish_fire_output_flag = 1;
        extinguish_fire_command = MOVE;
        extinguish_move_input = {0.0f, 0.5f, 0.0f};
    } else if (realign_to_fire_output_flag == 0 && sonar <= 5 && sensorValues[1] > 700) {
        extinguish_fire_output_flag = 1;
        extinguish_fire_command = FAN_ON;
    } else if (realign_to_fire_output_flag == 1 && motor_input == FAN_ON) {
        SerialCom->println("FAN OFF");
        extinguish_fire_command = FAN_OFF;
        extinguish_fire_output_flag = 1;
        scan_360 = 1;
        fires_extinguished++;
    } else {
        extinguish_fire_output_flag = 0;
    }
}

// ================================================================
//  ARBITRATE
// ================================================================
void arbitrate() {
    if (cruise_output_flag == 1) { motor_input = cruise_command; move_input = cruise_move_input;}
    if (realign_to_fire_output_flag == 1) { motor_input = realign_to_fire_command; move_input = realign_move_input;}
    if (avoid_obstacle_output_flag == 1) { motor_input = avoid_obstacle_command; move_input = avoid_move_input;}
    if (detect_fire_output_flag == 1) { motor_input = detect_fire_command; move_input = detect_move_input;}
    if (extinguish_fire_output_flag == 1) {motor_input = extinguish_fire_command; move_input = extinguish_move_input;}

    


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

    sonar = read_sonarsensor();
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
            digitalWrite(FAN_PIN, HIGH); // Turn fan ON
            break;
        case FAN_OFF:
            digitalWrite(FAN_PIN, LOW); // Turn fan OFF
            delay(500);
            break;
        case FINISH:
            stopRobot();
            disable_motors();
            SerialCom->println("TASK COMPLETE");
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

    front_left_IR  = 17948 * pow(signal1, -1.22);
    front_left_IR  = (front_left_IR  + 4.4859)  / 1.0697;

    front_right_IR = 17948.0f * pow(signal2, -1.22f);
    front_right_IR = (front_right_IR + 5.6134)  / 1.14117;

    rear_left_IR   = 17948 * pow(signal3, -1.22);
    rear_left_IR   = (rear_left_IR   + 7.7957)  / 2.5496;

    rear_right_IR  = 17948.0f * pow(signal4, -1.22f);
    rear_right_IR  = (rear_right_IR  + 10.8049f) / 2.9308f;
}

// ================================================================
//  READ SONAR
// ================================================================
float read_sonarsensor() {
    unsigned long t1, t2, pulse_width;
    float cm;

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    t1 = micros();
    while (digitalRead(ECHO_PIN) == 0) {
        t2 = micros();
        pulse_width = t2 - t1;
        if (pulse_width > (MAX_DIST + 1000)) {
            Serial.println("HC-SR04: NOT found");
            return 999;
        }
    }

    t1 = micros();
    while (digitalRead(ECHO_PIN) == 1) {
        t2 = micros();
        pulse_width = t2 - t1;
        if (pulse_width > (MAX_DIST + 1000)) {
            Serial.println("HC-SR04: Out of range");
            return 999;
        }
    }

    t2 = micros();
    pulse_width = t2 - t1;
    cm = pulse_width / 58.0;

    if (pulse_width > MAX_DIST) {
        Serial.println("HC-SR04: Out of range");
        return 999;
    }
    return cm;
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
//  GYRO READING TO RETURN ANGLE (USE FOR DETECT FIRE)
// ================================================================
void tare_heading() {
    headingOffset = currentHeading;
    velX = 0.0f;
    distX = 0.0f;
    lastTimeMicros = micros();
}

// -----------------------------------------------
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
      while (currentHeading < 0.0f) currentHeading += 360.0f;
      while (currentHeading >= 360.0f) currentHeading -= 360.0f;

      headingError = currentHeading - headingOffset;
      while (headingError < 0.0f) headingError += 360.0f;
      while (headingError >= 360.0f) headingError -= 360.0f;

      return headingError;
    }
  }
  
  return headingError;  // Return last known value instead of 0
}

// ================================================================
//  MECANUM DRIVE
//  x = strafe right (+1), y = forward (+1), rotation = CW (+1)
//  Servo follows strafe direction during obstacle avoidance only.
// ================================================================
void mecanumDrive(float x, float y, float rotation) {
    // if (avoid_obstacle_output_flag == 1) {
    //     if (x > 0)       sensor_servo.write(SERVO_RIGHT);
    //     else if (x < 0)  sensor_servo.write(SERVO_LEFT);
    //     else              sensor_servo.write(SERVO_CENTRE);
    // } else {
        sensor_servo.write(SERVO_CENTRE);  // ← ADD THIS
    // }

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
