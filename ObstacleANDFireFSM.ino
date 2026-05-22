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
    FAN_OFF
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

int detect_fire_output_flag   = 0;
int cruise_output_flag        = 0;
int avoid_obstacle_output_flag = 0;
int extinguish_fire_output_flag = 0;
int realign_to_fire_output_flag = 0;

// define threshold of phototransistor difference
int photo_dead_zone = 5;

// fire detecting sensors and variables
int sensorValues[4];
int Photopins[] = {A8, A9, A10, A11}; // phototransistor pins
int fires_extinguished = 0;
bool scan_360 = 0;
int scan_number = 0;
float spin_angle = 0;
int detect_angles[2] = {0, 0};

int ir_detect;
int ultrasonic_distance;
int bumper_left;
int bumper_right;
int bumper_back;

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
float avoid_strafe_dir  = 0.0f;
bool  avoid_aligned     = false;
bool  currently_strafing = false;
float last_strafe_dir   = 0.0f;

//realign variables
int rotate = 0;
int last_dir = 0;
#define RIGHT 1
#define LEFT 0

const float IR_FRONT_DANGER_CM  = 7.0f;
const float IR_FRONT_WARNING_CM = 15.0f;
const float IR_REAR_DANGER_CM   = 15.0f;
const float SONAR_OBSTACLE_CM   = 15.0f;
const float SONAR_CLEAR_CM      = 10.0f;
const float SONAR_SIDE_CLEAR_CM = 25.0f;

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
    Serial.begin(115200);
    SerialCom = &Serial1;

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
    realign_to_fire_output_flag = 1;

    return RUNNING;
}

STATE running() {
    serial_read_conditions();
    cruise();
    avoid_obstacle();
    //realign_to_fire();
    arbitrate();
    return RUNNING;
}

STATE stopped() {
    disable_motors();
    stopRobot();
}

// ================================================================
//  CRUISE
// ================================================================
void cruise() {
    //float correction = constrain(straightPID.compute(heading_locked, getHeading()) * 0.01f, -0.3f, 0.3f);
    float correction = 0; // --------------------------- need to change once integrated with align_to_fire
    cruise_command = MOVE;
    move_input     = {0.0f, DRIVE_SPEED, correction};
    cruise_output_flag = 1;
}

void realign_to_fire() {

        if (sensorValues[3] > 10 || sensorValues[2] > 10) { //if light detected by long range
        int dif = sensorValues[0] - sensorValues[1];
        SerialCom->println(dif);
        if (sensorValues[1] > 10 && sensorValues[0] > 10) {
          if (abs(dif) <= 15) {
            move_input = {0.0f, 0.0f, 0.0f}; //STOP
            realign_to_fire_command = STOP;
            realign_to_fire_output_flag = 0;
          } else if (dif > 15) {
            move_input = {0.0f, 0.0f, -0.4f}; //CLOCKWISE
            realign_to_fire_command = MOVE;
            realign_to_fire_output_flag = 1;
            rotate = -1.0;
          } else if (dif < -15) {
            move_input = {0.0f, 0.0f, 0.4f}; //ANTI
            realign_to_fire_command = MOVE;
            realign_to_fire_output_flag = 1;
            rotate = 1.0;
          } 
        }
        } else if (sensorValues[2] <= 10 || sensorValues[3] <= 10) { //else if no light detected
        if (last_dir == LEFT) {
          rotate =  -1;
          last_dir = -1;
        } else if (last_dir == RIGHT) {
          rotate = 1;
          last_dir = -1;
        }
          move_input = {0.0f, 0.0f, (1.0f*rotate)}; 
          realign_to_fire_command = MOVE;
          realign_to_fire_output_flag = 1;
        }
}

// ================================================================
//  AVOID OBSTACLE
// ================================================================
void avoid_obstacle() {
    bool fl_blocked = (front_left_IR  < IR_FRONT_WARNING_CM);
    bool fr_blocked = (front_right_IR < IR_FRONT_WARNING_CM);
    bool rl_blocked = (rear_left_IR   < IR_REAR_DANGER_CM);
    bool rr_blocked = (rear_right_IR  < IR_REAR_DANGER_CM);

    if (last_strafe_dir == 0) {
        bool sonar_blocked = (sonar < SONAR_OBSTACLE_CM);

        // ── ESCAPE: rear blocked, front clear ───────────────────────
        if ((rl_blocked || rr_blocked) && !fl_blocked && !fr_blocked && !sonar_blocked) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {0.0f, 0.5f, 0.0f};
            last_strafe_dir            = 0;
            currently_strafing         = false;
            Serial.println(F("[AVOID] Rear blocked, front clear — nudging forward"));
            return;
        }

        // ── ALL CLEAR ───────────────────────────────────────────────
        if (!fl_blocked && !sonar_blocked && !fr_blocked && !rl_blocked && !rr_blocked) {
            avoid_obstacle_output_flag = 0;
            avoid_strafe_dir           = 0.0f;
            avoid_aligned              = false;
            currently_strafing         = false;
            last_strafe_dir            = 0;
            sonar_fwd_triggered        = false;
            sensor_servo.write(SERVO_CENTRE);
            Serial.println(F("[AVOID] All clear"));
            return;
        }

        // ── FL only → strafe right ──────────────────────────────────
        if (fl_blocked && !fr_blocked && !rr_blocked) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {1.0f, 0.0f, 0.0f};
            last_strafe_dir            = 1.0f;
            currently_strafing         = true;
            Serial.println(F("[AVOID] FL only — strafe right"));
            return;
        } else if (fl_blocked && !fr_blocked && rr_blocked) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {-1.0f, 0.0f, 0.0f};
            last_strafe_dir            = -1.0f;
            currently_strafing         = true;
            Serial.println(F("[AVOID] FL + RR — strafe left"));
            return;
        }

        // ── FR only → strafe left ───────────────────────────────────
        if (!fl_blocked && fr_blocked && !rl_blocked) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {-1.0f, 0.0f, 0.0f};
            last_strafe_dir            = -1.0f;
            currently_strafing         = true;
            Serial.println(F("[AVOID] FR only — strafe left"));
            return;
        } else if (!fl_blocked && fr_blocked && rl_blocked) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {1.0f, 0.0f, 0.0f};
            last_strafe_dir            = 1.0f;
            currently_strafing         = true;
            Serial.println(F("[AVOID] FR + RL — strafe right"));
            return;
        }

        // ── SONAR or both IR front blocked → stop, sweep, pick side ─
        if (sonar_blocked || (fl_blocked && fr_blocked)) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = STOP;
            move_input                 = {0.0f, 0.0f, 0.0f};
            robotMove();  // stop immediately before sweep

            sensor_servo.write(SERVO_LEFT);
            delay(1000);
            float left_dist = read_sonarsensor();

            sensor_servo.write(SERVO_RIGHT);
            delay(1000);
            float right_dist = read_sonarsensor();

            sensor_servo.write(SERVO_CENTRE);

            bool left_clear  = (left_dist  >= SONAR_SIDE_CLEAR_CM);
            bool right_clear = (right_dist >= SONAR_SIDE_CLEAR_CM);
            avoid_strafe_dir = (left_clear && !right_clear) ? -1.0f : 1.0f;

            if (sonar_blocked) {
                sonar_fwd_triggered = true;
                Serial.println(F("[AVOID] sonar_fwd_triggered set"));
            }

            avoid_obstacle_command = MOVE;
            move_input             = {avoid_strafe_dir, 0.0f, 0.0f};
            last_strafe_dir    = avoid_strafe_dir;
            currently_strafing = true;
            strafe_start_ms    = millis();   // ← ADD THIS
            return;
        }

    } else if (last_strafe_dir == -1) {
        bool sonar_side_blocked = (sonar < SONAR_SIDE_CLEAR_CM);

        if (!fl_blocked && !fr_blocked) {
            if (sonar_fwd_triggered) {
                
                // IR clear but sonar originally triggered — verify forward before exiting
                stopRobot();
                sensor_servo.write(SERVO_CENTRE);
                delay(500);
                float sonar_fwd_check = read_sonarsensor();
                sensor_servo.write(SERVO_LEFT);
                delay(500);

                if (sonar_fwd_check < SONAR_OBSTACLE_CM) {
                    avoid_obstacle_output_flag = 1;
                    avoid_obstacle_command     = MOVE;
                    move_input                 = {-1.0f, 0.0f, 0.0f};
                    last_strafe_dir            = -1.0f;
                    currently_strafing         = true;
                    Serial.println(F("[AVOID] IR clear but sonar fwd still blocked — keep strafing left"));
                    return;
                }

                sonar_fwd_triggered        = false;
                avoid_obstacle_output_flag = 0;
                avoid_strafe_dir           = 0.0f;
                avoid_aligned              = false;
                currently_strafing         = false;
                last_strafe_dir            = 0;
                Serial.println(F("[AVOID] IR + sonar fwd confirmed clear — exiting"));
                return;
            }

            // IR-only trigger — IR clear is enough to exit
            avoid_obstacle_output_flag = 0;
            avoid_strafe_dir           = 0.0f;
            avoid_aligned              = false;
            currently_strafing         = false;
            last_strafe_dir            = 0;
            delay(500);
            Serial.println(F("[AVOID] All clear — IR confirmed"));
            return;
        }

        if (sonar_side_blocked || rl_blocked) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {1.0f, 0.0f, 0.0f};
            last_strafe_dir            = 1.0f;
            currently_strafing         = true;
            return;
        } else {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {-1.0f, 0.0f, 0.0f};
            last_strafe_dir            = -1.0f;
            currently_strafing         = true;
            return;
        }

    } else {
        // last_strafe_dir == 1
        bool sonar_side_blocked = (sonar < SONAR_SIDE_CLEAR_CM);

        if (!fl_blocked && !fr_blocked) {
            if (sonar_fwd_triggered) {
              
                stopRobot();
                sensor_servo.write(SERVO_CENTRE);
                delay(500);
                float sonar_fwd_check = read_sonarsensor();
                sensor_servo.write(SERVO_RIGHT);
                delay(500);

                if (sonar_fwd_check < SONAR_OBSTACLE_CM) {
                    avoid_obstacle_output_flag = 1;
                    avoid_obstacle_command     = MOVE;
                    move_input                 = {1.0f, 0.0f, 0.0f};
                    last_strafe_dir            = 1.0f;
                    currently_strafing         = true;
                    Serial.println(F("[AVOID] IR clear but sonar fwd still blocked — keep strafing right"));
                    return;
                }

                sonar_fwd_triggered        = false;
                avoid_obstacle_output_flag = 0;
                avoid_strafe_dir           = 0.0f;
                avoid_aligned              = false;
                currently_strafing         = false;
                last_strafe_dir            = 0;
                Serial.println(F("[AVOID] IR + sonar fwd confirmed clear — exiting"));
                return;
            }

            avoid_obstacle_output_flag = 0;
            avoid_strafe_dir           = 0.0f;
            avoid_aligned              = false;
            currently_strafing         = false;
            last_strafe_dir            = 0;
            Serial.println(F("[AVOID] All clear — IR confirmed"));
            return;
        }

        if (sonar_side_blocked || rr_blocked) {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {-1.0f, 0.0f, 0.0f};
            last_strafe_dir            = -1.0f;
            currently_strafing         = true;
            return;
        } else {
            avoid_obstacle_output_flag = 1;
            avoid_obstacle_command     = MOVE;
            move_input                 = {1.0f, 0.0f, 0.0f};
            last_strafe_dir            = 1.0f;
            currently_strafing         = true;
            return;
        }
    }
}

// ================================================================
//  DETECT FIRE
// ================================================================
void detect_fire() {
    if (scan_360 == 0 && fires_extinguished == 0) {
        if (spin_angle > 355.0 || spin_angle < 5.0) {
            scan_360 = 1;
            detect_fire_command    = STOP;
            detect_fire_output_flag = 1;
        } else {
            if (sensorValues[3] > 0 && sensorValues[2] > 0) {
                detect_angles[scan_number] = spin_angle;
                scan_number++;
            }
            detect_fire_command    = MOVE;
            move_input             = {0.0f, 0.0f, 0.3f};
            detect_fire_output_flag = 1;
        }
    } else if (scan_360 == 1 && fires_extinguished == 0) {
        int min_angle = min(detect_angles[0], detect_angles[1]);
        if (spin_angle > min_angle - 5 && spin_angle < min_angle + 5) {
            detect_fire_command    = STOP;
            detect_fire_output_flag = 0;
        } else {
            detect_fire_command    = MOVE;
            move_input             = {0.0f, 0.0f, 0.3f};
            detect_fire_output_flag = 1;
        }
    }

    if (scan_360 == 1 && fires_extinguished == 1) {
        if (sensorValues[3] > 0 && sensorValues[2] > 0) {
            detect_fire_command    = STOP;
            detect_fire_output_flag = 0;
        } else {
            detect_fire_command    = MOVE;
            move_input             = {0.0f, 0.0f, 0.3f};
            detect_fire_output_flag = 1;
        }
    }
}

// ================================================================
//  EXTINGUISH FIRE
// ================================================================
void extinguish_fire() {
    if (sensorValues[1] <= 10 && sensorValues[0] <= 10) {
        extinguish_fire_command    = FAN_ON;
        extinguish_fire_output_flag = 1;
    } else {
        extinguish_fire_output_flag = 0;
    }
}

// ================================================================
//  ARBITRATE
// ================================================================
void arbitrate() {
    if (cruise_output_flag == 1)
        motor_input = cruise_command;
    if (realign_to_fire_output_flag == 1)
        motor_input = realign_to_fire_command;
    if (avoid_obstacle_output_flag == 1)
        motor_input = avoid_obstacle_command;
    if (extinguish_fire_output_flag == 1)
        motor_input = extinguish_fire_command;
    if (detect_fire_output_flag == 1)
        motor_input = detect_fire_command;
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
            // turn fan on
            break;
        case FAN_OFF:
            // turn fan off
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
//  MECANUM DRIVE
//  x = strafe right (+1), y = forward (+1), rotation = CW (+1)
//  Servo follows strafe direction during obstacle avoidance only.
// ================================================================
void mecanumDrive(float x, float y, float rotation) {
    if (avoid_obstacle_output_flag == 1) {
        if (x > 0)       sensor_servo.write(SERVO_RIGHT);
        else if (x < 0)  sensor_servo.write(SERVO_LEFT);
        else              sensor_servo.write(SERVO_CENTRE);
    } else {
        sensor_servo.write(SERVO_CENTRE);  // ← ADD THIS
    }

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
