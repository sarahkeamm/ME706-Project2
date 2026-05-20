#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

// TEST VARIABLES
unsigned long lastDebugPrint = 0;
const int DEBUG_INTERVAL_MS = 200;

// define the control pin of each motor
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

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

// define motions states
enum MOTION{
MOVE,
STOP,
FAN_ON,
FAN_OFF
};

MOTION cruise_command;
MOTION avoid_obstacle_command;
MOTION extinguish_fire_command;
MOTION realign_to_fire_command;
MOTION detect_fire_command;
MOTION motor_input;

struct DriveCommand {
  float x;         // strafe: +1 = right, -1 = left
  float y;         // forward: +1 = forward
  float rotation;  // spin: +1 = CW, -1 = CCW
};

// declare function output and function flag - fire_detection, cruising, obstacle_avoidance, extinguishing_fire
DriveCommand move_input;

int detect_fire_output_flag = 0;
int cruise_output_flag = 0;
int avoid_obstacle_output_flag = 0;
int extinguish_fire_output_flag = 0;
int realign_to_fire_output_flag = 0;

//fire detecting sensors and variables
float headingOffset = 0.0f;
float currentHeading = 0.0f;
float headingError = 0.0f;
float velX = 0.0f;
float distX = 0.0f;
unsigned long lastTimeMicros = 0;
int fires_extinguished = 0;
bool scan_360 = 0;
int scan_number = 0;
float spin_angle = 0;
int detect_angles[2] = {0, 0};
int detect_distances[2] = {0, 0};

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
const int baseSpeed = 120;   // µs offset into mecanumDrive
int speed_change;

// ----------------------------
// avoid obstacle constants
// -------------------------
int avoid_strafe_dir = 0.0f;
bool avoid_aligned = false; 
float sonar_side = 0.0f;
bool currently_strafing = false; 
const float IR_FRONT_DANGER_CM  = 7.0f;  // front obstacle: strafe
const float IR_FRONT_WARNING_CM  = 15.0f;  // front obstacle: strafe
const float IR_REAR_DANGER_CM   = 15.0f;  // rear obstacle: nudge fwd
const float SONAR_OBSTACLE_CM   = 15.0f;  // sonar: blocked
const float SONAR_CLEAR_CM      = 10.0f;  // sonar: clear to drive
const float SONAR_SIDE_CLEAR_CM = 30.0f;  // side sweep: clear to strafe into

// ----------------------------------------------------------------
//  IR SENSOR PINS  (kept identical to base code)
// ----------------------------------------------------------------
const int IR_FRONT_LEFT  = A6;
const int IR_FRONT_RIGHT = A7;
const int IR_BACK_LEFT   = A4;
const int IR_BACK_RIGHT  = A5;

// ----------------------------------------------------------------
//  SONAR SERVO
// ----------------------------------------------------------------
Servo sensor_servo;
const int SERVO_PIN        = 10;
const int SERVO_LEFT       = 165;  // sweep left  (slightly inward)
const int SERVO_CENTRE     = 90;
const int SERVO_RIGHT      = 15;   // sweep right (slightly inward)
const int SWEEP_SETTLE_MS  = 250;  // ms to settle before reading

// Calibrated distances (cm) — filled by read_IR_sensors()
float front_left_IR  = 0;
float front_right_IR = 0;
float rear_left_IR   = 0;
float rear_right_IR  = 0;
float sonar_fwd      = 999;

// ----------------------------------------------------------------
//  PHOTO TRANSISTORS
// ----------------------------------------------------------------
int sensorValues[4];
int Photopins[] = {A8, A9, A10, A11}; // phototransistor pins

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
int fire_count = 0;

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
void  realign();

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

void setup() {
    Serial1.begin(115200);
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


void loop() {
  // put your main code here, to run repeatedly:
  static STATE machine_state = INITIALISING; // start from the sate INITIALIING
  switch (machine_state)
  {
  case INITIALISING:
  machine_state = initialising();
  break;
  case RUNNING:
  machine_state = running();
  break;
  case STOPPED:
  machine_state = stopped();
  break;
  }
}


STATE initialising() {
    enable_motors();
    Serial.println("INITIALISING");

    // Tare IMU
    delay(500);
    for (int i = 0; i < 50; i++) { updateIMU(); delay(10); }
    yaw_offset = yaw_raw;

    // Align to hardcoded fire bearing and lock heading
    spin_to_heading(FIRE_BEARING_DEG);
    heading_locked = getHeading();
    straightPID.reset();

    return RUNNING;
}

STATE running(){
    serial_read_conditions(); //read all sensors
    // four function
    detect_fire();
    // cruise();
    // avoid_obstacle();
    //realign_to_fire();
    // extinguish_fire();
    // select the output command based on the function priority
    arbitrate();
    //should set all sensor values to 0 here
    return RUNNING; // return to RUNNING STATE again, it will run the RUNNING
} // STATE REPEATLY


STATE stopped(){
    disable_motors(); // disable the motors
    stopRobot();
}

void speed_change_smooth() // change speed, called in RUNING STATE 
{
speed_val += speed_change; // speed value add on speed change
if(speed_val > 500) // make sure speed change less than 5000
speed_val = 500;
speed_change = 0; //make speed change equals 0 after updating the speed value
}

 
//have flag for how many fires extinguished, 360 turn**
//if no fires extinguished - full 360 turn, record angles of light detected and turn to strongest
//if 1 fire extinguished - turn until light detected
//return detect_fire_flag = 0 when robot is directed to the light

void detect_fire()
{
  SerialCom->println(scan_360);
    //initial scan for fire
    if (scan_360 == 0 && fires_extinguished == 0) {
        //make sure enough space for robot to turn 360 degrees
        if (spin_angle >= 350.0 && spin_angle < 358.0) {
            SerialCom->println("done");
            scan_360 = 1;
            move_input = {0.0f, 0.0f, 0.0f}; // STOP
            detect_fire_command = STOP;
            detect_fire_output_flag = 1;
        } else if (spin_angle < 345.0) {
            // SerialCom->println("spinning");
            int dif = abs(sensorValues[3] - sensorValues[2]);
            if (sensorValues[3] > 200 && sensorValues[2] > 200 && dif <= 50) {
                SerialCom->println(spin_angle);
                detect_angles[scan_number] = spin_angle;
                detect_distances[scan_number] = (sensorValues[3] + sensorValues[2]) / 2.0f; // or some function of sensorValues[2] and sensorValues[3]
                scan_number++;
            }
            move_input = {0.0f, 0.0f, 0.8f}; // CLOCKWISE 
            detect_fire_command = MOVE;
            detect_fire_output_flag = 1;
        }
    } 
    // else if (scan_360 == 1 && fires_extinguished == 0) {
    //     int min_distance = min(detect_distances[0], detect_distances[1]);
    //     int min_angle = 0;
    //     if (min_distance == detect_distances[0]) {
    //       min_angle = detect_angles[0];
    //     } else {
    //       min_angle = detect_angles[1];
    //     }
    //     if (spin_angle > min_angle - 5 && spin_angle < min_angle + 5) {
    //         move_input = {0.0f, 0.0f, 0.0f}; // STOP
    //         detect_fire_command = STOP;
    //         detect_fire_output_flag = 0;
    //     } else {
    //         move_input = {0.0f, 0.0f, 0.8f}; // CLOCKWISE
    //         detect_fire_command = MOVE;
    //         detect_fire_output_flag = 1;
    //     }
    // }
    
    // //rescanning after extinguishing first fire
    // if (scan_360 == 1 && fires_extinguished == 1) {
    //   int dif = abs(sensorValues[3] - sensorValues[2]);
    //     if (sensorValues[3] > 100 && sensorValues[2] > 100 && (dif <= 100)) {
    //         SerialCom->println(sensorValues[3]);
    //         SerialCom->println(sensorValues[2]);
    //         move_input = {0.0f, 0.0f, 0.0f}; // STOP
    //         detect_fire_command = STOP;
    //         detect_fire_output_flag = 0;
    //     } else {
    //         move_input = {0.0f, 0.0f, 0.8f}; //CLOCKWISE
    //         detect_fire_command = MOVE;
    //         detect_fire_output_flag = 1;
    //     }
    // } 
}

// cruise function output command and flag
// void cruise()
// {
//     float correction = constrain(straightPID.compute(heading_locked, getHeading()) * 0.01f, -0.3f, 0.3f);
//     cruise_command = {0.0f, DRIVE_SPEED, correction};
//     cruise_output_flag=1;
// }

// void avoid_obstacle() {
//   bool front_left_blocked  = (front_left_IR  < IR_FRONT_WARNING_CM );
//   bool front_right_blocked = (front_right_IR < IR_FRONT_WARNING_CM );
//   bool sonar_blocked       = (sonar_fwd      < SONAR_OBSTACLE_CM);
//   bool rear_left_blocked  = (front_left_IR  < IR_REAR_DANGER_CM);
//   bool rear_right_blocked = (front_right_IR < IR_REAR_DANGER_CM);

//   // ── All clear ───────────────────────────────────────────────────
//   if (!front_left_blocked && !front_right_blocked && !sonar_blocked) {
//     if (rear_left_blocked || rear_right_blocked) {
//       // Rear clipping something — nudge forward to clear it
//       avoid_obstacle_output_flag = 1;
//       //realign_to_fire_output_flag==1
//       avoid_obstacle_command = {0.0f, 0.5f, 0.0f};
//       Serial.println(F("[AVOID] Rear blocked — nudging forward"));
//     } else {
//       avoid_obstacle_output_flag = 0;  // fully clear, hand back to cruise
//       avoid_strafe_dir           = 0.0f;
//       avoid_aligned              = false;
//       currently_strafing = false;  
//       Serial.println(F("[AVOID] All clear — handing to cruise"));
//     }
//     return;
//   }

//   // ── Only sonar triggered (early warning) — no IR contact yet ────
//   if (!front_left_blocked && !front_right_blocked && sonar_blocked) {
//       avoid_obstacle_output_flag = 1;

//       // Only pick direction once — latch it just like the both-blocked case
//       if (avoid_strafe_dir == 0.0f) {
//           stopRobot();
//           sensor_servo.write(SERVO_LEFT);
//           delay(1000);
//           float left_dist = read_sonarsensor();

//           sensor_servo.write(SERVO_RIGHT);
//           delay(1000);
//           float right_dist = read_sonarsensor();

//           sensor_servo.write(SERVO_CENTRE);

//           float margin = 10.0f;
//           if (left_dist > right_dist + margin) {
//               avoid_strafe_dir = -1.0f;
//           } else if (right_dist > left_dist + margin) {
//               avoid_strafe_dir =  1.0f;
//           } else {
//               float fire_error = wrapAngle(heading_locked - getHeading());
//               avoid_strafe_dir = (fire_error < 0.0f) ? -1.0f : 1.0f;
//           }
//           Serial.print(F("[AVOID] Sonar — direction picked: "));
//           Serial.println(avoid_strafe_dir < 0 ? "LEFT" : "RIGHT");
//       }

//       avoid_obstacle_command = {avoid_strafe_dir, 0.0f, 0.0f};
//       return;
//     }

//   // ── Only left IR triggered — strafe right ───────────────────────
//   if (front_left_blocked && !front_right_blocked) {
//     avoid_obstacle_output_flag = 1;
//     //realign_to_fire_output_flag==1
//     avoid_obstacle_command = {1.0f, 0.0f, 0.0f};
//     Serial.println(F("[AVOID] Left IR — strafing right"));
//     return;
//   }

//   // ── Only right IR triggered — strafe left ───────────────────────
//   if (!front_left_blocked && front_right_blocked) {
//     avoid_obstacle_output_flag = 1;
//     //realign_to_fire_output_flag==1
//     avoid_obstacle_command = {-1.0f, 0.0f, 0.0f};
//     Serial.println(F("[AVOID] Right IR — strafing left"));
//     return;
//   }

//     // ── Both blocked ─────────────────────────────────────────────────
//   avoid_obstacle_output_flag = 1;
//   //currently_strafing = false;
//   //realign_to_fire_output_flag==1

//   // Step 1: rotate until both IRs read the same (wall-parallel)
//   // Returns early each loop iteration — avoid_aligned latches true once done
//   //  if (!avoid_aligned) {
//   //   bool danger_left  = (front_left_IR  < IR_FRONT_DANGER_CM);
//   //   bool danger_right = (front_right_IR < IR_FRONT_DANGER_CM);

//   //   if (!danger_left && !danger_right) {
//   //     // Both triggered at WARNING but not DANGER yet — just stop and wait
//   //     // to get close enough for a meaningful IR diff
//   //     avoid_obstacle_command = {0.0f, 0.0f, 0.0f};
//   //     Serial.println(F("[AVOID] Waiting to get closer for wall align"));
//   //     return;
//   //   }

//   //   float diff = front_left_IR - front_right_IR;
//   //   Serial.print(F("[AVOID] Aligning to wall, diff: "));
//   //   Serial.println(diff, 1);
//   //   if (fabs(diff) < 3.0f) {
//   //     avoid_aligned = true;
//   //   } else {
//   //     float rot = constrain(diff * 0.25f, -0.3f, 0.3f);
//   //     avoid_obstacle_command = {0.0f, 0.0f, rot};
//   //     return;
//   //   }
//   // }

//   // Step 2: pick strafe direction once — servo looks left then right
//   // avoid_strafe_dir latches so we don't re-check every loop
//   if (avoid_strafe_dir == 0.0f) {
//     stopRobot();
//     sensor_servo.write(0);               // look left (0° = robot's left)
//     delay(1000);
//     float left_dist = read_sonarsensor();
  
//     sensor_servo.write(180);             // look right
//     delay(1000);
//     float right_dist = read_sonarsensor();

//     sensor_servo.write(90);              // return to forward-facing

//     float margin = 10.0f;               // tune (cm) — gap needed to prefer one side
//     if (left_dist > right_dist + margin) {
//       avoid_strafe_dir = -1.0f;          // left clearly clearer
//     } else if (right_dist > left_dist + margin) {
//       avoid_strafe_dir =  1.0f;          // right clearly clearer
//     } else {
//       // Tied — strafe whichever way reduces angle to fire
//       float fire_error = wrapAngle(heading_locked - getHeading());
//       avoid_strafe_dir = (fire_error < 0.0f) ? -1.0f : 1.0f;
//     }
//     Serial.print(F("[AVOID] Direction picked: "));
//     Serial.println(avoid_strafe_dir < 0 ? "LEFT" : "RIGHT");
//   }

//   // Step 3: strafe — check if side is blocked, reset direction if so
//   if (sonar_side < SONAR_SIDE_CLEAR_CM) {
//     Serial.println(F("[AVOID] Side blocked — stopping, rechecking direction"));
//     // Side blocked — stop and force recheck next loop
//     avoid_strafe_dir = 0.0f;
//     avoid_aligned    = false;  // realign to wall again before rechecking
//     avoid_obstacle_command = {0.0f, 0.0f, 0.0f};  // stop
//     return;
//   }

//   // Step 3: strafe — direction is locked until front clears
//   currently_strafing = true;

//    // Read side sonar fresh right here
//   int servo_angle = (avoid_strafe_dir < 0.0f) ? SERVO_LEFT : SERVO_RIGHT;
//   sensor_servo.write(servo_angle);
//   delay(60);
//   sonar_side = read_sonarsensor();
//   sensor_servo.write(SERVO_CENTRE);

//   if (sonar_side < SONAR_SIDE_CLEAR_CM) {
//     Serial.print(F("[AVOID] Side blocked (")); Serial.print(sonar_side); Serial.println(F("cm) — rechecking direction"));
//     avoid_strafe_dir = 0.0f;
//     avoid_obstacle_command = {0.0f, 0.0f, 0.0f};
//     currently_strafing = false;
//     return;
//   }
  
//   Serial.print(F("[AVOID] Strafing: "));
//   Serial.println(avoid_strafe_dir < 0 ? "LEFT" : "RIGHT");
//   avoid_obstacle_command = {avoid_strafe_dir, 0.0f, 0.0f};
// }

void extinguish_fire()
{
    // //check distance from fire, if close enough set motor input to FAN
    // //otherwise set output flag to 0
    // if (sensorValues[1] <= 10 && sensorValues[0] <= 10) {
    //     extinguish_fire_command = FAN_ON;
    //     extinguish_fire_output_flag = 1;
    // } else {
    //     extinguish_fire_output_flag = 0;
    // }
}


// check flag and select command based on priority
void arbitrate ()
{
    if (cruise_output_flag==1)
    {motor_input=cruise_command;}
    if (realign_to_fire_output_flag==1) 
    {motor_input=realign_to_fire_command;}
    if (avoid_obstacle_output_flag ==1)
    {motor_input=avoid_obstacle_command;}
    // if (extinguish_fire_output_flag==1) //check if fire is close enough to extinguish
    // {fan_input=extinguish_fire_command;}
    if (detect_fire_output_flag==1) //first priority to detect fire
    {motor_input=detect_fire_command;}
    robotMove();
}


// read simulative sensor reading
void serial_read_conditions() {
    updateIMU();
    read_IR_sensors();

    for (int i = 0; i < 4; i++) {
        sensorValues[i] = analogRead(Photopins[i]);
    }

    // keep spin_angle in 0..360 using the IMU heading
    spin_angle = GYRO_reading();

    // Always read forward sonar here — servo stays centred
    sensor_servo.write(SERVO_CENTRE);
    delay(60);
    sonar_fwd = read_sonarsensor();

   
    // Debug print
    if (millis() - lastDebugPrint > DEBUG_INTERVAL_MS) {
        lastDebugPrint = millis();
        Serial.print(F("HDG:")); Serial.print(getHeading(), 1);
        Serial.print(F(" FL:")); Serial.print(front_left_IR, 1);
        Serial.print(F(" FR:")); Serial.print(front_right_IR, 1);
        Serial.print(F(" RL:")); Serial.print(rear_left_IR, 1);
        Serial.print(F(" RR:")); Serial.print(rear_right_IR, 1);
        Serial.print(F(" FWD:")); Serial.print(sonar_fwd, 1);
        Serial.print(F(" SIDE:")); Serial.print(sonar_side, 1);
        Serial.print(F(" STRAF_DIR:")); Serial.print(avoid_strafe_dir);
        Serial.print(F(" ALIGNED:")); Serial.print(avoid_aligned);
        Serial.print(F(" STRAFING:")); Serial.println(currently_strafing);
    }
}

void robotMove() {
   switch(motor_input)
  {
    case MOVE:
    mecanumDrive(move_input.x, move_input.y, move_input.rotation);
    break;
    case STOP:
    stopRobot();
    break;
    case FAN_ON:
    //turn fan on
    break;
  }
}

// ----------------------------------------------------------------- MOTOR COMANDS -----------------------------------------------------------

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
  // front_left_IR is long range (IR3 from calibration)
  front_left_IR = 17948*pow(signal1,-1.22); 
  front_left_IR = (front_left_IR + 4.4859) / 1.0697; // correction long
  
  // front_right_IR is long range (IR4 from calibration)
  front_right_IR = 17948.0f*pow(signal2, -1.22f);
  front_right_IR = (front_right_IR + 5.6134) / 1.14117; //correction long 

  // Rear sensors — medium range calibration
  // rear_left_IR is med range (IR1 from calibration)
  rear_left_IR = 17948*pow(signal3,-1.22);
  rear_left_IR = (rear_left_IR + 7.7957) / 2.5496;

  //rear_right_IR is med range (IR4 from calibration)
  rear_right_IR = 17948.0f * pow(signal4, -1.22f);
  rear_right_IR = (rear_right_IR + 10.8049f) / 2.9308f;

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
      Serial.println("HC-SR04: NOT found");
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
      Serial.println("HC-SR04: Out of range");
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
    // Serial.println("HC-SR04: Out of range");
  } else {
    // Serial.print("HC-SR04:");
    // Serial.print(cm);
    // SerialCom->println("cm");
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
