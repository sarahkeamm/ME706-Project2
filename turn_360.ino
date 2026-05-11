#include <math.h>
#include <Adafruit_BNO08x.h> 
#include <Servo.h>

#define M_PI 3.14159265358979f

// ******* GYROSCOPE ******* //
// --- BNO08X setup 
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float headingOffset = 0.0f;
float currentHeading = 0.0f;
float headingError = 0.0f;
float velX = 0.0f;
float distX = 0.0f;
unsigned long lastTimeMicros = 0;

float Kp = 80.0f; // start low, tune up


// ******** MOTORS ******** //
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29

int speed_val = 120;
float angle;

// Serial Pointer
HardwareSerial* SerialCom;
byte serialRead = 0; //for control serial communication

// Phototransistors
// Define the pins for the 4 phototransistors
const int photoPins[] = {A8, A9, A10, A11};
int sensorValues[4];

// pins 8 and 9 = short range w/ 15k resistors
  // 8 = right close
  // 9 = left close
// pins 10 and 11 = long range w/ 100k resistors
  // 10 = right far
  // 11 = left far

// -----------------------------------------------
void setup() {
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code");

  pinMode(LED_BUILTIN, OUTPUT);
  enable_motors();
  stop();

  for (int i = 0; i < 4; i++) {
    pinMode(photoPins[i], INPUT);
  }

   // Use USB Serial for debug output and reserve Serial1 for command input only.

  // if (!bno08x.begin_I2C()) {
  //   Serial.println("BNO08X not detected!");
  //   while (1);  // halt if sensor not found
  // }
   if (!bno08x.begin_I2C() || !bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000)) {
    while (1) {
      Serial.println("IMU failed");
      delay(100);
    }
  }

  // Wait a moment for the gyro to settle then tare
  GYRO_reading(); // get first reading so currentHeading is valid
  tare_heading(); // set current direction as "straight"
  ccw();
  delay(500);
}

// -----------------------------------------------

// void loop() {
//     GYRO_reading(); // updates currentHeading and headingError

//     int correction = (int)(Kp * headingError);
//     Serial.print("Correction: ");
//     Serial.println(correction);

//     forward(correction);
//     delay(5);
// }

void loop() {

  angle = GYRO_reading(); 
  
  // Treat "Close to 360" or "Close to 0" as the same thing
  if (angle > 355.0 || angle < 5.0) {
    stop();
  } else {
    ccw();
  }
  delay(50);
}


// -----------------------------------------------
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

      // Serial.print("Heading error (deg): ");
      // SerialCom->println(headingError);
      return headingError;
    }
  }
}

//----------------------Motor moments------------------------
// The Vex Motor Controller 29 use Servo Control signals to determine speed and
// direction, with 0 degrees meaning neutral
// https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_front_motor.detach();   // detach the servo on pin left_front to turn Vex
                               // Motor Controller 29 Off
  left_rear_motor.detach();    // detach the servo on pin left_rear to turn Vex
                               // Motor Controller 29 Off
  right_rear_motor.detach();   // detach the servo on pin right_rear to turn Vex
                               // Motor Controller 29 Off
  right_front_motor.detach();  // detach the servo on pin right_front to turn
                               // Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_front_motor.attach(left_front);  // attaches the servo on pin left_front
                                        // to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to
                                      // turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear
                                        // to turn Vex Motor Controller 29 On
  right_front_motor.attach(
      right_front);  // attaches the servo on pin right_front to turn Vex Motor
                     // Controller 29 On
}

void stop() {
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

// -----------------------------------------------
void forward(int correction) {
    correction = constrain(correction, -50, 50);

    left_front_motor.writeMicroseconds(1500 + speed_val + correction);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val + correction);
}

void cw() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}



