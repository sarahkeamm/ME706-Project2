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

float Kp = 40.0f; // start low, tune up


// ******** MOTORS ******** //
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29

int speed_val = 150;



// -----------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  enable_motors();
  stop();

   // Use USB Serial for debug output and reserve Serial1 for command input only.
  // SerialCom = &Serial1;
  // SerialCom->begin(115200);
  // SerialCom->println("MECHENG706_Base_Code");

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
  delay(500);
  GYRO_reading(); // get first reading so currentHeading is valid
  tare_heading(); // set current direction as "straight"
}

// -----------------------------------------------

// void loop() {
//     GYRO_reading(); // updates currentHeading and headingError

//     int correction = (int)(Kp * headingError);

//     forward(correction);
// }

void loop() {
    GYRO_reading(); // just watch the serial output
    delay(100);
}

// -----------------------------------------------
void tare_heading() {
    headingOffset = currentHeading;
    velX = 0.0f;
    distX = 0.0f;
    lastTimeMicros = micros();
}

// -----------------------------------------------
void GYRO_reading() {
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
      currentHeading = yaw;

      headingError = currentHeading - headingOffset;

      // Normalize to [-PI, PI]
      while (headingError >  M_PI) headingError -= 2.0f * M_PI;
      while (headingError < -M_PI) headingError += 2.0f * M_PI;

      // SerialCom->print("Heading error (deg): ");
      // SerialCom->println(headingError * 180.0f / M_PI);
      Serial.print("Heading error (deg): ");
      Serial.println(headingError * 180.0f / M_PI);
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
    correction = constrain(correction, -200, 200);

    left_front_motor.writeMicroseconds(1500 + speed_val - correction);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction);
    right_rear_motor.writeMicroseconds(1500 - speed_val + correction);
    right_front_motor.writeMicroseconds(1500 - speed_val + correction);
}



