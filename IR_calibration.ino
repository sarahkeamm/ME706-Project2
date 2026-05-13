#include <Servo.h>  //Need for Servo pulse output

//#include <Adafruit_BNO08x.h>  //Need for Gyroscope

//Gyroscope initialisation
//Adafruit_BNO08x bno08x(-1);
//sh2_SensorValue_t sensorValue;
float rad = 0.0;

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//Servo Setup
Servo sensor_servo;  

int speed_val = 100;
int speed_change;

// variables for IR and sonar sensors

int sensor1 = A4; //frontleftsensor is attached on pinA0
int sensor2 = A5; //frontleftsensor is attached on pinA1
int sensor3 = A6; //frontleftsensor is attached on pinA2
int sensor4 = A7; //frontleftsensor is attached on pinA3

byte serialRead = 0; //for control serial communication

int signal4 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signal1 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signal2 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signal3 = 0; // the read out signal in 0-1023 corresponding to 0-5v

bool wall = false;

float IR1 = 0; // the calculated distance in cm from the front left sensor
float IR2 = 0; // the calculated distance in cm from the back left sensor
float IR3 = 0; // the calculated distance in cm from the front right sensor
float IR4 = 0; // the calculated distance in cm from the back right sensor   
float sonarsensor_cm = 0; // the calculated distance in cm from the sonar sensor
float left_sonarsensor_cm = 0; 
float right_sonarsensor_cm = 0; 
float horizontal_distance_cm = 0; // the calculated horizontal distance from the front of the robot to the wall
float turn_angle = 0; // the calculated angle to turn to be parallel to the wall

float robot_width = 20.5; // the width of the robot in cm
float robot_length = 22.5; // the length of the robot in cm
float map_width = 121.5; // the width of the map in cm
float map_length = 199; // the length of the map in cm

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void) {
  Serial.begin(115200);
  turret_motor.attach(11);
  enable_motors();
  stop();
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Servo Setup for ultrasonic sensor
  sensor_servo.attach(10);
  sensor_servo.write(90);

  // Use USB Serial for debug output and reserve Serial1 for command input only.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000);  //settling time but no really needed
}


void loop(){
  read_IR_sensors();
  Serial.print(IR1);
  Serial.print(", ");
  Serial.print(IR2);  
  Serial.print(", ");  
  Serial.print(IR3);
  Serial.print(", ");
  Serial.print(IR4);  
  Serial.println(";");
  delay(1000);
}


void read_IR_sensors(){
  long sum1= 0, sum2 =0, sum3 = 0, sum4 = 0;
  for (int i = 0; i < 4; i++) {
    sum1 += analogRead(sensor1);
    sum2 += analogRead(sensor2);
    sum3 += analogRead(sensor3);
    sum4 += analogRead(sensor4);
    delay(5);
  }
  signal1 = sum1/4;
  signal2 = sum2/4;
  signal3 = sum3/4;
  signal4 = sum4/4;
 
  IR1 = 17948*pow(signal1,-1.22);
  IR1 = (IR1 + 7.7957) / 2.5496; //correction medium 
 // IR1 = (IR1 - 0.1596) / 0.8007; // correction long
  IR2= 17948*pow(signal2,-1.22);
  IR2 = (IR2 + 10.9421) / 3.04; //correction medium 
 // IR2 = (IR2 + 2.0700) / 0.9163; // correction long 
  IR3= 17948*pow(signal3,-1.22); 
  IR3 = (IR3 + 10.6425) / 2.9208; //correction medium 
 // IR3 = (IR3 + 4.4859) / 1.0697; // correction long
  IR4= 17948*pow(signal4,-1.22);
  IR4 = (IR4 + 10.8049) / 2.9308; //correction medium 
  //IR4 = (IR4 + 5.6134) / 1.14117; //correction long 
}

  

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


//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}