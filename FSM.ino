

# include <Servo.h> // include the library of servo motor control
// define the control pin of each motor
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// three machine states
enum STATE {
INITIALISING,
RUNNING,
STOPPED
};

// define motions states
enum MOTION{
FORWARD,
BACKWARD,
STRAFE_RIGHT,
STRAFE_LEFT,
CLOCKWISE,
COUNTERCLOCKWISE, 
FAN
};


// declare function output and function flag - fire_detection, cruising, obstacle_avoidance, extinguishing_fire
MOTION detect_fire_command;
int detect_fire_output_flag;
MOTION cruise_command;
int cruise_output_flag;
MOTION avoid_obstacle_command;
int avoid_obstacle_output_flag;
MOTION extinguish_fire_command;
int extinguish_fire_output_flag;
MOTION realign_to_fire_command;
int realign_to_fire_output_flag;
MOTION motor_input;


// define threshold of phototransistor difference
int photo_dead_zone = 5;

//fire detecting sensors and variables
int sensorValues[4];
int Photopins[] = {A8, A9, A10, A11}; // phototransistor pins
int fires_extinguished = 0;
bool scan_360 = 0;
float spin_angle = 0;
int detect_angles[2] = {0, 0};

int ir_detect;
int ultrasonic_distance;
int bumper_left;
int bumper_right;
int bumper_back;
// create servo objects for each motor
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;


int speed_val = 120;
int speed_change;


void setup() {
    Serial.begin(9600); // start serial communication

    //phototransistors setup
    for (int i = 0; i < 4; i++) {
    pinMode(photoPins[i], INPUT);
    }
}


void loop() {
  // put your main code here, to run repeatedly:
  static STATE machine_state = INITIALISING; // start from the sate
  INITIALIING
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


STATE initialising(){
  enable_motors(); // enable motors
  //check gryo
  //move servo
  Serial.println("INITIALISING"); // print the current stage
  return RUNNING; // return to RUNING STATE DIRECTLY
}

STATE running(){
    serial_read_conditions(); //read all sensors
    // four function
    detect_fire();
    cruise();
    avoid_obstacle();
    realign_to_fire();
    extinguish_fire();
    // select the output command based on the function priority
    arbitrate();
    //should set all sensor values to 0 here
    return RUNNING; // return to RUNNING STATE again, it will run the RUNNING
} // STATE REPEATLY


STATE stopped(){
    disable_motors(); // disable the motors
}

void speed_change_smooth() // change speed, called in RUNING STATE 
{
speed_val += speed_change; // speed value add on speed change
if(speed_val > 500) // make sure speed change less than 5000
speed_val = 500;
speed_change = 0; //make speed change equals 0 after updating the speed value
}

float GYRO_reading_angle() {
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

//have flag for how many fires extinguished, 360 turn**
//if no fires extinguished - full 360 turn, record angles of light detected and turn to strongest
//if 1 fire extinguished - turn until light detected
//return detect_fire_flag = 0 when robot is directed to the light

void detect_fire()
{
    //initial scan for fire
    if (scan_360 == 0 && fires_extinguished == 0) {
        //make sure enough space for robot to turn 360 degrees
        if (spin_angle > 355.0 || spin_angle < 5.0) {
            scan_360 = 1;
            detect_fire_commmand = STOP;
            detect_fire_output_flag = 1;
        } else {
            if (sensorValues[3] > 0 && sensorValues[2] > 0) {
                detect_angles[scan_number] = spin_angle;
                //also read distances
                scan_number++;
            }
            detect_fire_command = CLOCKWISE;
            detect_fire_output_flag = 1;
        }
    } else if (scan_360 == 1 && fires_extinguished == 0) {
        int min_angle = min(detect_angles[0], detect_angles[1]);
        if (spin_angle > min_angle - 5 && spin_angle < min_angle + 5) {
            detect_fire_command = STOP;
            detect_fire_output_flag = 0;
        } else {
            detect_fire_command = CLOCKWISE;
            detect_fire_output_flag = 1;
        }
    }
    
    //rescanning after extinguishing first fire
    if (scan_360 == 1 && fires_extinguished == 1) {
        if (sensorValues[3] > 0 && sensorValues[2] > 0) {
            detect_fire_command = STOP;
            detect_fire_output_flag = 0;
        } else {
            detect_fire_command = CLOCKWISE;
            detect_fire_output_flag = 1;
        }
    } 
}

// cruise function output command and flag
void cruise()
{
    cruise_command = FORWARD;
    cruise_output_flag=1;
}

void avoid_obstacle()
{

}

void extinguish_fire()
{
    //check distance from fire, if close enough set motor input to FAN
    //otherwise set output flag to 0
    if (sensorValues[1] <= 10 && sensorValues[0] <= 10) {
        extinguish_fire_command = FAN;
        extinguish_fire_output_flag = 1;
    } else {
        extinguish_fire_output_flag = 0;
    }
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
    if (extinguish_fire_output_flag==1) //check if fire is close enough to extinguish
    {motor_input=extinguish_fire_command;}
    if (detect_fire_output_flag==1) //first priority to detect fire
    {motor_input=detect_fire_command;}
    robotMove();
}


// read simulative sensor reading
void serial_read_conditions() {
    //read phototransistors
    for (int i = 0; i < 4; i++) {
    sensorValues[i] = analogRead(photoPins[i]);
    }

    spin_angle = GYRO_reading_angle(); //read gyro angle
}

void robotMove() {
  switch(motor_input)
  {
    case FORWARD:
    forward ();
    delay(1000);
    break;
    case BACKWARD:
    reverse ();
    delay(1000);
    break;
    case STRAFE_RIGHT:
    strafe_right();
    delay(1000);
    break;
    case STRAFE_LEFT:
    strafe_left();
    delay(1000);
    break;
    case CLOCKWISE:
    cw();
    delay(1000);
    break;
    case COUNTERCLOCKWISE:
    ccw();
    delay(1000);
    break;
    case FAN:
    //fan_on();
    delay(10000);
    //fan_off();
    break;
  }
}

// ----------------------------------------------------------------- MOTOR COMANDS -----------------------------------------------------------

void disable_motors(){ // function disable all motors, called in STOPPED STATE
    left_front_motor.detach();
    left_rear_motor.detach();
    right_rear_motor.detach();
    right_front_motor.detach();
    pinMode(left_front,INPUT); // set pinMode for next step
    pinMode(left_rear,INPUT);
    pinMode(right_rear,INPUT);
    pinMode(right_front,INPUT);
}

void enable_motors() { //enable all motors, was called in INITIALZING SATE
    left_front_motor.attach(left_front);
    left_rear_motor.attach(left_rear);
    right_rear_motor.attach(right_rear);
    right_front_motor.attach(right_front);
}

void stop(){ // stop motors
    left_front_motor.writeMicroseconds(1500);
    left_rear_motor.writeMicroseconds(1500);
    right_rear_motor.writeMicroseconds(1500);
    right_front_motor.writeMicroseconds(1500);
}

void forward(){ // moving forward
    left_front_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val);
}

void reverse(){ // reverse
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_front_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left(){ // straight left
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right(){ //straight right
    left_front_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 + speed_val);
}

void cw(){ //clockwise
    left_front_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw(){ //anticlockwise
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val);
}

void reverse_ccw()
{
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_front_motor.writeMicroseconds(1500 + speed_val);
    delay(500);
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val);
}
