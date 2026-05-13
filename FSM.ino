

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
// define the sensor reading results
int photo_left ;
int photo_right;
int ir_detect;
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
    //read_serial_command(); // read command from serial communication
    speed_change_smooth(); //function to speed up and slow down smoothly
    // this is just for test functions to read simulative sensor reading from monitor
    serial_read_conditions();
    // four function
    cruise();
    follow();
    avoid();
    escape();
    // select the output command based on the function priority
    arbitrate();
    photo_left = 0;
    photo_right = 0;
    ir_detect = 0;
    bumper_left = 0;
    bumper_right = 0;
    bumper_back = 0;
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


//have flag for how many fires extinguished, 360 turn**
//if no fires extinguished - full 360 turn, record angles of light detected and turn to strongest
//if 1 fire extinguished - turn until light detected
//return detect_fire_flag = 0 when robot is directed to the light

void detect_fire()
{
    if (fires_detected == 0) {
        //make sure enough space for robot to turn 360 degrees
        //360 turn and record angles of light detected
        //turn to the strongest light set flag as 0
    } else if (fires_detected == 1) {
        //turn until light detected
        //set flag as 0 when light detected
    } else {
        detect_fire_output_flag = 0;
    }
}

// cruise function output command and flag
void cruise()
{

}

void avoid_obstacle()
{

}

void extinguish_fire()
{
    extinguish_fire_output_flag = 1;
    extinguish_fire_command = FAN;
}


// check flag and select command based on priority
void arbitrate ()
{

}


// read simulative sensor reading
void serial_read_conditions() {

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
