// Firmware for a line following robot
// Olin College of Engineering PIE Miniproject 3

// Adafruit Motorsheild v2
#include <Adafruit_MotorShield.h>

// create motor sheild object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// assign motors to ports
Adafruit_DCMotor *leftWheel = AFMS.getMotor(3);   // left wheel port 3
Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);  // right wheel port 4
// assign sensor pins
const int RIGHT_SENSE = 0;  // analog pin 0
const int LEFT_SENSE = 1;   // analog pin 1

// function prototypes for organization
void setWheelLeft();
void setWheelRight();
void readSensors();
void stop();

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  Serial.print("START");

  // motors always run forwards
  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);

  // set initial motor speeds to 0
  rightWheel->setSpeed(0);
  leftWheel->setSpeed(0);
}

void loop() {
  while (Serial.available() == 0) {
    // wait for data available in serial receive buffer
  }
  String command = Serial.readStringUntil('\n');  // read until timeout
  if (command.startsWith("setWheelLeft")) {
    // note: wheel will be in the form "setWheelDirection N" where N is speed
    // extract speed from command string
    uint8_t speed = command.substring(command.indexOf(" ")+1, command.length()).toInt();
    setWheelLeft(speed);

  } else if (command.startsWith("setWheelRight")) {
    uint8_t speed = command.substring(command.indexOf(" ")+1, command.length()).toInt();
    setWheelRight(speed);

  } else if (command.equals("readSensors")) {
    // send sensor readings over serial
    readSensors();
  } else if (command.equals("STOP")) {
    // stop motors
    stop();
  }
}

void setWheelLeft(uint8_t speed){
Serial.println(speed);  // test
leftWheel->setSpeed(speed);
}
void setWheelRight(uint8_t speed){
Serial.println(speed);  // test
rightWheel->setSpeed(speed);
}
void readSensors(){
  // read right sensor voltage:
  int RVal = analogRead(RIGHT_SENSE);
  // read left sensor voltage:
  int LVal = analogRead(LEFT_SENSE);

  // print to serial as a tuple
  char buffer[40];
  sprintf(buffer, "(%d,%d)", RVal, LVal);
  Serial.println(buffer);
}
void stop(){
// stop both motors
leftWheel->setSpeed(0);
rightWheel->setSpeed(0);
}
