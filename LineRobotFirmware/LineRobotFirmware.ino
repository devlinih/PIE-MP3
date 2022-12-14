// Firmware for a line following robot
// Olin College of Engineering PIE Miniproject 3

// Adafruit Motorsheild v2
#include <Adafruit_MotorShield.h>

// create motor sheild object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Initialize global variables for DC motors
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
  // Serial.println("START");
  AFMS.begin();

  // set initial motor speeds to 0
  rightWheel->setSpeed(0);
  leftWheel->setSpeed(0);

  // motors always run forwards
  leftWheel->run(RELEASE);
  rightWheel->run(RELEASE);
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
    leftWheel->run(FORWARD);
    leftWheel->setSpeed(speed);
    Serial.println(speed);

  } else if (command.startsWith("setWheelRight")) {
    uint8_t speed = command.substring(command.indexOf(" ")+1, command.length()).toInt();
    rightWheel->run(FORWARD);
    rightWheel->setSpeed(speed);
    Serial.println(speed);

  } else if (command.startsWith("-setWheelLeft")) {
    uint8_t speed = command.substring(command.indexOf(" ")+1, command.length()).toInt();
    leftWheel->run(BACKWARD);
    leftWheel->setSpeed(speed);
    Serial.println(speed);

  } else if (command.startsWith("-setWheelRight")) {
    uint8_t speed = command.substring(command.indexOf(" ")+1, command.length()).toInt();
    rightWheel->run(BACKWARD);
    rightWheel->setSpeed(speed);
    Serial.println(speed);

  } else if (command.equals("readSensors")) {
    // send sensor readings over serial
    readSensors();
  } else if (command.equals("STOP")) {
    // stop motors
    rightWheel->run(RELEASE);
    leftWheel->run(RELEASE);
    Serial.println("0");
  }
}

void readSensors() {
  // read right sensor voltage:
  int RVal = analogRead(RIGHT_SENSE);
  // read left sensor voltage:
  int LVal = analogRead(LEFT_SENSE);

  // print to serial as a tuple
  char buffer[40];
  sprintf(buffer, "(%d,%d)", RVal, LVal);
  Serial.println(buffer);
}
