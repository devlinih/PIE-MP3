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
void serialMonitor();
void leftSensorRead();
void rightSensorRead();

void setup() {
  // initialize serial communication
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// TODO
void serialMonitor() {
  while (Serial.available() == 0) {
    // wait for data available in serial receive buffer
  }
  String command = Serial.readStringUntil('\n');  // read until timeout
  command.trim();                                 // remove white space or \n
  if (command.equals("left_wheel_set")) {
  } else if (command.indexOf("right_wheel_set") > 0) {
  } else if (command.indexOf("left_sensor_read") > 0) {
  } else if (command.indexOf("right_sensor_read") > 0) {
  }
}