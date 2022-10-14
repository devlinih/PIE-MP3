---
fontsize: 12pt
geometry:
    - margin=1in
urlcolor: blue
figPrefix:
    - "Figure"
    - "Figures"
lstPrefix:
    - "Listing"
    - "Listings"
codeBlockCaptions: true
---

# Appendix

Source code can also be found in the project's [Github Repository](https://github.com/devlinih/PIE-MP3).

```cpp
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
```
: Complete Firmware Source for `LineRobotFirmware.ino` {#lst:Firmware}

```python
"""
Connect to an Arduino running PIE MP3 Firmware.
"""

import ast
import serial
import serial.tools.list_ports as list_ports

BAUDRATE = 115200
ARDUINO_TIMEOUT = 3  # seconds

# List of Arduino IDs provided by Brad
# https://github.com/bminch/PIE/blob/main/Serial_cmd.py
ARDUINO_IDS = ((0x2341, 0x0043), (0x2341, 0x0001),
               (0x2A03, 0x0043), (0x2341, 0x0243),
               (0x0403, 0x6001), (0x1A86, 0x7523))


def guess_port() -> str:
    """
    Try to find an Arduino connected to the computer.

    Returns:
        A string representing the port of the first Arduino found. Returns the
        empty string if no Arduino is found.
    """
    devices = list_ports.comports()
    for device in devices:
        if (device.vid, device.pid) in ARDUINO_IDS:
            return device.device
    return ""


def send_command(ser: serial.Serial, command: str) -> str:
    """
    Send a command to an Arduino connected over serial.

    Args:
        ser: A serial connection to an Arduiono.
        command: A string representing a command to send to the Arduino.

    Returns:
        A string representing the received message from the Arduino.
    """
    message = bytes(command.strip() + "\n", "utf-8")
    ser.write(message)
    data = ser.readline()
    # data is a bytestring, return it as a number.
    return data.decode("utf-8").strip()


def parse_tuple(message: str) -> tuple[int, int]:
    """
    Process a string in the form '(a,b)' and return a tuple.

    Args:
        message: A string representing a tuple of ints.

    Returns:
        A tuple of two ints. If message is invalid, returns the tuple (0, 0).
    """
    try:
        return ast.literal_eval(message)
    except:
        return (0, 0)
```
: Complete listing for `arduino.py` {#lst:arduino.py}

```python
"""
Control loop for line following robot.
"""

import serial
import time

from arduino import (guess_port,
                     send_command,
                     parse_tuple,
                     BAUDRATE,
                     ARDUINO_TIMEOUT,)

DEFAULT_THRESHOLD = 780
DEFAULT_SPEED = 35


def move_straight(arduino: serial.Serial, speed: int) -> tuple[int, int]:
    """
    Move forward at a given speed.
    """
    send_command(arduino, f"setWheelLeft {speed}")
    send_command(arduino, f"setWheelRight {speed}")
    return (speed, speed)


def turn_right(arduino: serial.Serial, speed: int) -> tuple[int, int]:
    """
    Turn right at given speed.
    """
    send_command(arduino, f"setWheelLeft {speed}")
    send_command(arduino, f"-setWheelRight {speed}")
    return (speed, -speed)


def turn_left(arduino: serial.Serial, speed: int) -> tuple[int, int]:
    """
    Turn left at given speed.
    """
    send_command(arduino, f"-setWheelLeft {speed}")
    send_command(arduino, f"setWheelRight {speed}")
    return (-speed, speed)


def read_sensors(arduino: serial.Serial) -> tuple[int, int]:
    """
    Read sensors.
    """
    return parse_tuple(send_command(arduino, "readSensors"))


def stop_robot(arduino: serial.Serial):
    """
    Stop robot.
    """
    send_command(arduino, "STOP")


def control_cycle(arduino: serial.Serial, speed: int, threshold: int) -> tuple:
    """
    One one cycle of the control loop.

    Returns the sensor readings.
    """
    data = read_sensors(arduino)

    left_over_tape = data[0] > threshold
    right_over_tape = data[1] > threshold

    match (left_over_tape, right_over_tape):
        case (True, True):
            # Both sensors are over tape, we are at start line. Stop!
            speeds = stop_robot(arduino)
        case (False, False):
            # Neither sensor is over tape
            speeds = move_straight(arduino, speed)
        case (True, False):
            # Left sensor is over tape, turn left as sensor is in front
            speeds = turn_left(arduino, speed)
        case (False, True):
            # Right sensor is over tape, turn right as sensor is in front
            speeds = turn_right(arduino, speed)
    return (data, speeds)


def main():
    """
    Main loop, no interaction
    """
    speed = DEFAULT_SPEED
    threshold = DEFAULT_THRESHOLD

    port = guess_port()
    arduino = serial.Serial(port, BAUDRATE,
                            timeout=ARDUINO_TIMEOUT)
    time.sleep(5)

    while True:
        control_cycle(arduino, speed, threshold)


if __name__ == "__main__":
    main()
```
: Complete source listing for `control_loop.py`. {#lst:Control.py}

```python
"""
Tkinter based GUI for control loop
"""

# Standard library stuff
import threading
from threading import Thread
import queue
import time
import tkinter as tk

# PyPI
import serial

# Plotting
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk, )
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

# Custom Modules
from arduino import (guess_port,
                     BAUDRATE,
                     ARDUINO_TIMEOUT, )
from control_loop import (control_cycle,
                          stop_robot,
                          DEFAULT_SPEED,
                          DEFAULT_THRESHOLD, )


class SensorData:
    """
    Class to store the sensor data between threads.
    """
    # Don't care about locking or race conditions because this isn't for anything
    # computationally important, just for visualizing the current sensor readings.

    def __init__(self, data: tuple[int, int]):
        self.data = data
        self.history = [(0, 0)]
        self.speeds = [(0, 0)]

    def update(self, data, speeds):
        self.history.append(data)
        self.speeds.append(speeds)
        self.data = data


class RobotGui:
    """
    Class for GUI part of Robot. This is the producer.
    """
    # Be ware ye who enter, this is undoubtedly the worst code I have ever
    # written in my life

    def __init__(self, commands: queue.Queue, sensors: SensorData):
        """
        Initializse GUI
        """
        # Communicate with other threads
        self.commands = commands
        self.sensors = sensors

        # Setup root window for GUI
        self.root = tk.Tk()
        self.root.title("Line Follower GUI")

        # Plotting
        self.fig = Figure(figsize=(6,6), dpi=100)
        self.ax1, self.ax2 = self.fig.subplots(2, 1)
        self.sensor_left_line, = self.ax1.plot([], [], "r-",
                                              label="Left Sensor")
        self.sensor_right_line, = self.ax1.plot([], [], "b-",
                                               label="Right Sensor")
        self.speed_left_line, = self.ax2.plot([], [], "y-",
                                             label="Left Motor Speed")
        self.speed_right_line, = self.ax2.plot([], [], "g-",
                                             label="Right Motor Speed")
        self.ax1.set_xlabel("Sample Number")
        self.ax1.set_ylabel("Sensor Reading")
        self.ax2.set_xlabel("Sample Number")
        self.ax2.set_ylabel("Motor Speed")
        self.fig.tight_layout()
        self.fig.legend(loc="lower right")

        # Matplotlib Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()

        # Matplotlib Toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root,
                                            pack_toolbar=False)
        self.toolbar.update()

        # Create and Pack the GUI
        start = tk.Button(self.root,
                          text="Start Following",
                          command=self.start)
        start.pack()

        stop = tk.Button(self.root,
                         text="Stop Following",
                         command=self.stop)
        stop.pack()

        speed = tk.Scale(self.root,
                         from_=0, to=255,
                         orient=tk.HORIZONTAL,
                         length=512,
                         showvalue=tk.TRUE,
                         command=self.set_speed)
        speed.set(DEFAULT_SPEED)
        speed.pack()

        threshold = tk.Scale(self.root,
                             from_=0, to=1023,
                             orient=tk.HORIZONTAL,
                             length=512,
                             showvalue=tk.TRUE,
                             command=self.set_threshold)
        threshold.set(DEFAULT_THRESHOLD)
        threshold.pack()

        self.canvas.get_tk_widget().pack()
        self.toolbar.pack()
        self.update_plot_job = self.root.after(500, self.plot_graph)

        self.sensor_status = tk.Label(self.root, text="foobar")
        self.sensor_status.pack()
        self.update_job = self.root.after(50, self.update_sensors)

    def run(self):
        """
        Run GUI
        """
        self.root.mainloop()

    def add_to_queue(self, command: str):
        """
        Add command to queue.

        Commands in Queue are strings.
        """
        self.commands.put(command)

    def plot_graph(self):
        """
        Update plot.
        """
        try:
            sensors = self.sensors.history
            sensor_left, sensor_right = zip(*sensors)
        except:
            sensor_left = []
            sensor_right = []

        try:
            speeds = self.sensors.speeds
            speed_left, speed_right = zip(*speeds)
        except:
            speed_left = []
            speed_right = []

        samples = list(range(len(sensor_left)))

        self.sensor_left_line.set_data(samples, sensor_left)
        self.sensor_right_line.set_data(samples, sensor_right)
        self.speed_left_line.set_data(samples, speed_left)
        self.speed_right_line.set_data(samples, speed_right)

        self.ax1.set_xlim(0, max(samples)*1.1)
        self.ax2.set_xlim(0, max(samples)*1.1)

        self.ax1.set_ylim(min(min(sensor_left), min(sensor_right))
                         *1.05,
                         max(max(sensor_left), max(sensor_right))
                         *1.05)
        self.ax2.set_ylim(min(min(speed_left), min(speed_right))
                         *1.05,
                         max(max(speed_left), max(speed_right))
                         *1.05)
        self.canvas.draw()

        self.update_plot_job = self.root.after(500, self.plot_graph)

    def start(self):
        """
        Add START command.
        """
        self.add_to_queue("START")

    def stop(self):
        """
        Add STOP command.
        """
        self.add_to_queue("STOP")

    def set_speed(self, speed):
        """
        Add SPEED N command.
        """
        self.add_to_queue(f"SPEED {int(speed)}")

    def set_threshold(self, threshold):
        """
        Add THRESHOLD N command
        """
        self.add_to_queue(f"THRESHOLD {int(threshold)}")

    def update_sensors(self):
        """
        Update sensor data wigit
        """
        data = self.sensors.data
        # A tuple of (-1, -1) for sensor data indicates program is shutting down
        if data != (-1, -1):
            self.sensor_status.configure(text=f"{data}")
            self.update_job = self.root.after(50, self.update_sensors)


def run_robot(arduino: serial.Serial,
              commands: queue.Queue,
              sensors: SensorData, ):
    """
    Process commands in queue and send to Arduino.
    """
    moving = False
    speed = DEFAULT_SPEED
    threshold = DEFAULT_THRESHOLD
    while True:
        if not commands.empty():
            command = commands.get()
            # Pattern matching, we require Python 3.10
            match command.split():
                case ["START"]:
                    print("Starting Robot")
                    moving = True
                case ["STOP"]:
                    print("Stopping Robot")
                    moving = False
                case ["DONE"]:
                    print("Stopping Robot")
                    stop_robot(arduino)
                    break
                case ["SPEED", val]:
                    print(f"Setting Speed to {val}")
                    speed = int(val)
                case ["THRESHOLD", val]:
                    print(f"Setting Sensor Threshold to {val}")
                    threshold = int(val)
        if moving:
            data, speeds = control_cycle(arduino, speed, threshold)
            sensors.update(data, speeds)
        else:
            stop_robot(arduino)


def main():
    # Connect to Arduino
    try:
        port = guess_port()
        arduino = serial.Serial(port, BAUDRATE,
                                timeout=ARDUINO_TIMEOUT)
        print(f"Waiting for {port}")
        time.sleep(3)
        print("Ready. Starting GUI.")
    except:
        import sys
        print("Error: No Arduino Found.")
        sys.exit(-1)

    # Initialize variables
    commands = queue.Queue()
    sensors = SensorData((0, 0))
    gui = RobotGui(commands, sensors)

    # Start thread to process tasty data
    consumer = Thread(target=run_robot,
                      args=[arduino, commands, sensors],
                      daemon=True)
    consumer.start()

    # Start GUI
    gui.run()

    # Wait for queue to finish. Add a DONE command.
    commands.put("DONE")
    consumer.join()

    # Clean up
    print(f"Closing {port}")
    arduino.close()
    time.sleep(1)

    print("Thank you for using our line follower!")


if __name__ == "__main__":
    main()
```
: Complete source listing for `robot_gui.py`. {#lst:GUI.py}
