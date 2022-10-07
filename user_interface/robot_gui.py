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

# Custom Modules
from arduino import (guess_port,
                     BAUDRATE,
                     ARDUINO_TIMEOUT, )
from control_loop import (control_cycle,
                          stop_robot, )

# Note: Referenced
# https://www.oreilly.com/library/view/python-cookbook/0596001673/ch09s07.html
# Not sure of licensing


class RobotGui:
    """
    Class for GUI part of Robot. This is the producer.
    """

    def __init__(self, commands: queue.Queue):
        """
        Initializse GUI
        """
        self.commands = commands

        self.root = tk.Tk()
        self.root.title("Line Follower GUI")

        start = tk.Button(self.root,
                          text="Start Following",
                          command=self.start)
        start.pack()

        stop = tk.Button(self.root,
                         text="Stop Following",
                         command=self.stop)
        stop.pack()

        speed = tk.Scale(self.root,
                         from_ = 0, to = 255,
                         orient = tk.HORIZONTAL,
                         showvalue = tk.TRUE,
                         command = self.set_speed)
        speed.set(30)
        speed.pack()

        threshold = tk.Scale(self.root,
                             from_ = 0, to = 1023,
                             orient = tk.HORIZONTAL,
                             showvalue = tk.TRUE,
                             command = self.set_threshold)
        threshold.set(400)
        threshold.pack()

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


def run_robot(arduino: serial.Serial,
              commands: queue.Queue,
              finished: queue.Queue, ):
    """
    Process commands in queue and send to Arduino.
    """
    moving = False
    speed = 30
    threshold = 400
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
                    speed = val
                case ["THRESHOLD", val]:
                    print(f"Setting Sensor Threshold to {val}")
                    threshold = val
        if moving:
            control_cycle(arduino, speed, threshold)
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
    finished = queue.Queue()
    gui = RobotGui(commands)

    # Start thread to process tasty data
    consumer = Thread(target=run_robot,
                      args=[arduino, commands, finished],
                      daemon=True)
    consumer.start()

    # Start GUI
    gui.run()

    # Wait for queue to finish. Add a DONE command.
    commands.put("DONE")
    finished.join()

    # Clean up
    print(f"Closing {port}")
    arduino.close()
    time.sleep(1)

    print("Thank you for using our line follower!")


if __name__ == "__main__":
    main()
