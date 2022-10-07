"""
Tkinter based GUI for control loop
"""

import threading
from threading import Thread
import queue
import serial
import time
import tkinter as tk

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
                         from_ = 0, to = 1000,
                         orient = tk.HORIZONTAL,
                         showvalue = tk.TRUE,
                         command = self.set_speed)
        speed.pack()

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
        Add SPEED command.
        """
        self.add_to_queue(f"SPEED {int(speed)}")


def consume_commands(arduino: serial.Serial,
                     commands: queue.Queue,
                     finished: queue.Queue, ):
    """
    Process commands in queue and send to Arduino.
    """
    while True:
        if not commands.empty():
            command = commands.get()
            print(command)
            if command == "DONE":
                break



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
    consumer = Thread(target=consume_commands,
                      args=[arduino, commands, finished],
                      daemon=True)
    consumer.start()

    # Start GUI
    gui.run()

    # Wait for queue to finish. Add a DONE command.
    commands.put("DONE")
    finished.join()

    print("Thank you for using our line follower!")


if __name__ == "__main__":
    main()
