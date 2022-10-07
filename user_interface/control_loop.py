"""
Control loop for line following robot.

Right now, it's just hard coded, but adding CLI will be done later as well as
GUI maybe.
"""

import serial
import time

from arduino import (guess_port,
                     send_command,
                     parse_tuple,
                     BAUDRATE,
                     ARDUINO_TIMEOUT,)


def move_straight(arduino: serial.Serial, speed: int):
    """
    Move forward at a given speed.
    """
    adjusted_speed = speed // 2
    send_command(arduino, f"setWheelLeft {adjusted_speed}")
    send_command(arduino, f"setWheelRight {adjusted_speed}")


def turn_left(arduino: serial.Serial, speed: int):
    """
    Turn left at given speed.
    """
    send_command(arduino, f"setWheelLeft {speed}")


def turn_right(arduino: serial.Serial, speed: int):
    """
    Turn right at given speed.
    """
    send_command(arduino, f"setWheelRight {speed}")


def read_sensors(arduino: serial.Serial) -> tuple[int, int]:
    """
    Read sensors.
    """
    return parse_tuple(send_command(arduino, "readSensors"))


def main():
    """
    Main loop, no interaction
    """
    speed = 50
    threshold = 400

    port = guess_port()
    arduino = serial.Serial(port, BAUDRATE,
                            timeout=ARDUINO_TIMEOUT)
    time.sleep(5)

    while True:
        data = read_sensors(arduino)
        if data[0] > threshold:
            turn_left(arduino, speed)
        if data[1] > threshold:
            turn_right(arduino, speed)
        else:
            move_straight(arduino, speed)


if __name__ == "__main__":
    main()
