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
            speeds = move_straight(arduino, 0)
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
