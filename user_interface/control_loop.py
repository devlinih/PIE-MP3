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
    send_command(arduino, "setWheelRight 0")


def turn_right(arduino: serial.Serial, speed: int):
    """
    Turn right at given speed.
    """
    send_command(arduino, "setWheelLeft 0")
    send_command(arduino, f"setWheelRight {speed}")


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

    if data[0] > threshold:
        turn_left(arduino, speed)
    if data[1] > threshold:
        turn_right(arduino, speed)
    else:
        move_straight(arduino, speed)
    return data



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
        control_cycle(arduino, speed, threshold)


if __name__ == "__main__":
    main()
