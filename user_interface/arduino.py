"""
Connect to an Arduino running PIE MP3 Firmware.
"""

import ast
import serial
import serial.tools.list_ports as list_ports

BAUDRATE = 9600
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
