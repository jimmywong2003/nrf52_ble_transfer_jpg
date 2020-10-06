import sys
import os
import binascii
import serial
from serial import SerialException
import time
import struct
import click

from serial.tools.list_ports import comports

try:
    raw_input
except NameError:
    raw_input = input   # in python3 it's "raw"
    unichr = chr

def ask_for_port():
    """\
    Show a list of ports and ask the user for a choice. To make selection
    easier on systems with long device names, also allow the input of an
    index.
    """
    sys.stderr.write('\n--- Available ports:\n')
    ports = []
    for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
        sys.stderr.write('--- {:2}: {:20} {}\n'.format(n, port, desc))
        ports.append(port)
    while True:
        port = raw_input('--- Enter port index or full name: ')
        try:
            index = int(port) - 1
            if not 0 <= index < len(ports):
                sys.stderr.write('--- Invalid index!\n')
                continue
        except ValueError:
            pass
        else:
            port = ports[index]
        return port

def main():

    port_select = ask_for_port()
    print("You select the port number " + port_select)

    # Open the Serial Port
    try:
        ser = serial.Serial(port_select, 115200, timeout=30)
    except:
        print ("Please check to open the correct com port!!")
        sys.exit(0)

    ser.close()

if __name__=="__main__":
  main()
