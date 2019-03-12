import sys
import os
import binascii
import serial
from serial import SerialException
import time
import struct
import click

global MTU
global filesize

jpeg_signatures = [
    binascii.unhexlify(b'FFD8FFD8'),
    binascii.unhexlify(b'FFD8FFE0'),
    binascii.unhexlify(b'FFD8FFE1')
]


def getSize(fileobject):
    fileobject.seek(0,2) # move the cursor to the end of the file
    size = fileobject.tell()
    return size

def check_file_jpg(filename):
    with open(filename, 'rb') as file:
        first_four_bytes = file.read(4)

    if first_four_bytes in jpeg_signatures:
        print("JPEG detected.")
        return 1
    else:
        print("File does not look like a JPEG.")
        sys.exit(0)
        return 0



PAYLOAD_JPG_OPCODE_PING         = b'\xA0'
PAYLOAD_JPG_OPCODE_FILEINFO_RSP = b'\xA1'
PAYLOAD_JPG_OPCODE_MTU_REQ      = b'\xA2'
PAYLOAD_JPG_OPCODE_MTU_RSP      = b'\xA3'
PAYLOAD_JPG_OPCODE_DATA_REQ     = b'\xA4'
PAYLOAD_JPG_OPCODE_DATA_RSP     = b'\xA5'
PAYLOAD_JPG_OPCODE_COMPLETE     = b'\xA6'
PAYLOAD_JPG_OPCODE_COMPLETE_ACK = b'\xA7'
PAYLOAD_JPG_OPCODE_DATA_RSP_LAST  = b'\xA8'

mtu_length      =   "global"
file_offset     =   "global"
file_offset_len =   "global"
file_crc32      =   "global"

def main():

    DEFAULT_MTU_LEN             =   20

    # print ("Number of argument = ", len(sys.argv))
    len_argument = len(sys.argv)
    filesize = 0
    if (len_argument != 2):
      print ("")
      print ("Error - Please provide JPG file!!!")
      print ("")
      print ("Correct Usage:")
      print ("python load_jpg_file.py <jpg_file>")
      print ("")
      sys.exit(0)

    # Open the JPG file
    try:
        binary_file = open(sys.argv[1], 'rb')
        binary_data = binary_file.read()
        binary_crc = binascii.crc32(binary_data)
        filesize = getSize(binary_file)
        print("filesize and CRC", filesize, binary_crc)
    except:
        print ("Fail to open jpg file ", sys.argv[1])
        sys.exit(0)

    # Open the Serial Port
    try:
        ser = serial.Serial('COM43', 115200, timeout=30)
    except:
        print ("Please check to open the correct com port!!")
        sys.exit(0)

    print ("Waiting to get the PING Message from DK Board [Timeout = 30 sec]!!")
    payload_len = 3
    ping_request = ser.read(3)

    print (ping_request)

    if ping_request == '':
          print ("[Error] : Timeout, missing the ping message from the DK board!!")
          sys.exit(0)

    (payload_len, opcode, mtu_len) = struct.unpack('BsB', ping_request)

    print (payload_len, opcode, mtu_len)

    if (opcode != PAYLOAD_JPG_OPCODE_PING):
          print ("[Error]: Incorrect OPCODE PING", opcode)
          sys.exit(0)
    else:
          print ("Get the PING message from device !!!")

    if (mtu_len > 247):
          print ("[Error] MTU Length (", mtu_len, ") is > 247!!")
          sys.exit(0)

    # # Send the filesize
    payload_len = 9                    # Opcode (1) + FileSize(4) + CRC32(4)
    ser.write(struct.pack("B", payload_len))
    ser.write(PAYLOAD_JPG_OPCODE_FILEINFO_RSP)
    #Send the file size
    ser.write(filesize.to_bytes(4, byteorder='little', signed=True))
    #Send File CRC32
    ser.write(binary_crc.to_bytes(4, byteorder='little', signed=False))

    data_index = 0
    offset = 0
    length = mtu_len
    data_index = 0
    offset_addr = 0

    while (data_index < filesize):

        print (offset_addr, length, filesize)

        print ("Waiting for the data request [offset, length] !!!")
        data_request = ser.read(8)
        print (data_request)
        (payload_len, opcode, request_length, offset_addr) = struct.unpack('BsHL', data_request)
        print (payload_len, opcode, "request length = ", request_length, "offset address = ", offset_addr)

        if ((opcode == PAYLOAD_JPG_OPCODE_DATA_REQ) or (opcode == PAYLOAD_JPG_OPCODE_DATA_RSP_LAST)):
            print ("OPCODE ", opcode)
        else:
            print ("Failure on incorrect OPCODE", opcode)
            sys.exit(0)

        if (offset_addr+request_length > filesize):
            print ("Error: the request offset address is outside the filesize!!")
            sys.exit(0)


        data_index = 0
        offset = offset_addr
        length = mtu_len

        while (data_index < request_length):

            if (data_index >= request_length - mtu_len):
                length = request_length - data_index
            else:
                length = mtu_len

            binary_file.seek(offset_addr+data_index, 0)
            print ("offset_addr: ", offset_addr, "data_index ", data_index, "length ", length, "filesize ", filesize)
            payload_len = length + 1
            ser.write(struct.pack("B", payload_len))
            if (data_index+length >= request_length):
                print ("PAYLOAD_JPG_OPCODE_DATA_RSP_LAST: offset_addr", offset_addr, "data_index ",  data_index, "length ", length)
                ser.write(PAYLOAD_JPG_OPCODE_DATA_RSP_LAST)
            else:
                print ("PAYLOAD_JPG_OPCODE_DATA_RSP : offset_addr", offset_addr, "data_index ", data_index, "length",  length)
                ser.write(PAYLOAD_JPG_OPCODE_DATA_RSP)
            ser.write(binary_file.read(length))
            # time.sleep(10)


            data_index = data_index + length

            if (offset_addr+data_index >= filesize):
              sys.exit(0)


    binary_file.close()


if __name__=="__main__":
  main()
