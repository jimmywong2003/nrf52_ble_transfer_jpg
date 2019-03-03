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

# MTU = 50

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
#
# def check_file_provided():
#   # This method ensures a valid file was provided to the invoked script ##
#   if (len(sys.argv) < 2):
#     print ("")
#     print ("Error - JPG file was provided")
#     print ("")
#     print ("Correct Usage:")
#     print ("python serial_example.py <jpg_file>")
#     print ("")
#     sys.exit(0)

      # if not os.path.isfile(sys.argv[1]):
      #   print ("")
      #   print ("Error - The file provided does not exist")
      #   print ("")
      #
      #   # load_jpg(sys.argv[1])
      # if check_file_jpg(sys.argv[1]):
      #   print ("This is a jpg file")
      #   binary_file = open(sys.argv[1], "r+b")


# def CRCtoSTR(self,CRC32):
#     # Split 4 bytes
#     mask = 0xFF000000;   # 4 bytes mask
#     n = 0
#     buffer = [0,0,0,0] # CRC buffer
#     for i in range(self.CRC_NBYTES,0,-1): # Split the 4 bytes
#         byte_chunkn = CRC32 & mask>>((i-1)*8) # Shift the bits
#         byte1 = byte_chunkn>>8*n        # Get the LSBs
#         # print byte1, chr(byte1)
#         buffer[i-1] = byte1
#         n+=1
#
#     CRC_string = ""
#     for byte in buffer:
#         CRC_string += str(chr(byte))
#
#     print(CRC_string)
#     return CRC_string


# hello.py


# @click.command()
# @click.argument('filename', type=click.Path(exists=True, readable=True), nargs=1)
# def hello(filename):
#     click.echo(filename)


PAYLOAD_JPG_OPCODE_PING         = b'\xA0'
PAYLOAD_JPG_OPCODE_FILEINFO_RSP = b'\xA1'
PAYLOAD_JPG_OPCODE_MTU_REQ      = b'\xA2'
PAYLOAD_JPG_OPCODE_MTU_RSP      = b'\xA3'
PAYLOAD_JPG_OPCODE_DATA_REQ     = b'\xA4'
PAYLOAD_JPG_OPCODE_DATA_RSP     = b'\xA5'
PAYLOAD_JPG_OPCODE_COMPLETE     = b'\xA6'
PAYLOAD_JPG_OPCODE_COMPLETE_ACK = b'\xA7'
PAYLOAD_JPG_OPCODE_DATA_RSP_LAST  = b'\xA8'

# file_size       =   "global"
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
        ser = serial.Serial('COM7', 115200, timeout=30)
    except:
        print ("Please check to open the correct com port!!")
        sys.exit(0)

    # ping_request = ser.read(1)
    # if (ping_request != PAYLOAD_JPG_OPCODE_PING):
    #      print ("Warning: Can't wait for the ping message from DK board")
    #
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
        print (payload_len, opcode, request_length, offset_addr)

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
            print (offset_addr, data_index, length, filesize)
            payload_len = length + 1
            ser.write(struct.pack("B", payload_len))
            if (data_index+length >= request_length):
                print ("PAYLOAD_JPG_OPCODE_DATA_RSP_LAST", offset_addr, data_index, length)
                ser.write(PAYLOAD_JPG_OPCODE_DATA_RSP_LAST)
            else:
                print ("PAYLOAD_JPG_OPCODE_DATA_RSP", offset_addr, data_index, length)
                ser.write(PAYLOAD_JPG_OPCODE_DATA_RSP)
            ser.write(binary_file.read(length))
            # time.sleep(10)


            data_index = data_index + length

            if (offset_addr+data_index >= filesize):
              sys.exit(0)



    return

    # print ("Waiting for the MTU Request")
    # mtu_response = ser.read(3)
    # print (mtu_response)
    # (mtu_response_len, mtu_opcode, mtu_len) = struct.unpack('3B', mtu_response)
    # print (mtu_response_len, mtu_opcode, mtu_len)
    #
    # # Send the MTU ACK Response
    # ser.write(struct.pack("B", payload_len))
    # ser.write(PAYLOAD_JPG_OPCODE_FILEINFO_RSP)

    # receive_cmd = ser.read(200)
    # print (receive_cmd)
    # for value in receive_cmd.split():
    #     print (value)
    #
    # receive_cmd = ser.read(200)
    # print (receive_cmd)

    return

    # PAYLOAD_
    # PAYLOAD_JPG_HEADER_FILESIZE  = b'\xAA'
    # PAYLOAD_JPG_HEADER_CRC32     = b'\xAB'
    #
    # PAYLOAD_JPG_HEADER_DATA_FIRST_RPN = b'\xAC'
    # PAYLOAD_JPG_HEADER_DATA     = b'\xAD'
    # PAYLOAD_JPG_HEADER_MTU_RSP  = b'\xAE'
    # PAYLOAD_JPG_HEADER_DONE_RSP = b'\xAF'
    # PAYLOAD_JPG_HEADER_RPN_RSP  = b'\xB0'
    # PAYLOAD_JPG_HEADER_DATA_RSQ  = b'\xB1'

        # PAYLOAD_JPG_HEADER_FILESIZE = 0xAA,
        # PAYLOAD_JPG_HEADER_CRC32    = 0xAB,
        # PAYLOAD_JPG_HEADER_DATA_FIRST_RPN = 0xAC,
        # PAYLOAD_JPG_HEADER_DATA     = 0xAD,
        # PAYLOAD_JPG_HEADER_MTU_RSP  = 0xAE,
        # PAYLOAD_JPG_HEADER_DONE_RSP = 0xAF,
    # global filesize

  # if (len(sys.argv) < 2):
  #   print ("")
  #   print ("Error - JPG file was provided")
  #   print ("")
  #   print ("Correct Usage:")
  #   print ("python load_jpg_file.py <jpg_file>")
  #   print ("")
  #   sys.exit(0)

  # print (len(sys.argv))
  # sys.exit(0)

  # if not os.path.isfile(sys.argv[1]):
  #   print ("")
  #   print ("Error - The file provided does not exist")
  #   print ("")
  #
  # if check_file_jpg(sys.argv[1]):

def test():

    try:
        binary_file = open(sys.argv[1], 'rb')
        binary_data = binary_file.read()
        binary_crc = binascii.crc32(binary_data)
        filesize = getSize(binary_file)
        print("filesize and CRC", filesize, binary_crc)
    except:
        print ("Fail to open jpg file ", sys.argv[1])
        sys.exit(0)

    # crc_string = CRCtoSTR(binary_crc)
    # print(crc_string)
    try:
        ser = serial.Serial('COM7', 115200, timeout=30)
    except:
        print ("Please check to open the correct com port!!")
        sys.exit(0)

    # waitCount = 0
    mtu_length = DEFAULT_MTU_LEN
    MTU_request = ser.read(2)
    print (MTU_request)
    b_mtu = bytearray(MTU_request)

    mtu_length = b_mtu[1]
    print (b_mtu[0])
    print (b_mtu[1])

    # if (b_mtu[0] != PAYLOAD_JPG_HEADER_MTU_RSP):
    #     print ("Warning: Can't wait for the ping message from DK board")
    #     return


    # # Send the filesize
    payload_len = 6;
    number_of_prn = DEFAULT_PRN # Packet Reciept Notification
    #ser.write(b'\x05')
    # a = struct.pack("B", payload_len);
    # print (a)

    ser.write(struct.pack("B", payload_len))
    ser.write(PAYLOAD_JPG_HEADER_FILESIZE)
    ser.write(filesize.to_bytes(4, byteorder='little', signed=True))
    ser.write(struct.pack("B", number_of_prn))

    # ser.write(PAYLOAD_FORMAT_FILE_INFO)
    # print(binary_crc.to_bytes(4, byteorder='big', signed=True))
    #
    # # send the CRC
    # ser.write(binary_crc.to_bytes(4, byteorder='big', signed=True))


    offset = 0
    length = 0
    count = 0
    prn_index = DEFAULT_PRN

    # filesize = 3000
    while (offset < filesize):
        if (prn_index == 0):
            time.sleep(0.4)
            # prn_ack = ser.read(1)
            # if (prn_ack == PAYLOAD_JPG_HEADER_RPN_RSP):
            prn_index = number_of_prn
        else:
            prn_index = prn_index - 1

        if (offset >= filesize - mtu_length):
            length = filesize - offset
        else:
            length = mtu_length

        binary_file.seek(offset, 0)

        print (offset, length)
        payload_len = length + 1
        ser.write(struct.pack("B", payload_len));
        ser.write(PAYLOAD_JPG_HEADER_DATA)
        ser.write(binary_file.read(length))
        count = count + 1
        # if (count % 6 == 0):
        #     time.sleep(0.2)
        # time.sleep(0.01)
        offset = offset + length



    # binary_file.seek(32,0)
    # file_data = binary_file.read(32)

    # print(bytearray(file_data))
    # for value in bytearray(file_data):
    #     print(hex(value))

    binary_file.close()


if __name__=="__main__":
  main()
