#!/usr/bin/python

import serial
import binascii
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-p", "--port", dest="port", help="serial port to use")

(options, args) = parser.parse_args()

ser = serial.Serial(options.port, 19200, timeout=35);

ser.flush()

learn_cmd = 'AAA155'
ser.write(learn_cmd.decode("hex"))

ack = ser.read(3)
print ack.encode('hex_codec')

# Wait for timeout
ack = ser.read(3)

if ack == binascii.unhexlify('aaa255'):
    print("PASS: Received expected value 0xaaa255")
else:
     print("FAIL: Received " + binascii.hexlify(ack) + ", expected value 0xaaa255")

#print ack.encode('hex_codec')

ser.close()
