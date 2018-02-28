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
if ack == binascii.unhexlify('AAA055'):
    print "PASS: Got correct ack"
else:
    print "FAIL: Didn't get correct ack " + binascii.hexlify(ack)

# Wait for timeout
result = ser.read(3)

if result == binascii.unhexlify('AAA255'):
    print "PASS: Received expected value " + binascii.hexlify(result)
else:
    print "FAIL: Received " + binascii.hexlify(result) + ", expected value aaa255"

# Reply with ack
ser.write(binascii.unhexlify('AAA055'))

ser.close()
