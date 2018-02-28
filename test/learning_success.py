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

# Wait for ack
ack = ser.read(3)
if ack == binascii.unhexlify('AAA055'):
    print "PASS: Got correct ack"
else:
    print "FAIL: Didn't get correct ack " + binascii.hexlify(ack)

print "Push a remote key within 30 s"

result = ser.read(12)

if result[0:2] == binascii.unhexlify('AAA3') and result[-1] == binascii.unhexlify('55'):
    print("PASS: Received expected value 0xAAA355")
else:
     print("FAIL: Received " + binascii.hexlify(result[0:2]) + binascii.hexlify(result[-1]) + ", expected value 0xAAA355")

# Reply with result
ser.write(binascii.unhexlify('AAA055'))

ser.close()
