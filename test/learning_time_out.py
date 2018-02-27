#!/usr/bin/python

import serial
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-p", "--port", dest="port", help="serial port to use")

(options, args) = parser.parse_args()

ser = serial.Serial(options.port, 19200, timeout=1);


print(ser.name)
ser.close()
