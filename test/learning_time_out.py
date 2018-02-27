#!/usr/bin/python

import serial

ser = serial.Serial('/dev/cu.usbserial-00000000', 19200, timeout=1);
print(ser.name)
ser.close()
