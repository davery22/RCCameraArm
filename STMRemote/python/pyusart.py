#!/usr/bin/python3

import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
line = ser.readline()
print(line)
while True:
	print(ser.readline())
