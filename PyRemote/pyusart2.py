#!/usr/bin/python3

import serial
import time
from numpy import int8

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
line = ser.readline()
print(line)
while True:
    val = ser.readline()
    if(len(val) < 3):
        continue;
    y = int8(val[0])
    z = int8(val[2])
    print(y, '\t', z)
