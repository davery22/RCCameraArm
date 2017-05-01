#!/usr/bin/python3
import serial
import time
from numpy import int16

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
line = ser.readline()
print(line)
yy = 0;
zz = 0;
yavg = 0;
zavg = 0;
ycounter = 0;
while True:
    val = ser.readline()
    if len(val) != 14:
        continue

    y = int16((int(val[7])<<8) + int(val[8]))
    z = int16((int(val[11])<<8) + int(val[12]))
    if ycounter < 256:
        ycounter+=1
        yavg += y/256
        zavg += z/256
    else:
        yy += int((y - yavg - (yy>>6)))
        zz += int((z - zavg - (zz>>6)))
        print(yy>>12, "\t", zz>>12)
    print(yavg, '\t', zavg)

