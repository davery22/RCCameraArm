#!/usr/bin/python3

import socket
import sys
import serial

s = socket.socket()
host = 'keeton-rpi2.uconnect.utah.edu'
port = 8089
s.connect((host, port))

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
while True:
	s.send(ser.readline())

s.close
