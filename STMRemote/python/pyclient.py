import socket
import sys

s = socket.socket()
host = sys.argv[1]
port = 12345

s.connect((host, port))
while True:
	print(s.recv(1024))

s.close
