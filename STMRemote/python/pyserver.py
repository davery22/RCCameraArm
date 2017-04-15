#!/usr/bin/python3

import socket

s = socket.socket()
port = 12345
s.bind(('',port))

s.listen(5)
while True:
    c, addr = s.accept()
    print('Got connection from', addr)
    while True:
        #print(c.recv(1024))
        data = c.recv(1024)
        if not data: 
            break
        print(data)
    c.close()
