from __future__ import print_function

import socket


ECHO_SERVER_ADDRESS = "192.168.2.2"
ECHO_SERVER_PORT = 7


print("setting up socket...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("connecting socket...")
s.connect((ECHO_SERVER_ADDRESS, ECHO_SERVER_PORT))

try:
    while True:
        print(s.recv(16))
finally:
    s.close()
