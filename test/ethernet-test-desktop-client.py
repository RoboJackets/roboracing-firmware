from __future__ import print_function
import socket
import signal
import sys

# Evan's laptop works with: IP whatever, netmask 255.255.255.0, gateway 0.0.0.0

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    s.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

ECHO_SERVER_ADDRESS = "192.168.2.2"
ECHO_SERVER_PORT = 7
message = b'Hello, world'

print("setting up socket...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("connecting socket...")
s.connect((ECHO_SERVER_ADDRESS, ECHO_SERVER_PORT))

print('Sending', str(message))
s.sendall(message)
data = s.recv(1024)
s.close()
print('Received', str(data))
