#!/usr/bin/env python3
"""Answer MSI's clock probes with this machine's Unix time. Standard library only."""
import socket
import struct
import time

PORT = 5603

server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server.bind(("0.0.0.0",PORT))
while True:
    _,sender = server.recvfrom(64)
    server.sendto(struct.pack("<q",time.time_ns()),sender)
