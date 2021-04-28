#!/usr/bin/env python3

import time
import socket

# from sensor.mpu import MPU
from util.timed_task import TimedTask

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

pitch = 0.0
roll = 0.0


def run(now, dt):
    global pitch, roll
    pitch += 0.1
    roll -= 0.1
    message = bytes(f"{pitch} {roll}", "utf-8")
    server.sendto(message, ('<broadcast>', 37020))
    print(f"{now}: {message}")


tt = TimedTask(delay=0.1, run=run)
if __name__ == "__main__":
    try:
        while True:
            tt.loop()
    except KeyboardInterrupt:
        pass
