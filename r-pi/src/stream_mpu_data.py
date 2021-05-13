#!/usr/bin/env python3

import time
import socket

from sensor.mpu import MPU
from util.timed_task import TimedTask

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)



def run(now, dt):
    global mpu
    roll, pitch = mpu.get_roll_pitch_deg()
    message = bytes(f"{pitch} {roll}", "utf-8")
    server.sendto(message, ('<broadcast>', 37020))
    # print(f"{now}: {message}")

mpu = MPU()
tt = TimedTask(delay=0.05, run=run)
if __name__ == "__main__":

    print("Broadcasting MPU data...")
    try:
        while True:
            tt.loop()
    except KeyboardInterrupt:
        pass
    print("Done")
