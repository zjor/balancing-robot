#!/usr/bin/env python3

import asyncio
import datetime
import random
import websockets

from mpu import MPU

async def time(websocket, path):
    while True:
        roll, pitch = mpu.get_roll_pitch_deg()
        await websocket.send(f"{roll:.2f},{pitch:.2f}")
        await asyncio.sleep(0.1)

mpu = MPU()

start_server = websockets.serve(time, '0.0.0.0', 5678)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()