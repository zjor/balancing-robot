import time
from math import atan2, pi, degrees
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250


def get_roll_pitch(mpu):
    x, y, z = mpu.readAccelerometerMaster()
    roll = atan2(x, z)
    pitch = atan2(y, z)
    return roll, pitch


if __name__ == "__main__":
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS, 
        address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
        address_mpu_slave=None, 
        bus=1,
        gfs=GFS_1000, 
        afs=AFS_4G, 
        mfs=AK8963_BIT_16, 
        mode=AK8963_MODE_C100HZ)

    mpu.configure()

    while True:
    	r, p = get_roll_pitch(mpu)
    	print(f"roll: {degrees(r):6.2f}; pitch: {degrees(p):6.2f}", end='\r')

    	time.sleep(0.1)

