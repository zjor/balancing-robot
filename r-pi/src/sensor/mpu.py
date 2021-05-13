from math import atan2, pi, degrees
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250


class MPU:
    def __init__(self, calibrate=False):
        self.mpu = MPU9250(
            address_ak=AK8963_ADDRESS, 
            address_mpu_master=MPU9050_ADDRESS_68,
            address_mpu_slave=None, 
            bus=1,
            gfs=GFS_1000, 
            afs=AFS_4G, 
            mfs=AK8963_BIT_16, 
            mode=AK8963_MODE_C100HZ)

        if calibrate:
            # self.mpu.calibrate()
            self.mpu.calibrateMPU6500()
        self.mpu.configure()
        self.mpu.writeMaster(CONFIG, 0x04)


    def get_roll_pitch(self):
        x, y, z = self.mpu.readAccelerometerMaster()
        roll = atan2(x, z)
        pitch = atan2(y, z)
        return roll, pitch


    def get_roll_pitch_deg(self):
        roll, pitch = self.get_roll_pitch()
        return degrees(roll), degrees(pitch)

        