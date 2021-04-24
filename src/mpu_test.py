import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None, 
    bus=1,
    gfs=GFS_1000, 
    afs=AFS_4G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ)

mpu.calibrate()
mpu.configure()

while True:

    print("|.....MPU9250 in 0x68 Address.....|")
    print("Accelerometer", mpu.readAccelerometerMaster())
    print("Gyroscope", mpu.readGyroscopeMaster())
    print("Magnetometer", mpu.readMagnetometerMaster())
    print("Temperature", mpu.readTemperatureMaster())
    print("\n")

    time.sleep(1)