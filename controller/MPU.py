from pynq_i2c import pynq_i2c_instance
from time import sleep
from controller.constant import *
from utils import TCA_channel_select

def MPU_select():
     TCA_channel_select(1)

# https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
def MPU_Init():
    pynq_i2c_instance.iic_writeByte(MPU_DEVICE_ADDRESS, MPU_SMPLRT_DIV, 7)
    sleep(100e-3)
    pynq_i2c_instance.iic_writeByte(MPU_DEVICE_ADDRESS, MPU_PWR_MGMT_1, 1)
    sleep(100e-3)
    pynq_i2c_instance.iic_writeByte(MPU_DEVICE_ADDRESS, MPU_CONFIG, 0)
    sleep(100e-3)
    pynq_i2c_instance.iic_writeByte(MPU_DEVICE_ADDRESS, MPU_GYRO_CONFIG, 24)
    sleep(100e-3)
    pynq_i2c_instance.iic_writeByte(MPU_DEVICE_ADDRESS, MPU_INT_ENABLE, 1)
    sleep(100e-3)

def MPU_read_addr(addr):
    #Accelero and Gyro value are 16-bit
    high = pynq_i2c_instance.iic_readByte(MPU_DEVICE_ADDRESS, addr)
    low = pynq_i2c_instance.iic_readByte(MPU_DEVICE_ADDRESS, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

def MPU_read_accel() -> tuple[float, float, float]:
    acc_x = MPU_read_addr(MPU_ACCEL_XOUT_H)
    acc_y = MPU_read_addr(MPU_ACCEL_YOUT_H)
    acc_z = MPU_read_addr(MPU_ACCEL_ZOUT_H)
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    return (acc_x, acc_y, acc_z)


def MPU_read_gyro() -> tuple[float, float, float]:
    gyro_x = MPU_read_addr(MPU_GYRO_XOUT_H)
    gyro_y = MPU_read_addr(MPU_GYRO_YOUT_H)
    gyro_z = MPU_read_addr(MPU_GYRO_ZOUT_H)
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    return (Gx, Gy, Gz)

     