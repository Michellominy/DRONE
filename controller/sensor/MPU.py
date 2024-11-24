# Interface for the MPU6050
from pynq_i2c import pynq_i2c_instance
from time import sleep
from controller.constant import *
import sensor.TCA as tca

def select():
     tca.channel_select(TCA_MOTOR_CHANNEL)

# https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
def init():
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

def read_addr(addr):
    #Accelero and Gyro value are 16-bit
    high = pynq_i2c_instance.iic_readByte(MPU_DEVICE_ADDRESS, addr)
    low = pynq_i2c_instance.iic_readByte(MPU_DEVICE_ADDRESS, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

def read_accelerometer() -> tuple[float, float, float]:
    acc_x = read_addr(MPU_ACCEL_XOUT_H)
    acc_y = read_addr(MPU_ACCEL_YOUT_H)
    acc_z = read_addr(MPU_ACCEL_ZOUT_H)
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    return (acc_x, acc_y, acc_z)


def read_gyro() -> tuple[float, float, float]:
    gyro_x = read_addr(MPU_GYRO_XOUT_H)
    gyro_y = read_addr(MPU_GYRO_YOUT_H)
    gyro_z = read_addr(MPU_GYRO_ZOUT_H)
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    return (Gx, Gy, Gz)

     
