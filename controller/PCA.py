from controller.constant import *
from pynq_i2c import pynq_i2c_instance
from time import sleep
import math

# https://github.com/adafruit/Adafruit_Python_PCA9685/blob/master/Adafruit_PCA9685/PCA9685.py
def PCA_Init():    
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, PCA9685_MODE2, PCA9685_OUTDRV)
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, PCA9685_MODE1, PCA9685_ALLCALL)
    sleep(0.005)
    mode1 = pynq_i2c_instance.iic_readByte(PCA9685_ADDRESS, PCA9685_MODE1)
    mode1 = mode1 & ~PCA9685_SLEEP 
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, PCA9685_MODE1, mode1)
    sleep(0.005)
    
    set_pwm_freq(1000)


def set_pwm_freq(freq_hz):
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq_hz)
    prescaleval -= 1.0
    prescale = int(math.floor(prescaleval + 0.5))
    oldmode = pynq_i2c_instance.iic_readByte(PCA9685_ADDRESS, PCA9685_MODE1)
    newmode = (oldmode & 0x7F) | 0x10    # sleep
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, PCA9685_MODE1, newmode)  # go to sleep
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, PCA9685_PRESCALE, prescale)
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, PCA9685_MODE1, oldmode)
    sleep(0.005)
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, PCA9685_MODE1, oldmode | 0x80)

def set_pwm(channel, on, off):
    """Sets a single PWM channel."""
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, LED0_ON_L+4*channel, on & 0xFF)
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, LED0_ON_H+4*channel, on >> 8)
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, LED0_OFF_L+4*channel, off & 0xFF)
    pynq_i2c_instance.iic_writeByte(PCA9685_ADDRESS, LED0_OFF_H+4*channel, off >> 8)

def PCA_motor_to_channel(motor) -> int:
    match motor:
        case 1:
                return 3
        case 2:
                return 4
        case 3:
                return 7
        case 4:
                return 8
    # TODO; handle error