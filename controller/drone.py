import controller.sensor.motor as motor
import controller.sensor.MPU as mpu
from controller.utils import normalize
from controller.constant import MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ
"""
FRONT
       1        2
     +---+    +---+
     | o |    | o |
     +---+    +---+
        \      /
         \    /
          \  /
           ||
       +-------+
       | DRONE |
       | BODY  |
       | FROM  |
       | ABOVE |
       +-------+
           ||
          /  \
         /    \
        /      \
     +---+    +---+
     | o |    | o |
     +---+    +---+
       3        4
BACK
"""


def init():
    mpu.select()
    mpu.init()
    motor.select()
    motor.arm_all()


def read_accelerometer() -> tuple[float, float, float]:
    mpu.select()
    return mpu.read_accelerometer()


def read_gyro()-> tuple[float, float, float]:
    mpu.select()
    return mpu.read_gyro()


def start_motors(motor_to_speed_dict: dict[int, float]): 
    motor.select()
    for motor_channel in motor_to_speed_dict:
        speed_percentage = motor_to_speed_dict.get(motor_channel)
        motor.start_motor(motor_channel, normalize(speed_percentage, 0, 100, MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ))