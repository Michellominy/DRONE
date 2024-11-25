import sensor.motor as motor
import sensor.MPU as mpu
from utils import normalize
from constant import MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ
"""
Positive Pitch: motor 1,3 push more
Positive Yaw: motor 2,3 push more
Positive Roll: motor 3,4 push more

FRONT
       3        1
     +---+    +---+
     | ↺ |    | ↻ |
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
     | ↻ |    | ↺ |
     +---+    +---+
       4        2
BACK
        ^(y)
        |
        |
        |
        |
        |
        o ----------> (x)
        (z)
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