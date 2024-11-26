import sensor.motor as motor
import sensor.MPU as mpu
from utils import normalize
from constant import MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ
from time import *

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
        * ----------> (x)
        (z)
"""

class Drone:
    def __init__(self):
        mpu.select()
        mpu.init()
        motor.select()
        motor.arm_all()
        self.set_gyro_bias()
        print(f"Gyro Bias: x: {self.gyro_bias_x}, y: {self.gyro_bias_y}, z: {self.gyro_bias_z}")

    def set_gyro_bias(self):
        gxs:list[float] = []
        gys:list[float] = []
        gzs:list[float] = []
        started_at_ticks_ms:int = time.ticks_ms()
        while ((time.ticks_ms() - started_at_ticks_ms) / 1000) < 3.0:
            gyro_x, gyro_y, gyro_z = mpu.read_gyro()
            gyro_y = gyro_y * -1    # gyro sensor is flipped upside down
            gyro_z = gyro_z * -1    # gyro sensor is flipped upside down
            gxs.append(gyro_x)
            gys.append(gyro_y)
            gzs.append(gyro_z)
            time.sleep(0.025)
        self.gyro_bias_x = sum(gxs) / len(gxs)
        self.gyro_bias_y = sum(gys) / len(gys)
        self.gyro_bias_z = sum(gzs) / len(gzs)


    def read_accelerometer(self) -> tuple[float, float, float]:
        mpu.select()
        return mpu.read_accelerometer()


    def read_gyro(self)-> tuple[float, float, float]:
        mpu.select()
        gyro_x, gyro_y, gyro_z = mpu.read_gyro()
        gyro_y = gyro_y * -1    # gyro sensor is flipped upside down
        gyro_z = gyro_z * -1    # gyro sensor is flipped upside down
        gyro_x -= self.gyro_bias_x
        gyro_y -= self.gyro_bias_y
        gyro_z -= self.gyro_bias_z
        return gyro_x, gyro_y, gyro_z


    def start_motors(self, motor_to_speed_dict: dict[int, float]): 
        motor.select()
        for motor_channel in motor_to_speed_dict:
            speed_percentage = motor_to_speed_dict.get(motor_channel)
            motor.start_motor(motor_channel, normalize(speed_percentage, 0, 100, MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ))
