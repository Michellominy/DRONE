import time
import logging

from common.zeroMQManager import zeroMQServer
import controller.sensor.motor as motor
import controller.sensor.MPU as mpu
from controller.utils import normalize
from controller.constant import MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ

MAX_ALLOWED_MOTOR_PERCENTAGE = 0.3

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
        self.logger = logging.getLogger()
        self.last_gyro_x, self.last_gyro_y, self.last_gyro_z = 0.0, 0.0, 0.0
        self.gyro_reading_fail = 0
        self.gyro_max_consecutive_attempt = 10
        self.motor_fail = 0
        self.motor_max_consecutive_fail = 5
        self.logger.info("Initialising MPU")
        mpu.select()
        mpu.init()
        self.set_gyro_bias()
        motor.select()
        self.logger.info("Arming Motor")
        motor.arm_all()
        self.logger.info(f"Gyro Bias: x: {self.gyro_bias_x}, y: {self.gyro_bias_y}, z: {self.gyro_bias_z}")
        
    def set_gyro_bias(self):
        gxs:list[float] = []
        gys:list[float] = []
        gzs:list[float] = []
        started_at_ms = time.time() * 1000  # Start time in milliseconds
        while ((time.time() * 1000 - started_at_ms) / 1000) < 3.0:            
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
        try:
            mpu.select()
            gyro_x, gyro_y, gyro_z = mpu.read_gyro()
            gyro_y = gyro_y * -1    # gyro sensor is flipped upside down
            gyro_z = gyro_z * -1    # gyro sensor is flipped upside down
            gyro_x -= self.gyro_bias_x
            gyro_y -= self.gyro_bias_y
            gyro_z -= self.gyro_bias_z
            self.last_gyro_x, self.last_gyro_y, self.last_gyro_z = gyro_x, gyro_y, gyro_z
            self.gyro_reading_fail = 0
        except Exception as e:
            self.logger.error(f"Error when reading gyro: {e}")
            gyro_x, gyro_y, gyro_z = self.last_gyro_x, self.last_gyro_y, self.last_gyro_z
            self.gyro_reading_fail += 1
            if self.gyro_reading_fail >= self.gyro_max_consecutive_attempt:
                raise Exception("Maximum consecutive gyro reading attempts reached")
        
        zeroMQServer.send("gyro", {"x": gyro_x, "y": gyro_y, "z": gyro_z})
        return gyro_x, gyro_y, gyro_z


    def start_motors(self, motor_to_speed_dict: dict[int, float]): 
        try:
            motor.select()
            for motor_channel in motor_to_speed_dict:
                speed_percentage = self.clamp_motor_percentage(motor_to_speed_dict.get(motor_channel))
                motor.start_motor(motor_channel, normalize(speed_percentage, 0, 1, MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ))
            self.motor_fail = 0  # Reset the fail counter on success
        except Exception as e:
            self.logger.error(f"Error when starting motor: {e}")
            self.motor_fail += 1
            if self.motor_fail >= self.motor_max_consecutive_fail:
                raise Exception("Maximum consecutive motor start attempts reached")
    
    def clamp_motor_percentage(self, speed_percentage):
        if speed_percentage > MAX_ALLOWED_MOTOR_PERCENTAGE:
            return MAX_ALLOWED_MOTOR_PERCENTAGE
        elif speed_percentage < 0:
            return 0
        return speed_percentage
                
    
    def turn_off_all(self):
        while True:
            try:
                motor.select()
                motors = [1,2,3,4]
                for motor_channel in motors:
                    motor.turn_off(motor_channel)
                break
            except BaseException as e:
                self.logger.error(f"Error when trying to turn off drone : {e}")
