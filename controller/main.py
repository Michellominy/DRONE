from pynq.overlays.base import BaseOverlay
from pynq import Overlay
from time import time, perf_counter, sleep
import traceback
import logging
import sys
import os
import json

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.zeroMQManager import zeroMQServer, ZeroMQSubscriber
from controller.drone import Drone
from controller.pid import PID
from common.constants import BASE_STATION_IP

zeroMQServer.setLogger()
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# https://timhanewich.medium.com/how-i-developed-the-scout-flight-controller-part-7-full-flight-controller-code-4269c83b3b48

# Max attitude rate of change rates (degrees per second)
max_rate_roll:float = 3.0
max_rate_pitch:float = 3.0 
max_rate_yaw:float = 5.0 

target_cycle_hz:float = 15
cycle_time_seconds:float = 1.0 / target_cycle_hz
cycle_time_us:int = int(round(cycle_time_seconds * 1000000, 0)) 

throttle_idle:float = 0.2 # the minumum throttle needed to apply to the four motors for them all to spin up, but not provide lift (idling on the ground). 
throttle_max:float = 0.30
throttle_range:float = throttle_max - throttle_idle

roll_pid = PID(kp=0.0, ki=0.0, kd=0.0, i_limit=150.0, cycle_time_seconds=cycle_time_seconds)
yaw_pid = PID(kp=0.0, ki=0.0, kd=0.0, i_limit=150.0, cycle_time_seconds=cycle_time_seconds)
pitch_pid = PID(kp=0.004, ki=0.0001, kd=0.0, i_limit=25.0, cycle_time_seconds=cycle_time_seconds) # kp [0.002, 0.005]
                                                                                                  # i_limit start low, Increase the throttle and observe if the drone stabilizes well. 
                                                                                                  # If it appears sluggish or takes too long to correct an error, increase i_limit


input("Press key to start ...")
logger.info("Initializing Drone")
drone = Drone()

input_throttle:float = 0.0  # between 0.0 and 1.0
input_pitch:float = 0.0 # between -1.0 and 1.0
input_roll:float = 0.0  # between -1.0 and 1.0
input_yaw:float = 0.0   # between -1.0 and 1.0


def update_input_throttle(json_message):
    global input_throttle
    message = json.loads(json_message.decode())
    if (input_throttle >= 0):
        input_throttle = message["data"]

input_throttle_subscriber = ZeroMQSubscriber(host=BASE_STATION_IP, topic="throttle", callback=update_input_throttle)
    
try:
    while True:
        loop_start = perf_counter()

        adj_throttle:float = throttle_idle + (throttle_range * input_throttle)
        logger.info(f"Adjusted Throttle: {adj_throttle}")
        
        gyro_x, gyro_y, gyro_z = drone.read_gyro()
        # logger.info(f"Drone gyro data: gyro_x(pitch), gyro_y(roll), gyro_z(yaw): {gyro_x}, {gyro_y}, {gyro_z}")
        
        # calculate errors - diff between the actual rate of change in that axis (gyro_*) and the desired rate of change in that axis (input_* * max_rate_*)
        error_rate_roll:float = (input_roll * max_rate_roll) - gyro_y
        error_rate_pitch:float = (input_pitch * max_rate_pitch) - gyro_x
        error_rate_yaw:float = (input_yaw * max_rate_yaw) - gyro_z
        logger.info(f"error_rate_roll: {error_rate_roll}")
        logger.info(f"error_rate_pitch: {error_rate_pitch}")
        logger.info(f"error_rate_yaw: {error_rate_yaw}")
        
        roll_calculated_PID = roll_pid.calculate(error_rate_roll)
        pitch_calculated_PID = pitch_pid.calculate(error_rate_pitch)
        yaw_calculated_PID = yaw_pid.calculate(error_rate_yaw)
        logger.info(f"roll_calculated_PID: {roll_calculated_PID}")
        logger.info(f"pitch_calculated_PID: {pitch_calculated_PID}")
        logger.info(f"yaw_calculated_PID: {yaw_calculated_PID}")
        
        # calculate throttle values
        t1:float = adj_throttle + pitch_calculated_PID - roll_calculated_PID - yaw_calculated_PID
        t2:float = adj_throttle - pitch_calculated_PID - roll_calculated_PID + yaw_calculated_PID
        t3:float = adj_throttle + pitch_calculated_PID + roll_calculated_PID + yaw_calculated_PID
        t4:float = adj_throttle - pitch_calculated_PID + roll_calculated_PID - yaw_calculated_PID
        logger.info("Updating Motor Thrust")
        logger.info(f"t1: {t1}, t2: {t2}, t3: {t3}, t4: {t4}")
        
        # Adjust throttle according to input
        motor_to_speed_dict = {
            1: t1,
            2: t2,
            3: t3,
            4: t4
        }
        drone.start_motors(motor_to_speed_dict)
                    
        elapsed = perf_counter() - loop_start
        remaining_time = cycle_time_seconds - elapsed
        if remaining_time > 0:
            logger.info(f"going to sleep: {remaining_time} seconds")
            sleep(remaining_time)
        else:
            logger.warning("Cycle overran by {:.2f} ms".format(abs(remaining_time)*1000))

except BaseException as e:
    logger.error(f"An exception occurred: {e}")
    logger.error(traceback.format_exc())
    
finally:
    drone.turn_off_all()
    logger.info("Turning motor off")
    
