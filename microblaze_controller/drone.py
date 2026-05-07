
import json
from microblaze_controller.constant import *
from microblaze_controller.microblaze import mb_instance
from common.zeroMQManager import zeroMQServer
import logging

# the smallest positive representable value is 2^{-16} (about 0.0000152)
init_gains = [
    # ROLL
    int(0.008 * Q16_ONE),   # Kp (Proportional)
    int(0.1 * Q16_ONE),    # Ki (Integral)
    int(0.0001 * Q16_ONE),  # Kd (Derivative)

    # PITCH
    int(0.008 * Q16_ONE),   # Kp
    int(0.2 * Q16_ONE),    # Ki 0.12
    int(0.0002 * Q16_ONE),  # Kd

    # YAW
    int(0.015 * Q16_ONE),   # Kp
    int(0.08 * Q16_ONE),    # Ki
    int(0.0 * Q16_ONE),    # Kd
]

throttle_idle:float = 0.2 # the minumum throttle needed to apply to the four motors for them all to spin up, but not provide lift (idling on the ground). 
throttle_max:float = 0.4 # Max throttle that can be applied by user
throttle_range:float = throttle_max - throttle_idle

class Drone:
    def __init__(self):
        self.logger = logging.getLogger()
        self.input_throttle:float = 0.0  # between 0.0 and 1.0
        self.input_pitch:float = 0.0 # between -1.0 and 1.0
        self.input_roll:float = 0.0  # between -1.0 and 1.0
        self.input_yaw:float = 0.0   # between -1.0 and 1.0
        self.logger.info("Initializing sensors...")
        self.init_sensors() 
        self.logger.info("Sensors initialized.")
        self.logger.info("Setting initial PID gains...")
        self.set_gains()
        self.logger.info("Gains set.")
        
    def init_sensors(self):
        mb_instance.mailbox_write_cmd(CMD_INIT)
        mb_instance.wait_for_cmd(CMD_INIT)
        
    def update_input_throttle_callback(self, json_message):
        message = json.loads(json_message.decode())
        print("Received throttle input: {}".format(message["data"]))
        if (message["data"] >= 0):
            self.input_throttle = message["data"]
            self.set_throttle()
            
    def set_gains(self):
        for i, gain in enumerate(init_gains):
            mb_instance.mailbox_write_data(i, gain)
        mb_instance.mailbox_write_cmd(CMD_SET_GAINS)
        mb_instance.wait_for_cmd(CMD_SET_GAINS)

    def set_roll(self):
        mb_instance.mailbox_write_data(0, int(self.input_roll * Q16_ONE))

    def set_pitch(self):
        mb_instance.mailbox_write_data(1, int(self.input_pitch * Q16_ONE))
        
    def set_yaw(self):
        mb_instance.mailbox_write_data(2, int(self.input_yaw * Q16_ONE))

    def set_throttle(self):
        mb_instance.mailbox_write_data(3, int((throttle_idle + (throttle_range * self.input_throttle)) * Q16_ONE))

    def start(self):
        self.set_roll()
        self.set_pitch()
        self.set_yaw()
        self.set_throttle()
        mb_instance.mailbox_write_cmd(CMD_START_PID)
        self.logger.info("PID loop running.")

    def stop(self):
        mb_instance.mailbox_write_cmd(CMD_STOP_PID)
        mb_instance.wait_for_cmd(CMD_STOP_PID)
        
    def read_gyro(self):
        gx = mb_instance.mailbox_read_signed_q16(8)
        gy = mb_instance.mailbox_read_signed_q16(9)
        gz = mb_instance.mailbox_read_signed_q16(10)
        zeroMQServer.send("gyro", {"x": gx, "y": gy, "z": gz})
        return gx, gy, gz
    
    def read_accel(self):
        ax = mb_instance.mailbox_read_signed_q16(5)
        ay = mb_instance.mailbox_read_signed_q16(6)
        az = mb_instance.mailbox_read_signed_q16(7)
        zeroMQServer.send("accel", {"x": ax, "y": ay, "z": az})
        return ax, ay, az
                
    def read_debug(self):
        gx, gy, gz = self.read_gyro()
        print("Gyro rates: gx={:.3f}, gy={:.3f}, gz={:.3f}".format(gx, gy, gz))
                
        ax, ay, az = self.read_accel()
        print("Accel values: ax={:.3f}, ay={:.3f}, az={:.3f}".format(ax, ay, az))
        
        elapsed_time_ms = mb_instance.mailbox_read_unsigned(4)
        print("Elapsed time for PID loop: {} ms".format(elapsed_time_ms))
        