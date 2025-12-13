
import json
from microblaze_controller.constant import *
from microblaze_controller.microblaze import mb_instance

init_gains = [
    int(1.0 * Q16_ONE),  # Roll Kp
    int(0.0 * Q16_ONE),  # Roll Ki
    int(0.0 * Q16_ONE),  # Roll Kd
    int(1.0 * Q16_ONE),  # Pitch Kp
    int(0.0 * Q16_ONE),  # Pitch Ki
    int(0.0 * Q16_ONE),  # Pitch Kd
    int(1.0 * Q16_ONE),  # Yaw Kp
    int(0.0 * Q16_ONE),  # Yaw Ki
    int(0.0 * Q16_ONE),  # Yaw Kd
]

throttle_idle:float = 0.1 # the minumum throttle needed to apply to the four motors for them all to spin up, but not provide lift (idling on the ground). 
throttle_max:float = 0.35 # Max throttle that can be applied by user
throttle_range:float = throttle_max - throttle_idle

class Drone:
    def __init__(self):
        self.input_throttle:float = 0.0  # between 0.0 and 1.0
        self.input_pitch:float = 0.0 # between -1.0 and 1.0
        self.input_roll:float = 0.0  # between -1.0 and 1.0
        self.input_yaw:float = 0.0   # between -1.0 and 1.0
        print("Setting initial PID gains...")
        self.set_gains()
        print("Gains set. Initializing sensors...")
        self.init_sensors()
        print("Sensors initialized.")
        
    def init_sensors(self):
        mb_instance.mailbox_write_cmd(CMD_INIT)
        mb_instance.wait_for_cmd(CMD_INIT)
        
    def update_input_throttle_callback(self, json_message):
        message = json.loads(json_message.decode())
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
        print("PID loop running.")

    def stop(self):
        mb_instance.mailbox_write_cmd(CMD_STOP_PID)
        mb_instance.wait_for_cmd(CMD_STOP_PID)
                
    def read_debug(self):
        gx = mb_instance.mailbox_read_signed_q16(8)
        gy = mb_instance.mailbox_read_signed_q16(9)
        gz = mb_instance.mailbox_read_signed_q16(10)
        print("Gyro rates: gx={:.3f}, gy={:.3f}, gz={:.3f}".format(gx, gy, gz))
                
        t1 = mb_instance.mailbox_read_signed_q16(4)
        t2 = mb_instance.mailbox_read_signed_q16(5)
        t3 = mb_instance.mailbox_read_signed_q16(6)
        t4 = mb_instance.mailbox_read_signed_q16(7)
        print("Motor throttles: {:.3f} {:.3f} {:.3f} {:.3f}".format(t1, t2, t3, t4))
        
        elapsed_time_ms = mb_instance.mailbox_read_unsigned(11)
        print("Elapsed time for PID loop: {} ms".format(elapsed_time_ms))

            
            
            
            
