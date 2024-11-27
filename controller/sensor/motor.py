# Interface for the T-Motor F80 PRO KV2200

import sensor.PCA as PCA
from constant import MIN_MOTOR_FREQ_HZ, MAX_MOTOR_FREQ_HZ, TCA_MOTOR_CHANNEL
import utils as utils
from time import sleep
import sensor.TCA as TCA

def select():
    TCA.channel_select(TCA_MOTOR_CHANNEL)


def arm_all():
    PCA.init()

    motors = [1, 2, 3, 4]
    
    min_pwm = utils.us_to_pwm(MIN_MOTOR_FREQ_HZ)
    max_pwm = utils.us_to_pwm(MAX_MOTOR_FREQ_HZ)
    
    for motor in motors:
        print(f"Arming motor: {motor}")
        sleep(0.5)
        PCA.set_pwm(motor, 0, min_pwm)
        sleep(1)
        PCA.set_pwm(motor, 0, max_pwm)
        sleep(1)
        PCA.set_pwm(motor, 0, min_pwm)
        sleep(0.5)
        
    print("ARMING SEQUENCE DONE")

def start_motor(motor, speed_us):
    # Don't need to clamp speed, because it is normalize
    
    speed_pwm = utils.us_to_pwm(speed_us)

    PCA.set_pwm(motor, 0, speed_pwm)


def clamp_motor_frequency(value_us: int) -> int:
    if value_us > MAX_MOTOR_FREQ_HZ:
        return MAX_MOTOR_FREQ_HZ
    elif value_us < MIN_MOTOR_FREQ_HZ:
        return MIN_MOTOR_FREQ_HZ
    return value_us

    
